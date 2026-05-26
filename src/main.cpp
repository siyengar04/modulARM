#include "G2MotorDriver.h"

// ===================== MOTOR DRIVER PINS =====================
// IMPORTANT: Do NOT use default constructor, because default DIR pin = 2,
// which conflicts with encoderPinA = 2.
//
// Wiring:
// DIR  -> 7
// PWM  -> 9   // Timer1, 20 kHz on Arduino Uno with this library
// SLP  -> 4
// FLT  -> 6
// CS   -> A0
const uint8_t MD_DIR = 7;
const uint8_t MD_PWM = 11;
const uint8_t MD_SLP = 4;
const uint8_t MD_FLT = 6;
const uint8_t MD_CS = A0;

// Limit switch pins 
const uint8_t limitSwitchPin1 = 4;
const uint8_t limitSwitchPin2 = 9;

G2MotorDriver24v13 md(MD_DIR, MD_PWM, MD_SLP, MD_FLT, MD_CS);

// ===================== ENCODER PINS =====================
const uint8_t encoderPinA = 2;
const uint8_t encoderPinB = 3;

volatile long encoderCount = 0;

// ===================== ENCODER CONSTANTS =====================
// If 64 already means quadrature counts per motor rev, keep 64.
// If 64 means pulses per channel per motor rev, use 64*4 = 256.
const float gearRatio = 270.0;
const float countsPerMotorRev = 64.0;
const float countsPerOutputRev = gearRatio * countsPerMotorRev;

// ===================== CONTROL TIMING =====================
const unsigned long controlPeriodMicros = 2000; // mus = 500 Hz
unsigned long lastControlMicros = 0;

// ===================== PID GAINS =====================
float Kp = 500.0;
float Ki = 300.0;
float Kd = 2.0;

// ===================== SETPOINT =====================
float theta_des = PI; // rad

// ===================== PID STATES =====================
float e_int = 0.0;
float e_prev = 0.0;

// Integral anti-windup limit
const float eIntMax = 50.0;

// ===================== MEASURED STATES =====================
float theta_meas = 0.0;
float omega_meas = 0.0;

long prevCount = 0;

// ===================== SERIAL PRINTING =====================
int printCounter = 0;
const int printEvery = 30; // print every 30 control loops = 500 Hz

// ===================== ENCODER ISR =====================
void doEncoderA()
{
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

  if (A == B)
    encoderCount++;
  else
    encoderCount--;
}

void doEncoderB()
{
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);

  if (A == B)
    encoderCount--;
  else
    encoderCount++;
}

// ===================== FAULT CHECK =====================
void stopIfFault()
{
  if (md.getFault())
  {
    md.setSpeed(0);
    md.Sleep();

    Serial.println("Motor driver fault detected. Driver disabled.");

    while (1)
    {
      delay(100);
    }
  }
}

// ===================== MOTOR COMMAND =====================
void setMotorCommand(float u)
{
  int u_cmd = (int)constrain(u, -400.0, 400.0);
  md.setSpeed(u_cmd);
}

// ===================== READ ENCODER =====================
void readEncoderState(float dt)
{
  long count;

  noInterrupts();
  count = encoderCount;
  interrupts();

  theta_meas = count * 2.0 * PI / countsPerOutputRev;

  long dCount = count - prevCount;
  omega_meas = dCount * 2.0 * PI / (countsPerOutputRev * dt);

  prevCount = count;
}

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

  md.init();
  // Keep motor driver disabled during startup
  md.Sleep();
  delay(10);
  // Force command to zero before enabling the driver
  md.setSpeed(0);
  delay(10);
  // Now enable the driver
  md.Wake();
  delay(10);
  // Calibrate current sensor at zero command
  md.calibrateCurrentOffset();
  delay(10);

  lastControlMicros = micros();

  Serial.println("theta_des,theta_meas,omega_meas,u_cmd");
}

// ===================== LOOP =====================
void loop()
{
  unsigned long now = micros();

  if ((unsigned long)(now - lastControlMicros) >= controlPeriodMicros)
  {
    float dt = (now - lastControlMicros) * 1e-6;
    lastControlMicros = now;

    readEncoderState(dt);
    stopIfFault();

    // Position error
    float e = theta_des - theta_meas;

    // Integral term with anti-windup clamp
    e_int += e * dt;
    e_int = constrain(e_int, -eIntMax, eIntMax);

    // Derivative term:
    // For constant position setpoint, de/dt = -omega_meas.
    // This is cleaner than differentiating noisy position error.
    // float e_dot = -omega_meas;
    float e_dot = (e - e_prev) / dt;
    e_prev = e;

    float u = Kp * e + Ki * e_int + Kd * e_dot;

    // Saturate command
    float u_sat = constrain(u, -400.0, 400.0);

    // Extra anti-windup: stop integrating if saturated in the same direction
    if ((u != u_sat) && ((e > 0 && u > 0) || (e < 0 && u < 0)))
    {
      e_int -= e * dt;
      e_int = constrain(e_int, -eIntMax, eIntMax);
    }

    setMotorCommand(u_sat);

    // Print at lower rate to avoid slowing control loop
    printCounter++;
    if (printCounter >= printEvery)
    {
      printCounter = 0;

      Serial.print(theta_des, 4);
      Serial.print(",");
      Serial.print(theta_meas, 4);
      Serial.print(",");
      Serial.print(omega_meas, 4);
      Serial.print(",");
      Serial.println(u_sat, 2);
    }
  }
}