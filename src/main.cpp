#include "G2MotorDriver.h"

// Driver config

//driver 1
const uint8_t MD_DIR = 7;
const uint8_t MD_PWM = 11;
const uint8_t MD_SLP = 4;
const uint8_t MD_FLT = 6;
const uint8_t MD_CS = A0;

//driver 2 TODO
// const uint8_t MD_DIR2 = 7;
// const uint8_t MD_PWM2 = 11;
// const uint8_t MD_SLP2 = 4;
// const uint8_t MD_FLT2 = 6;
// const uint8_t MD_CS2 = A0;

// Limit switch pins 
const uint8_t limitSwitchPin1 = 20;
const uint8_t limitSwitchPin2 = 21;

const float maxTheta1 = PI;
const float maxTheta2 = PI;

//interrupt flags and nested delay for interrupt var
volatile bool calibrated = false;
volatile bool systemLock = false;

G2MotorDriver24v13 md(MD_DIR, MD_PWM, MD_SLP, MD_FLT, MD_CS);
// G2MotorDriver24v13 md2(MD_DIR2, MD_PWM2, MD_SLP2, MD_FLT2, MD_CS2);

// encoder config
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
float theta_des = 2.0; // rad

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

void doLimit1()
{
  md.setBrake(400); // Full brake
  
  if (calibrated && !systemLock)            //Flags the program to not move further when the button is pressed
  {
    systemLock = true;
    Serial.println("Interrupt Triggered: movement is locked until arm moves in bounds.");
    return;
  }
}

void doLimit2()
{
  // md2.setBrake(400); // Full brake
  // Handle limit switch 2 interrupt
}
// Forces system to wait for user input to start PID.
void waitForUserStart() {
  Serial.println("\n==============================================");
  Serial.println("Do you want it to start? (Y/N)");
  Serial.println("==============================================");

  while (true) {
    if (Serial.available() > 0) {
      char response = Serial.read();

      // Check for yes (both uppercase and lowercase)
      if (response == 'Y' || response == 'y') {
        Serial.println("Input Y detected. Starting...");
        return; // Exit the loop and start the system
      }
      // Check for no (both uppercase and lowercase)
      else if (response == 'N' || response == 'n') {
        Serial.println("System remains idle. Waiting for 'Y' to start...");
      }
      // Ignore trailing newline or carriage return characters from entering again
      else if (response == '\n' || response == '\r') {
        continue;
      }
      // Handle invalid inputs
      else {
        Serial.print("Invalid input '");
        Serial.print(response);
        Serial.println("'. Please enter Y or N.");
      }
    }
  }
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
  if (systemLock) 
  {
      // If locked against Switch 1 (Left), unlock only if driving right (positive)
      // If out of bounds on the right, unlock if moving left
    if ((digitalRead(limitSwitchPin1) == LOW && u > 0)||(theta_meas >= maxTheta1 && u < 0)) {
      systemLock = false;
      Serial.println("Motor moving within bounds. Movement unlocked.");
    }
    else 
    {
      md.setBrake(400);
      return;
    }
  }
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
  //to prevent motor kick on startup write PWM to low before starting the driver
  digitalWrite(MD_PWM, LOW); 
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(limitSwitchPin1, INPUT_PULLUP);
  pinMode(limitSwitchPin2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(limitSwitchPin2), doLimit2, FALLING);

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
  waitForUserStart();
  md.Wake();
  delay(10);
  // Calibrate current sensor at zero command
  md.calibrateCurrentOffset();
  delay(10);

  lastControlMicros = micros();

  Serial.println("theta_des,theta_meas,omega_meas,u_cmd");
  Serial.println("Homing motor...");
  do {
    md.setSpeed(-200); // Move towards limit switch at moderate speed
    delay(100);
  } while (digitalRead(limitSwitchPin1) == HIGH); // Wait until limit switch is triggered
  md.setSpeed(0);
  encoderCount = 0;
  theta_meas = 0.0; // Set current position as zero reference
  e_int = 0.0;
  e_prev = 0.0;
  calibrated = true;
  Serial.println("Homing complete.");

  // 2. Put the driver chip into low-power sleep mode
  delay(50);
  md.Sleep();
  waitForUserStart();
  md.Wake();
  md.setSpeed(0);

  attachInterrupt(digitalPinToInterrupt(limitSwitchPin1), doLimit1, FALLING);
  
  lastControlMicros = micros();
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

    if (theta_meas >= maxTheta1 && !systemLock)  //prevents movement if passed threshold by triggering systemLock
    {
      doLimit1();
    }
    setMotorCommand(u_sat);
    
    // Print at lower rate to avoid slowing control loop
    printCounter++;
    if (printCounter >= printEvery)
    {
      printCounter = 0;
      Serial.println(encoderCount);
      Serial.print(",");
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
