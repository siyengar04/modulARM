#include "Arduino.h"
#include "G2MotorDriver.h"
// #include "avr8-stub.h"

// driver 2 pins
const uint8_t MD_DIR2 = 7;
const uint8_t MD_PWM2 = 2;
const uint8_t MD_SLP2 = 4;
const uint8_t MD_FLT2 = 30;
const uint8_t MD_CS2 = A1;

// Limit switch pins
const uint8_t limitSwitchPin2B = 20;
const uint8_t limitSwitchPin2A = 8;

// instantiate motor driver
G2MotorDriver18v17 md2(MD_DIR2, MD_PWM2, MD_SLP2, MD_FLT2, MD_CS2);

// initialize encoder
volatile int encoderPosition = 0;
const uint8_t encoder2PinA = 19;
const uint8_t encoder2PinB = 18;
volatile float encoderAngleB = 0.0;
volatile int encoderCount2 = 0;
float previousBAngle = 0.0;
float encoderBAngle = 0.0;
float encoderBSpeed = 0.0;
float currentSpeed = 0.0;
long lastSpeedTime = 0;
int lastEncoderPosition = 0;

// motor variables
const float maxTheta2 = 180.0;
const float home = 0.0;
const float maxCurrent = 0.6;

// system variables
volatile bool calibrated = false;
volatile bool systemLock = false;
unsigned long now = micros();

const float gearRatio = 270.0;
const float countsPerMotorRev = 64.0;
const float countsPerOutputRev = gearRatio * countsPerMotorRev;

// ===================== CONTROL TIMING =====================
const unsigned long controlPeriodMicros = 2000; // mus = 500 Hz
unsigned long lastControlMicros = 0;

// ===================== PID GAINS =====================
// // motor 2 testing
// float Kp2 = 500.0;
// float Ki2 = 300.0;
// float Kd2 = 2.0;

float Kp2 = 200.0;
float Ki2 = 50.0;
float Kd2 = 2.0;

// ===================== SETPOINT =====================
float thetaDes2 = 0.0;

// ===================== PID STATES =====================
float e_int2 = 0.0;
float e_prev2 = 0.0;

// Integral anti-windup limit
const float eIntMax2 = 50.0;

// ===================== MEASURED STATES =====================
float thetaMeas2 = 0.0;
float omegaMeas2 = 0.0;

long prevCount2 = 0;

// ===================== SERIAL PRINTING =====================
int printCounter = 0;
const int printEvery = 150; // print every 150 control loops = 100 Hz
float rad2Deg(float rad)
{
  return rad * (180.0 / PI);
}
// ==========================  Prototypes  ================================
// void readEncoderState(float dt2, volatile long &encoderCount, long &prevCount, float &thetaMeas2, float &omega_meas);

// handle interrupts for reading encoders and limit switches
void doEncoder2A()
{
  bool A = digitalRead(encoder2PinA);
  bool B = digitalRead(encoder2PinB);

  if (A == B)
    encoderCount2++;
  else
    encoderCount2--;
}

void doEncoder2B()
{
  bool A = digitalRead(encoder2PinA);
  bool B = digitalRead(encoder2PinB);

  if (A == B)
    encoderCount2--;
  else
    encoderCount2++;
}

void readEncoderState(float dt)
{
  long count;

  noInterrupts();
  count = encoderCount2;
  interrupts();

  thetaMeas2 = rad2Deg(count * 2.0 * PI / countsPerOutputRev);
  // thetaMeas2 = 180.0 * count * 2.0 / countsPerOutputRev;

  long dCount = count - prevCount2;
  omegaMeas2 = dCount * 2.0 * PI / (countsPerOutputRev * dt);

  prevCount2 = count;
}

void doLimit2()
{
  md2.setSpeed(0);
  systemLock = true;
}
// void calcSpeed(float dt)
// {

//   unsigned long currentTime = millis();

//   if (dt >= 10) // Calculate speed every 10ms for PID input freshness
//   {
//     long deltaPulses = encoderPosition - lastEncoderPosition;
//     float deltaAngle = (deltaPulses * 360.0) / pulsesPerRevolution;
//     currentSpeed = (deltaAngle * 1000.0) / dt;
//     lastEncoderPosition = encoderPosition;
//     lastSpeedTime = currentTime;
//   }
// }
// accept user input
void set_theta_des()
{

  // Prompt User
  Serial.println("==============================================");
  Serial.print("Enter desired position in degrees (0.00 to ");
  Serial.print(maxTheta2, 2);
  Serial.println("):");
  Serial.println("==============================================");

  Serial.setTimeout(10000);

  float new_theta2 = Serial.parseFloat();
  thetaDes2 = constrain(new_theta2, home, maxTheta2);
  Serial.print("Target position successfully set to: ");
  Serial.println(thetaDes2, 4);
  while (Serial.available() && Serial.peek() != '\n')
    Serial.read(); // Clear buffer

  return;
}

void waitForUserStart()
{
  Serial.println("\n==============================================");
  Serial.println("Do you want it to start? (Y/N)");
  Serial.println("==============================================");

  while (true)
  {
    if (Serial.available() > 0)
    {
      char response = Serial.read();

      // Check for yes (both uppercase and lowercase)
      if (response == 'Y' || response == 'y')
      {
        Serial.println("Input Y detected.");
        if (calibrated)
        {
          set_theta_des();

          Serial.println("----------------------------------------------");
          Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
          Serial.println("----------------------------------------------");

          delay(10);
          while (Serial.available() > 0)
          {
            Serial.read();
          }
          while (true)
          {
            if (Serial.available() > 0)
            {
              char c = Serial.read();
              if (c == '\n' || c == '\r')
              {
                break; // Exit and return control to setup/loop
              }
            }
          }

          delay(10);
          while (Serial.available() > 0)
          {
            Serial.read();
          }

          Serial.println("Motor engaged! Starting control loop...");
        }
        return; // Exit the loop and start the system
      }
      // Check for no (both uppercase and lowercase)
      else if (response == 'N' || response == 'n')
      {
        Serial.println("System remains idle. Waiting for 'Y' to start...");
      }
      // Ignore trailing newline or carriage return characters from entering again
      else if (response == '\n' || response == '\r')
      {
        continue;
      }
      // Handle invalid inputs
      else
      {
        Serial.print("Invalid input '");
        Serial.print(response);
        Serial.println("'. Please enter Y or N.");
      }
    }
  }
}

void keyPress()
{
  if (Serial.available() > 0)
  {
    md2.setSpeed(0);
    md2.Sleep();

    set_theta_des();

    Serial.println("----------------------------------------------");
    Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
    Serial.println("----------------------------------------------");

    delay(10);
    while (Serial.available() > 0)
    {
      Serial.read();
    }
    while (true)
    {
      if (Serial.available() > 0)
      {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
          break; // Exit and return control to setup/loop
        }
      }
    }

    delay(10);
    while (Serial.available() > 0)
    {
      Serial.read();
    }

    Serial.println("Motor engaged! Starting control loop...");

    // e_int2 = 0.0;

    // unsigned long now_pre = micros();
    // float dt_pre = (now_pre - lastControlMicros) * 1e-6;
    // if (dt_pre <= 0)
    //   dt_pre = 0.002;

    // e_prev2 = thetaDes2 - thetaMeas2;

    md2.Wake();
    // lastControlMicros = micros();
  }
}

// stop for fault, currently unassigned
// void stopIfFault()
// {
//   if (md2.getFault())
//   {
//     md2.setSpeed(0);
//     md2.Sleep();
//     Serial.println("Motor driver fault detected. Driver disabled.");
//     while (1)
//     {
//       delay(100);
//     }
//   }
// }

// ===================== MOTOR COMMAND =====================
void setMotorCommand2(float u)
{
  int u_cmd = (int)constrain(u, -50.0, 50.0);
  md2.setSpeed(u_cmd);
}

// ===================== READ ENCODER =====================

// void readEncoderState(float dt2, volatile long &encoderCount, long &prevCount, float &thetaMeas2, float &omega_meas)
// {

//   long count;

//   noInterrupts();
//   count = encoderCount;
//   interrupts();

//   thetaMeas2 = count * 2.0 * PI / countsPerOutputRev;

//   long dCount = count - prevCount;
//   omega_meas = dCount * 2.0 * PI / (countsPerOutputRev * dt2);

//   prevCount = count;
// }

// ==================== PID Function ========================= Made separate for ease of use between motors
float calculatePID(float Kp, float Ki, float Kd, float dt)
{

  // calcSpeed();
  // Position error
  float e = thetaDes2 - thetaMeas2;
  // error wrapping
  // if (e > 180)
  //   e -= 360;
  // if (e < -180)
  //   e += 360;
  // Integral term with anti-windup clamp
  e_int2 += e * dt;
  e_int2 = constrain(e_int2, -eIntMax2, eIntMax2);

  // Derivative term:
  float e_dot2 = (e - e_prev2) / dt;
  e_prev2 = e;

  float u = Kp * e + Ki * e_int2 + Kd * e_dot2;

  // Saturate command
  float u_sat = constrain(u, -50.0, 50.0);

  // Extra anti-windup: stop integrating if saturated in the same direction
  if ((u != u_sat) && ((e > 0 && u > 0) || (e < 0 && u < 0)))
  {
    e_int2 -= e * dt;
    e_int2 = constrain(e_int2, -eIntMax2, eIntMax2);
  }

  // Current Sense stuff
  //  float currentB = md2.getCurrentMilliamps();

  // if (((currentB / 2800) >= maxCurrent) && (omega_measB <= 3))
  // {
  //   return 0;
  // }
  if (digitalRead(limitSwitchPin2A) == LOW)
  {
    systemLock = true;
    return 0;
  }
  // Boundary check
  if (((thetaMeas2 >= maxTheta2) && u_sat > 0) || (thetaMeas2 <= 0.14 && u_sat < 0))
  {
    return 0;
  }

  return u_sat;
}

// ===================== SETUP =====================
void setup()
{
  lastSpeedTime = millis();
  lastEncoderPosition = encoderPosition;
  // debug_init();
  Serial.begin(115200);
  // to prevent motor kick on startup write PWM to low before starting the driver
  digitalWrite(MD_PWM2, LOW);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(limitSwitchPin2B, INPUT_PULLUP);
  pinMode(limitSwitchPin2A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2B, CHANGE);

  md2.init();
  // Keep motor driver disabled during startup
  md2.Sleep();
  delay(10);
  // Force command to zero before enabling the driver
  md2.setSpeed(0);
  delay(10);
  // Now enable the driver
  waitForUserStart();
  md2.Wake();
  delay(10);
  // Calibrate current sensor at zero command
  md2.calibrateCurrentOffset();
  delay(10);

  lastControlMicros = micros();

  Serial.println("Homing motor B...");
  do
  {
    md2.setSpeed(-50);
    // delay(100);
  } while (digitalRead(limitSwitchPin2B) == HIGH);
  md2.setSpeed(0);
  encoderCount2 = 0;
  thetaMeas2 = 0.0;
  e_int2 = 0.0;
  e_prev2 = 0.0;
  calibrated = true;
  Serial.println("Homing complete.");

  // 2. Put the driver chip into low-power sleep mode
  delay(50);
  md2.Sleep();
  waitForUserStart();
  md2.Wake();
  md2.setSpeed(0);

#if defined(__AVR_ATmega2560__)
  EIFR = bit(INTF0) | bit(INTF1);
#endif

  attachInterrupt(digitalPinToInterrupt(limitSwitchPin2B), doLimit2, FALLING);

  lastControlMicros = micros();
}

// ===================== LOOP =====================
void loop()
{
  // After Calibration, if button hits the limit switch the program stops motors and requires reset
  if (systemLock)
  {
    // stop motor B
    md2.setSpeed(0);
    md2.setBrake(400);
    md2.Sleep();
    Serial.println("Limit switch hit! Halting.");
    while (true)
    {
    }
  }

  keyPress();

  unsigned long now = micros();

  if ((unsigned long)(now - lastControlMicros) >= controlPeriodMicros)
  {
    float dt = (now - lastControlMicros) * 1e-6;
    lastControlMicros = now;
    readEncoderState(dt);

    // stopIfFault();
    // calcSpeed(dt);
    float u_sat2 = calculatePID(Kp2, Ki2, Kd2, dt);
    setMotorCommand2(u_sat2);

    // Print at lower rate to avoid slowing control loop
    printCounter++;
    if (printCounter >= printEvery)
    {
      printCounter = 0;

      Serial.println("Motor2:");
      Serial.print(thetaDes2, 4);
      Serial.print(",");
      Serial.print(thetaMeas2, 4);
      Serial.print(",");
      Serial.print(omegaMeas2, 4);
      Serial.print(",");
      Serial.println(u_sat2, 2);
      Serial.print("Raw Encoder Count: ");
      Serial.print(encoderCount2);
      Serial.print("\n");
    }
  }
}
