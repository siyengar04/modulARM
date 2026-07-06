#include "Modified_G2MotorDriver.h"

// Driver config
// driver 1
const uint8_t MD_DIR1 = 7; // 5;  //7
const uint8_t MD_PWM1 = 3; // 12; //2
const uint8_t MD_SLP1 = 4;
const uint8_t MD_FLT1 = 30;
const uint8_t MD_CS1 = A1;

// driver 2
const uint8_t MD_DIR2 = 7; // 5;  //7
const uint8_t MD_PWM2 = 2; // 12; //2
const uint8_t MD_SLP2 = 4;
const uint8_t MD_FLT2 = 30;
const uint8_t MD_CS2 = A1;

// Limit switch pins
const uint8_t limitSwitchPin2B = 20;
const uint8_t limitSwitchPin2A = 8;

const float maxTheta2 = 2.62;
const float home2 = 0.26;

volatile bool calibrated = false;
volatile bool systemLock = false;

// G2MotorDriver18v17 md1(MD_DIR1, MD_PWM1, MD_SLP1, MD_FLT1, MD_CS1);

G2MotorDriver18v17 md2(MD_DIR2, MD_PWM2, MD_SLP2, MD_FLT2, MD_CS2);

const uint8_t encoder2PinA = 18;
const uint8_t encoder2PinB = 19;

volatile long encoderCountB = 0;

// ===================== ENCODER CONSTANTS =====================
// If 64 already means quadrature counts per motor rev, keep 64.
// If 64 means pulses per channel per motor rev, use 64*4 = 256.
const float gearRatio = 270.0;
const float countsPerMotorRev = 64.0;
const float countsPerOutputRev = gearRatio * countsPerMotorRev;

// ===================== CONTROL TIMING =====================
const unsigned long controlPeriodMicros = 1000; // mus = 500 Hz
unsigned long lastControlMicros = 0;

// ===================== PID GAINS =====================
// motor 1
float Kp1 = 200.0; // 500
float Ki1 = 200.0; // 300
float Kd1 = .1;    // 2

// motor 2
float Kp2 = 700.0; // 500
float Ki2 = 200.0; // 300
float Kd2 = .1;    // 2

// ===================== SETPOINT =====================
// float thetaDes2B = PI;

const int MAX_STATES = 10; // max allowed number of states
float thetaDes2B[MAX_STATES];
int num_of_pos = 1; // Desired number of states

int completed = 0; // Tracks current state

// ===================== PID STATES =====================
float e_int2 = 0.0;
float e_prev2 = 0.0;

// Integral anti-windup limit
const float eIntMax = 50.0;

// ===================== MEASURED STATES =====================
float theta_measB = 0.0;
float omega_measB = 0.0;

long prevCountB = 0;

// ===================== SERIAL PRINTING =====================
int printCounter = 0;
const int printEvery = 300; // print every 30 control loops = 500 Hz

// ==========================  Prototypes  ================================
void readEncoderState(float dt, volatile long encoderCount, long &prevCount, float &theta_meas, float &omega_meas);

// ===================== ENCODER ISR =====================
void doEncoderC()
{
  bool A = digitalRead(encoder2PinA);
  bool B = digitalRead(encoder2PinB);

  if (A == B)
    encoderCountB++;
  else
    encoderCountB--;
}

void doEncoderD()
{
  bool A = digitalRead(encoder2PinA);
  bool B = digitalRead(encoder2PinB);

  if (A == B)
    encoderCountB--;
  else
    encoderCountB++;
}

void doLimit2()
{
  // Stops motor and triggers flag to stop all operation on the next loop
  if (omega_measB < 0)
  {
    md2.setSpeed(0);
    systemLock = true;
  }
}

// ===================== USER INPUT FUNCTIONS =====================  //ONLY HANDLES ONE MOTOR

void set_thetaDes2(float &thetaDes2, float maxTheta, int pos)
{
  // Prompt User for desired position
  Serial.println("==============================================");
  Serial.print("Enter desired position in radians (0.00 to ");
  Serial.print(maxTheta, 2);
  Serial.print(") ");
  Serial.print("for position ");
  Serial.print(pos);
  Serial.println(":");
  Serial.println("==============================================");

  // Reads user input (float)
  Serial.setTimeout(10000);
  float new_theta = Serial.parseFloat();

  thetaDes2 = constrain(new_theta, home2, maxTheta2);

  // Print confirmation
  Serial.print("Target position for state ");
  Serial.print(pos);
  Serial.print(" successfully set to: ");
  Serial.print("Target position successfully set to: "); // comment out when switching to array
  Serial.println(thetaDes2, 4);

  return;
}

void waitForUserStart()
{
  // Prompt user to start current process
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
          // If not for calibration, asks user to set up the states for the program
          // Prompt user for how many positions
          Serial.println("\n==============================================");
          Serial.print("How many positions do you want? (Type an integer between 1 and ");
          Serial.print(MAX_STATES);
          Serial.println(")");
          Serial.println("==============================================");

          // Reads user input (int)
          Serial.setTimeout(10000);
          num_of_pos = Serial.parseInt();
          num_of_pos = constrain(num_of_pos, 1, MAX_STATES);

          // Prompt user to set the position for the desired number of states
          for (int i = 0; i < num_of_pos; i++)
          {
            set_thetaDes2(thetaDes2B[i], maxTheta2, i);
          }

          completed = 0;

          // Prints an array with all of the states for visual confirmation
          Serial.println("All states are the following: ");
          Serial.print("[");
          for (int i = 0; i < num_of_pos - 1; i++)
          {
            Serial.print(thetaDes2B[i]);
            Serial.print(", ");
          }
          Serial.print(thetaDes2B[num_of_pos - 1]);
          Serial.println("]");

          // Waits for user to actually start the program
          // Allows for wait time between setup and operation
          Serial.println("----------------------------------------------");
          Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
          Serial.println("----------------------------------------------");

          // Clear serial buffer
          delay(10);
          while (Serial.available() > 0)
          {
            Serial.read();
          }

          // Checks for ENTER key
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

          // Clears buffer
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
    // Deactivates motor while setting up new states for safety
    md2.setSpeed(0);
    md2.Sleep();

    // Prompt user for how many positions and to set positions
    Serial.println("\n==============================================");
    Serial.print("How many positions do you want? (Type an integer between 1 and ");
    Serial.print(MAX_STATES);
    Serial.println(")");
    Serial.println("==============================================");

    // Reads user input (int)
    Serial.setTimeout(10000);
    num_of_pos = Serial.parseInt();
    num_of_pos = constrain(num_of_pos, 1, MAX_STATES);

    // Print confirmation
    Serial.print("Number of positions set to: ");
    Serial.println(num_of_pos);

    // Makes user choose the positions for the desired number of states
    for (int i = 0; i < num_of_pos; i++)
    {
      set_thetaDes2(thetaDes2B[i], maxTheta2, i);
    }

    // Reset to do proper counting of current state
    completed = 0;

    // Prints array with state positions for visual confirmation
    Serial.println("All states are the following: ");
    Serial.print("[");
    for (int i = 0; i < num_of_pos - 1; i++)
    {
      Serial.print(thetaDes2B[i]);
      Serial.print(", ");
    }
    Serial.print(thetaDes2B[num_of_pos - 1]);
    Serial.println("]");

    // Require user input to start the program again
    Serial.println("----------------------------------------------");
    Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
    Serial.println("----------------------------------------------");

    // Clear Serial buffer
    delay(10);
    while (Serial.available() > 0)
    {
      Serial.read();
    }

    // Waits for user to hit ENTER
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

    // CLear buffer
    delay(10);
    while (Serial.available() > 0)
    {
      Serial.read();
    }

    Serial.println("Motor engaged! Starting control loop...");
    // Reset integral value for the new state
    e_int2 = 0.0;

    // Read encoder to setup for new state
    unsigned long now_pre = micros();
    float dt_pre = (now_pre - lastControlMicros) * 1e-6;
    if (dt_pre <= 0)
      dt_pre = 0.002;
    readEncoderState(dt_pre, encoderCountB, prevCountB, theta_measB, omega_measB);

    // Prevents sudden jerking movement I think (may not be necessary)
    e_prev2 = thetaDes2B[0] - theta_measB;

    md2.Wake();
    lastControlMicros = micros();
  }
}

// ===================== FAULT CHECK =====================
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
void setMotorCommandB(float u)
{
  int u_cmd = (int)constrain(u, -400.0, 400.0);
  md2.setSpeed(u_cmd);
}

// ===================== READ ENCODER =====================
void readEncoderState(float dt, volatile long encoderCount, long &prevCount, float &theta_meas, float &omega_meas)
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

// ==================== PID Function ========================= Made separate for ease of use between motors
float calculatePID(float thetaDes2, float theta_meas, float maxTheta, float dt, float &e_int, float &e_prev, float Kp, float Ki, float Kd)
{
  // Position error
  float e = thetaDes2 - theta_meas;
  // Integral term with anti-windup clamp
  e_int += e * dt;
  e_int = constrain(e_int, -eIntMax, eIntMax);

  // Derivative term:
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

  if (digitalRead(limitSwitchPin2A) == LOW)
  {
    systemLock = true;
    return 0;
  }

  // Boundary check
  if (((theta_meas >= maxTheta) && u_sat > 0) || (theta_meas <= 0.05 && u_sat < 0))
  {
    return 0;
  }

  return u_sat;
}

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);

  // TODO: Get the PWM Signal Working (Needs to be 20kHz)
  // ===================== DONT WORK YET ========================================
  // Sets the PWM of timer 3 to 20kHz
  // Basically copy pasted the one in library  for timer 1
  // TCCR assignments are the waveform generator settings, should be ok to copy
  // ICR assignment sets the TOP value like in the motor driver
  // TCCR3A = 0b10101000;
  // TCCR3B = 0b00010001;
  // ICR3 = 400;
  // ===============================================================================

  // to prevent motor kick on startup write PWM to low before starting the driver
  digitalWrite(MD_PWM2, LOW);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(limitSwitchPin2B, INPUT_PULLUP);
  pinMode(limitSwitchPin2A, INPUT_PULLUP);

  // Setup encoders to be read properly
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoderD, CHANGE);

  // Sets default PID thetaDes2 to PI if something goes wrong with user input
  thetaDes2B[0] = PI;

  md2.init();
  md2.calibrateCurrentOffset();

  // Keep motor driver disabled during startup
  md2.setSpeed(0);
  md2.Sleep();
  // Now enable the driver
  waitForUserStart();
  md2.Wake();


  lastControlMicros = micros();


  Serial.println("Homing motor B...");

  // Homing loop
  do
  {
    md2.setSpeed(-50);
  } while (digitalRead(limitSwitchPin2B) == HIGH);

  // Initializes values after homing
  md2.setSpeed(0);
  encoderCountB = 0;
  theta_measB = 0.0;
  e_int2 = 0.0;
  e_prev2 = 0.0;
  calibrated = true;
  Serial.println("Homing complete.");

  // 2. Put the driver chip into low-power sleep mode
  md2.Sleep();
  waitForUserStart();
  md2.Wake();
  md2.setSpeed(0);

  // Resets interrupt triggers 0 and 1 (they're triggered during homing but not deactivated)
  EIFR = bit(INTF0) | bit(INTF1);

  // Attach interrupt for limit switch safety feature
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
    md2.Sleep();
    Serial.println("Limit switch hit! Halting.");
    while (true)
    {
    }
  }

  // Checks for user input
  keyPress();

  unsigned long now = micros();
  if ((unsigned long)(now - lastControlMicros) >= controlPeriodMicros)
  {
    float dt = (now - lastControlMicros) * 1e-6;
    lastControlMicros = now;

    readEncoderState(dt, encoderCountB, prevCountB, theta_measB, omega_measB);
    // stopIfFault();

    // If close enough to current state, move onto new state
    // Threshold is currently arbitrary. Need to check and fine tune
    if (fabs(thetaDes2B[completed] - theta_measB) <= 0.05)
    {
      int wait = 300;
      // Increment PID and tells user
      if (completed < num_of_pos - 1)
      {
        Serial.print("State ");
        Serial.print(completed);
        Serial.println(" completed.");

        // stops motor and resets integral error for next position
        e_int2 = 0.0;
        md2.setSpeed(0);
        completed++;

        // If delay is too short, prevents current spike from braking by forcing error of next iteration to be 0;
        e_prev2 = thetaDes2B[completed] - theta_measB;

        // Tells user about delay and delay progress
        for (int i = 0; i < wait; i++)
        {
          Serial.print("Now delaying ");
          Serial.print(i);
          Serial.print("/");
          Serial.println(wait);
        }
      }
    }

    // Calculate PID and control motor
    float u_satB = calculatePID(thetaDes2B[completed], theta_measB, maxTheta2, dt, e_int2, e_prev2, Kp2, Ki2, Kd2);
    setMotorCommandB(u_satB);

    // Print at lower rate to avoid slowing control loop
    printCounter++;
    if (printCounter >= printEvery)
    {
      Serial.println("MotorB:");
      Serial.print("Desired position: ");
      Serial.print(thetaDes2B[completed], 4);
      Serial.print(", ");
      Serial.print("Current Position: ");
      Serial.print(theta_measB, 4);
      Serial.print(", ");
      Serial.print("Angular Velocity: ");
      Serial.print(omega_measB, 4);
      Serial.print(", ");
      Serial.print("u_sat: ");
      Serial.println(u_satB, 2);
      Serial.print("Encoder Count: ");
      Serial.println(encoderCountB);
      Serial.print("State: ");
      Serial.println(completed);
      Serial.print("\n");

      printCounter = 0;
    }
  }
}
