#include "G2MotorDriver.h"

// Driver config
//driver 2 TODO
const uint8_t MD_DIR2 = 5;
const uint8_t MD_PWM2 = 12;
const uint8_t MD_SLP2 = 4;
const uint8_t MD_FLT2 = 30;
const uint8_t MD_CS2 = A1;

// Limit switch pins 
const uint8_t limitSwitchPinB2 = 20;
const uint8_t limitSwitchPinB1 = 8;

const float maxTheta2 = 3;
const float home = 0.0;
const float maxCurrent = 0.6;

volatile bool calibrated = false;
volatile bool systemLock = false;

G2MotorDriver24v13 md2(MD_DIR2, MD_PWM2, MD_SLP2, MD_FLT2, MD_CS2);

const uint8_t encoderBPinA = 18;
const uint8_t encoderBPinB = 19;

volatile long encoderCountB = 0;

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
//motor B
float KpB = 500.0;
float KiB = 300.0;
float KdB = 2.0;

// ===================== SETPOINT =====================
//float theta_desB = PI;

const int MAX_STATES = 10;     // max allowed number of states
float theta_desB[MAX_STATES];
int num_of_pos = 1;

int completed = 0;

// ===================== PID STATES =====================
float e_intB = 0.0;
float e_prevB = 0.0;

// Integral anti-windup limit
const float eIntMax = 50.0;

// ===================== MEASURED STATES =====================
float theta_measB = 0.0;
float omega_measB = 0.0;

long prevCountB = 0;

// ===================== SERIAL PRINTING =====================
int printCounter = 0;
const int printEvery = 30; // print every 30 control loops = 500 Hz

// ==========================  Prototypes  ================================
void readEncoderState(float dt, volatile long encoderCount, long &prevCount, float &theta_meas, float &omega_meas);

// ===================== ENCODER ISR =====================
void doEncoderC()
{
  bool A = digitalRead(encoderBPinA);
  bool B = digitalRead(encoderBPinB);

  if (A == B)
    encoderCountB++;
  else
    encoderCountB--;
}

void doEncoderD()
{
  bool A = digitalRead(encoderBPinA);
  bool B = digitalRead(encoderBPinB);

  if (A == B)
    encoderCountB--;
  else
    encoderCountB++;
}

void doLimit2()
{
  md2.setSpeed(0);
  systemLock = true;
}

// ===================== USER INPUT FUNCTIONS =====================  //ONLY HANDLES ONE MOTOR

void set_theta_des(float &theta_des, float maxTheta, int pos)  {
  //Prompt User
  Serial.println("==============================================");
  Serial.print("Enter desired position in radians (0.00 to ");
  Serial.print(maxTheta, 2);
  Serial.print(") ");
  Serial.print("for position ");
  Serial.print(pos);
  Serial.println(":");
  Serial.println("==============================================");

  Serial.setTimeout(10000);

  float new_theta = Serial.parseFloat();

  theta_des = constrain(new_theta, home, maxTheta);

  // Print confirmation
  Serial.print("Target position for state ");
  Serial.print(pos);
  Serial.print(" successfully set to: ");
  Serial.print("Target position successfully set to: "); //comment out when switching to array
  Serial.println(theta_des, 4);

  while (Serial.available() && Serial.peek() != '\n')
    Serial.read(); // Clear buffer
  return;
  
}

void waitForUserStart() {
  Serial.println("\n==============================================");
  Serial.println("Do you want it to start? (Y/N)");
  Serial.println("==============================================");

  while (true) {
    if (Serial.available() > 0) {
      char response = Serial.read();

      // Check for yes (both uppercase and lowercase)
      if (response == 'Y' || response == 'y') {
        Serial.println("Input Y detected.");
        if (calibrated) 
        {
          
          // Prompt user for how many positions and to set positions
          Serial.println("\n==============================================");
          Serial.print("How many positions do you want? (Type an integer between 1 and ");
          Serial.print(MAX_STATES);
          Serial.println(")");
          Serial.println("==============================================");

          Serial.setTimeout(10000);
          num_of_pos = Serial.parseInt();
          num_of_pos = constrain(num_of_pos, 1, MAX_STATES);

          for (int i = 0; i < num_of_pos; i++) {
          set_theta_des(theta_desB[i], maxTheta2, i);
          }

          completed = 0;

          Serial.println("All states are the following: ");
          Serial.print("[");
          for (int i = 0; i < num_of_pos - 1; i++) {
            Serial.print(theta_desB[i]);
            Serial.print(", ");
          }
          Serial.print(theta_desB[num_of_pos-1]);
          Serial.println("]");

          Serial.println("----------------------------------------------");
          Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
          Serial.println("----------------------------------------------");

          delay(10);
          while (Serial.available() > 0) { Serial.read(); }

          while (true) 
          {
            if (Serial.available() > 0) 
            {
              char c = Serial.read();
              if (c == '\n' || c == '\r') {
                break; // Exit and return control to setup/loop
              }
            }
          }

          delay(10);
          while (Serial.available() > 0) { Serial.read(); }
          
          Serial.println("Motor engaged! Starting control loop...");
        }
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

void keyPress() 
{
  if (Serial.available() > 0) 
  {
    md2.setSpeed(0);
    md2.Sleep();

    // Prompt user for how many positions and to set positions
    Serial.println("\n==============================================");
    Serial.print("How many positions do you want? (Type an integer between 1 and ");
    Serial.print(MAX_STATES);
    Serial.println(")");
    Serial.println("==============================================");

    Serial.setTimeout(10000);

    num_of_pos = Serial.parseInt();
    num_of_pos = constrain(num_of_pos, 1, MAX_STATES);
    Serial.print("Number of positions set to: ");
    Serial.println(num_of_pos);

    for (int i = 0; i < num_of_pos; i++){
      set_theta_des(theta_desB[i], maxTheta2, i);
    }

    completed = 0;

    Serial.println("----------------------------------------------");
    Serial.println(">>> Press ENTER again to engage the PID motor loop <<<");
    Serial.println("----------------------------------------------");

    delay(10);
    while (Serial.available() > 0) { Serial.read(); }
    while (true) 
    {
      if (Serial.available() > 0) 
      {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          break; // Exit and return control to setup/loop
        }
      }
    }

    delay(10);
    while (Serial.available() > 0) { Serial.read(); }
    
    Serial.println("Motor engaged! Starting control loop...");

    e_intB = 0.0;
    
    unsigned long now_pre = micros();
    float dt_pre = (now_pre - lastControlMicros) * 1e-6;
    if (dt_pre <= 0) dt_pre = 0.002; 
    readEncoderState(dt_pre, encoderCountB, prevCountB, theta_measB, omega_measB);
    
    e_prevB = theta_desB[0] - theta_measB;

    md2.Wake();
    lastControlMicros = micros();
  }
}

// ===================== FAULT CHECK =====================
void stopIfFault()
{
  if (md2.getFault())
  {

    md2.setSpeed(0);
    md2.Sleep();

    Serial.println("Motor driver fault detected. Driver disabled.");

    while (1)
    {
      delay(100);
    }
  }
}

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
float calculatePID(float theta_des, float theta_meas, float maxTheta, float dt, float &e_int, float &e_prev, float Kp, float Ki, float Kd)
{
  // Position error
  float e = theta_des - theta_meas;
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

  //Current Sense stuff
  // float currentB = md2.getCurrentMilliamps();
  
  // if (((currentB / 2800) >= maxCurrent) && (omega_measB <= 3))
  // {
  //   return 0;
  // }
  if (digitalRead(limitSwitchPinB1) == LOW) 
  {
    systemLock = true;
    return 0;
  }
  // Boundary check
  if (((theta_meas >= maxTheta) && u_sat > 0) || (theta_meas <= 0.14 && u_sat < 0))  
  {
    return 0;
  }

  return u_sat;
}

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  //to prevent motor kick on startup write PWM to low before starting the driver
  digitalWrite(MD_PWM2, LOW);
  pinMode(encoderBPinA, INPUT_PULLUP);
  pinMode(encoderBPinB, INPUT_PULLUP);
  pinMode(limitSwitchPinB2, INPUT_PULLUP);
  pinMode(limitSwitchPinB1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderBPinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPinB), doEncoderD, CHANGE);

  theta_desB[0] = PI;

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
  do {
    md2.setSpeed(-200);
    delay(100);
  } while (digitalRead(limitSwitchPinB2) == HIGH);
  md2.setSpeed(0);
  encoderCountB = 0;
  theta_measB = 0.0;
  e_intB = 0.0;
  e_prevB = 0.0;
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

  attachInterrupt(digitalPinToInterrupt(limitSwitchPinB2), doLimit2, FALLING);

  lastControlMicros = micros();
}


// ===================== LOOP =====================
void loop()
{
  // After Calibration, if button hits the limit switch the program stops motors and requires reset
  if (systemLock) {
    //stop motor B
    md2.setSpeed(0);
    md2.setBrake(400);
    md2.Sleep();
    Serial.println("Limit switch hit! Halting.");
    while (true) { }
  }

  keyPress();

  unsigned long now = micros();
  if ((unsigned long)(now - lastControlMicros) >= controlPeriodMicros)
  {
    float dt = (now - lastControlMicros) * 1e-6;
    lastControlMicros = now;

    readEncoderState(dt, encoderCountB, prevCountB, theta_measB, omega_measB);
    stopIfFault();

    if(fabs(theta_desB[completed] - theta_measB) <= 0.05) 
    {
        if (completed < num_of_pos - 1){
          completed++;
        }
    }  

    float u_satB = calculatePID(theta_desB[completed], theta_measB, maxTheta2, dt, e_intB, e_prevB, KpB, KiB, KdB);
    setMotorCommandB(u_satB);

    
    // Print at lower rate to avoid slowing control loop
    printCounter++;
    if (printCounter >= printEvery)
    {
      Serial.println("MotorB:");
      Serial.print(theta_desB[completed], 4);
      Serial.print(",");
      Serial.print(theta_measB, 4);
      Serial.print(",");
      Serial.print(omega_measB, 4);
      Serial.print(",");
      Serial.println(u_satB, 2);
      Serial.print("\n");
      Serial.print("State: ");
      Serial.println(completed);

      printCounter = 0;
    }

  }
}
