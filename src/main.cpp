#include "G2MotorDriver.h"
#include "PID_v1.h"

#define encoderPinA 2
#define encoderPinB 3

G2MotorDriver24v13 md(7, 9, 4, 6, 0); // DIRPin, PWMPin, SLPPin, FLTPin, CSPin

volatile int encoderPosition = 0;
float previousAngle = 0.0;
float encoderAngle = 0.0;
float encoderSpeed = 0.0;
float rpm = 0.0;
float theta = 0.0; 
float theta_dot = 0.0;
int angle = 0;

long startTime = 0;
long currentTime = 0;
long timeWindow = 0;
int totalPulses = 0;

double setpoint, input, output;
// double Kp = 0.3, Ki = 0.75, Kd = 0.3;
double Kp = 0.1, Ki = 0.3, Kd = 0.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
bool positionControlMode = true; 
float targetPosition = 0.0;      
float targetSpeed = 0.0;       
unsigned long lastPIDTime = 0;
unsigned long pidInterval = 5;
float currentSpeed = 0.0;
long lastSpeedTime = 0;
int lastEncoderPosition = 0;

// Hard angle limits
float minAngle = -90.0;  // Minimum angle in degrees
float maxAngle = 90.0;   // Maximum angle in degrees
bool angleLimitsEnabled = false;

void doEncoderA()
{
  if (digitalRead(encoderPinA) == HIGH)
  {
    if (digitalRead(encoderPinB) == LOW)
      encoderPosition++;  // CW
    else
      encoderPosition--;  // CCW
  }
  else
  {
    if (digitalRead(encoderPinB) == HIGH)
      encoderPosition++;  // CW
    else
      encoderPosition--;  // CCW
  }
}
void doEncoderB()
{
  if (digitalRead(encoderPinB) == HIGH)
  {
    if (digitalRead(encoderPinA) == HIGH)
      encoderPosition++;  // CW
    else
      encoderPosition--;  // CCW
  }
  else
  {
    if (digitalRead(encoderPinA) == LOW)
      encoderPosition++;  // CW
    else
      encoderPosition--;  // CCW
  }
}

void setup()
{
  Serial.begin(115200);

  // Motor driver
  md.init();
  md.Wake();
  md.calibrateCurrentOffset();
  delay(10);
  // md.Sleep();

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-300, 300);
  myPID.SetSampleTime(pidInterval);

  // Initialize setpoint to current position
  setpoint = 0.0;

  lastPIDTime = millis();
  lastSpeedTime = millis();
  lastEncoderPosition = encoderPosition;

  Serial.println("setup complete!");
}
void calcSpeed()
{
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastSpeedTime;

  if (dt >= 10)  // Calculate speed every 10ms for PID input freshness
  {
    long deltaPulses = encoderPosition - lastEncoderPosition;
    float deltaAngle = (deltaPulses * 360.0) / 17280.0; 
    currentSpeed = (deltaAngle * 1000.0) / dt; 
    lastEncoderPosition = encoderPosition;
    lastSpeedTime = currentTime;
  }
}
void updatePID()
{
  encoderAngle = fmod((encoderPosition * 360.0) / 17280.0 + 360.0, 360.0);
  calcSpeed();

  if (positionControlMode)
  {
    input = encoderAngle;
    
    // Calculate signed error with wrap-around
    float error = setpoint - input;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // Check if close enough to target (within 2 degrees)
    if (abs(error) < 5.0)
    {
      output = 0;  // Stop output when settled
      // md.setSpeed(0);
    }
    else
    {
      myPID.Compute();
    }
  }
  else
  {
    input = currentSpeed;
    myPID.Compute();
  }

  // Apply hard angle limits using direction pin logic
  int finalSpeed = constrain(output, -400, 400);
  
  if (angleLimitsEnabled)
  {
    // Enforce hard angle limits
    if (encoderAngle <= minAngle && finalSpeed < 0)
    {
      // At minimum angle, prevent counterclockwise motion
      finalSpeed = 0;
    }
    else if (encoderAngle >= maxAngle && finalSpeed > 0)
    {
      // At maximum angle, prevent clockwise motion
      finalSpeed = 0;
    }
  }

  md.setSpeed(finalSpeed);
}
void processSerialCommands()
{
  if (Serial.available())
  {
    char cmd = Serial.read();

    if (cmd == 'p')
    { 
      float pos = Serial.parseFloat();
      targetPosition = pos;
      positionControlMode = true;
      setpoint = targetPosition;
      Serial.print("Position mode - Target: ");
      Serial.print(targetPosition);
      Serial.println(" degrees");
      while (Serial.available() && Serial.peek() != '\n') Serial.read();  // Clear buffer
    }
    else if (cmd == 's')
    {
      float speed = Serial.parseFloat();
      targetSpeed = speed * (244.44/400.0); // deg / sec
      positionControlMode = false;
      setpoint = targetSpeed * (400.0/244.44);
      Serial.print("Speed mode - Target: ");
      Serial.print(targetSpeed);
      Serial.println(" deg/sec");
      while (Serial.available() && Serial.peek() != '\n') Serial.read();  // Clear buffer
    }
    else if (cmd == 'k')
    {
      char param = Serial.read();
      float value = Serial.parseFloat();

      if (param == 'p')
      {
        Kp = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Kp set to: ");
        Serial.println(Kp);
      }
      else if (param == 'i')
      {
        Ki = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Ki set to: ");
        Serial.println(Ki);
      }
      else if (param == 'd')
      {
        Kd = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Kd set to: ");
        Serial.println(Kd);
      }
      while (Serial.available() && Serial.peek() != '\n') Serial.read();  // Clear buffer
    }
    else if (cmd == 'l')
    {
      char param = Serial.read();
      float value = Serial.parseFloat();

      if (param == 'n')
      {
        minAngle = value;
        Serial.print("Min angle limit set to: ");
        Serial.println(minAngle);
      }
      else if (param == 'x')
      {
        maxAngle = value;
        Serial.print("Max angle limit set to: ");
        Serial.println(maxAngle);
      }
      else if (param == 'e')
      {
        angleLimitsEnabled = (value != 0);
        Serial.print("Angle limits ");
        Serial.println(angleLimitsEnabled ? "ENABLED" : "DISABLED");
      }
      while (Serial.available() && Serial.peek() != '\n') Serial.read();  // Clear buffer
    }
  }
}

void loop()
{
  processSerialCommands();  // Process incoming serial commands
  
  unsigned long currentTime = millis();

  // Serial.println(startTime);
  if (currentTime - lastPIDTime >= pidInterval)
  {
    updatePID();
    lastPIDTime = currentTime;

    // Debug output every 100ms
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 100)
    {
      Serial.print("mode: ");
      Serial.print(positionControlMode ? "POS" : "SPD");
      Serial.print(" | setpoint: ");
      Serial.print(setpoint);
      Serial.print(" | input: ");
      Serial.print(input);
      Serial.print(" | output: ");
      Serial.print(output);
      Serial.print(" | position: ");
      Serial.print(encoderAngle);
      Serial.print("° | speed: ");
      Serial.print(currentSpeed);
      Serial.print("°/s | limits: [");
      Serial.print(minAngle);
      Serial.print(", ");
      Serial.print(maxAngle);
      Serial.print("] ");
      Serial.println(angleLimitsEnabled ? "(ON)" : "(OFF)");
      lastDebugTime = currentTime;
    }
  }
}