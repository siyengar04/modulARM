#include "G2MotorDriver.h"
#include "PID_v1.h"

#define encoderPinA 2
#define encoderPinB 3

G2MotorDriver24v13 md;

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
double Kp = 2.0, Ki = 5.0, Kd = 0.5; 
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
bool positionControlMode = true; 
float targetPosition = 0.0;      
float targetSpeed = 0.0;       
unsigned long lastPIDTime = 0;
unsigned long pidInterval = 10;
float currentSpeed = 0.0;
long lastSpeedTime = 0;
int lastEncoderPosition = 0;

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

  pinMode(encoderPinA, input);
  pinMode(encoderPinB, input);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-400, 400);
  myPID.SetSampleTime(pidInterval);

  lastPIDTime = millis();
  lastSpeedTime = millis();
  lastEncoderPosition = encoderPosition;

  Serial.println("setup complete!");
}
void calcSpeed()
{
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastSpeedTime;

  if (dt >= 50)
    long deltaPulses = encoderPosition - lastEncoderPosition;
    float deltaAngle = (deltaPulses * 360.0) / 17280.0; 
    currentSpeed = (deltaAngle * 1000.0) / dt; 
    lastEncoderPosition = encoderPosition;
    lastSpeedTime = currentTime;
  }
}

void loop()
{
  md.setSpeed(50);
  // Serial.println(startTime);

  if (millis() - startTime > 1000)
  {
    encoderAngle = (((encoderPosition) * 360) / 17280);
    Serial.println("Theta = ");
    Serial.print(encoderAngle);
    Serial.println(" deg");
    encoderSpeed = (encoderAngle - previousAngle);
    Serial.println("Theta_Dot = ");
    Serial.print(encoderSpeed);
    // currentTime = millis();
    // timeWindow = (currentTime - startTime);
    // Serial.println("");
    // Serial.println(timeWindow);
    previousAngle = encoderAngle;
    startTime = millis();
  }

  // while (millis() > 20) {
  // }
  // There is 4 pulses per step
  // Encoder has 64 steps per revolution
  // Gearbox ratio is 1:270
}
