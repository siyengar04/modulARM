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
double Setpoint, Input, Output;

long startTime = 0;
long currentTime = 0;
long timeWindow = 0;
int totalPulses = 0;

int angle = 0;

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
  md.Sleep();

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);
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
