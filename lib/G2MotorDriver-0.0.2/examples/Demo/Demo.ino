// Include the G2MotorDriver library
#include "G2MotorDriver.h"

// Uncomment the Initialize library version corresponding with the version of your breakout board
// Initialize library
G2MotorDriver18v17 md;
//G2MotorDriver18v25 md;
//G2MotorDriver24v13 md;
//G2MotorDriver24v21 md;

void stopIfFault()
{
    if (md.getFault())
    {
        md.Sleep(); // put the driver to sleep on fault
        delay(1);
        Serial.println("Motor fault");
        while(1);
    }
}

void setup()
{
    // start serial port
    Serial.begin(115200);
    Serial.println("G2 Motor Driver");

    // Start up the library
    md.init();
    md.Wake(); // Wake the driver for current readings
    md.calibrateCurrentOffset();
    delay(10);
    md.Sleep(); // Put the Motor driver into sleep mode until you need to use it.
    delay(10);
}

void loop()
{
    md.Wake(); // Wake up the driver, it's time to drive motors.

    for (int i = 0; i <= 400; i++)
    {
        md.setSpeed(i);
        stopIfFault();
        if (i%200 == 100)
        {
            Serial.print("Motor current: ");
            Serial.println(md.getCurrentMilliamps());
        }
        delay(2);
    }

    for (int i = 400; i >= -400; i--)
    {
        md.setSpeed(i);
        stopIfFault();
        if (i%200 == 100)
        {
            Serial.print("Motor current: ");
            Serial.println(md.getCurrentMilliamps());
        }
        delay(2);
    }

    for (int i = -400; i <= 0; i++)
    {
        md.setSpeed(i);
        stopIfFault();
        if (i%200 == 100)
        {
            Serial.print("Motor current: ");
            Serial.println(md.getCurrentMilliamps());
        }
        delay(2);
    }

    md.Sleep(); // Put the Motor driver into sleep mode until they are needed again.
    delay(500);
}
