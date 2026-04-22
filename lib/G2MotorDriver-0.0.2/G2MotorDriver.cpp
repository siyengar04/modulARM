/**
 * G2MotorDriver.cpp - Library for controlling a G2 MotorDriver.
 * Created by Walt Sorensen, March 23, 2019.
**/

#include "Arduino.h"
#include "G2MotorDriver.h"

boolean G2MotorDriver::_flip = false;

// Constructors
G2MotorDriver::G2MotorDriver()
{
    // Pin Map
    _DIRPin = 2;
    _PWMPin = 9; // default PWM on Timer 1 for Uno all other boards or pins need to be mapped
    _SLPPin = 4;
    _FLTPin = 6;
    _CSPin  = A0;
}

G2MotorDriver::G2MotorDriver(
            unsigned char DIRPin, //INA1,
            unsigned char PWMPin,
            unsigned char SLPPin,
            unsigned char FLTPin, //EN1DIAG1,
            unsigned char CSPin)
{
    // Pin Map
    _DIRPin = DIRPin;
    _PWMPin = PWMPin;
    _SLPPin = SLPPin;
    _FLTPin = FLTPin;
    _CSPin  = CSPin;
}

// Public Methods
void G2MotorDriver::init()
{
    // Define pinMode for the pins
    pinMode(_DIRPin, OUTPUT);
    pinMode(_PWMPin, OUTPUT);
    pinMode(_SLPPin, OUTPUT);
    pinMode(_FLTPin, INPUT_PULLUP); // Output is driven low when a fault has occurred INPUT_PULLUP where HIGH means the sensor is off, and LOW means the sensor is on
    pinMode(_CSPin, INPUT);

    // Set the frequency for timer1.
    #ifdef G2MOTORDRIVER_TIMER1_AVAILABLE
    if (_PWMPin == _PWM_TIMER1_PIN_A || _PWMPin == _PWM_TIMER1_PIN_B)
    {
        /**
         * Timer 1 Phase-Correct PWM configuration
         * prescaler: clockI/O / 1
         * outputs enabled
         * phase-correct PWM
         * top of 400
         * 
         * PWM frequency calculation
         * 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = ~20kHz
         *
         * The Timer/Counter Control Registers TCCRnA and TCCRnB hold the main control bits for the timer.
        **/
        TCCR1A = 0b10100000;
        TCCR1B = 0b00010001;
        ICR1 = 400;
    }
    #endif
}

// Set speed for motor 1, speed is a number betwenn -400 and 400
void G2MotorDriver::setSpeed(int speed)
{
    unsigned char reverse = 0;

    if (speed < 0)
    {
        speed = -speed;  // Make speed a positive quantity
        reverse = 1;  // Preserve the direction
    }

    if (speed > 400)  // Max PWM dutycycle
        speed = 400;

    #ifdef G2MOTORDRIVER_TIMER1_AVAILABLE
    if (_PWMPin == _PWM_TIMER1_PIN_A)
    {
        // PWM timer counts from 0 to OCRnA the timer top limit (i.e. value of output compare register A) representing a duty cycle
        OCR1A = speed;
    }
    else if (_PWMPin == _PWM_TIMER1_PIN_B)
    {
        // PWM timer counts from 0 to OCRnB the timer top limit (i.e. value of output compare register B) representing a duty cycle
        OCR1B = speed;
    }
    else
    {
        analogWrite(_PWMPin, speed * 51 / 80); // map 400 to 255
    }
    #else
        analogWrite(_PWMPin, speed * 51 / 80); // map 400 to 255
    #endif

    if (reverse ^ _flip) // flip if speed was negative or _flip setting is active, but not both
    {
        digitalWrite(_DIRPin, HIGH); // DIRPin is high, current will flow from OUTA to OUTB
    }
    else
    {
        digitalWrite(_DIRPin, LOW); // DIRPin is low, current will flow from OUTB to OUTA.
    }
}

void G2MotorDriver::flip(boolean flip)
{
    G2MotorDriver::_flip = flip;
}

// Brake motor, brake is a number between 0 and 400
void G2MotorDriver::setBrake(int brake)
{
    // normalize brake
    if (brake < 0)
    {
        brake = -brake;
    }

    if (brake > 400)  // Max brake
        brake = 400;
        digitalWrite(_DIRPin, LOW);

    #ifdef G2MOTORDRIVER_TIMER1_AVAILABLE
    if (_PWMPin == _PWM_TIMER1_PIN_A)
    {
        OCR1A = brake;
    }
    else if (_PWMPin == _PWM_TIMER1_PIN_B)
    {
        OCR1B = brake;
    }
    else
    {
        analogWrite(_PWMPin, brake * 51 / 80); // map 400 to 255
    }
    #else
        analogWrite(_PWMPin, brake * 51 / 80); // map 400 to 255
    #endif
}

// Set voltage offset of Motor current reading at 0 speed.
void G2MotorDriver::calibrateCurrentOffset()
{
    setSpeed(0);
    Wake();
    delay(1);
    G2MotorDriver::_currentOffset = getCurrentReading();
}

unsigned int G2MotorDriver::getCurrentReading()
{
    return analogRead(_CSPin);
}

// Return motor current value in milliamps.
unsigned int G2MotorDriver::getCurrentMilliamps(int gain)
{
    /**
      * 5V / 1024 ADC counts / gain mV per A
      * The 24v13 results in 122 mA per count.
      * The 18v17, and 24v21 results in 244 mA per count.
      * The 18v25 results in 488 mA per count.
    **/
    unsigned int mAPerCount = 5000000/1024/gain;
    int reading = (getCurrentReading() - G2MotorDriver::_currentOffset);

    if (reading > 0)
    {
        return reading * mAPerCount;
    }

    return 0;
}

// Return error status for motor
unsigned char G2MotorDriver::getFault()
{
    return !digitalRead(_FLTPin);
}

// Put the motor driver to sleep
void G2MotorDriver::Sleep()
{
    digitalWrite(_SLPPin, LOW); // SLPPin must be driven logic low to disable the driver putting it to sleep.
}

// Wake up the motor driver
void G2MotorDriver::Wake()
{
    digitalWrite(_SLPPin, HIGH); // SLPPin must be driven logic High to enable the driver and waking it up.
    delay(1);  // The driver require a maximum of 1ms to elapse time when brought out of sleep mode.
}

// Return current value in milliamps for 18v17 version.
unsigned int G2MotorDriver18v17::getCurrentMilliamps()
{
    int gainValue = 20;
    return G2MotorDriver::getCurrentMilliamps(gainValue);
}

// Return current value in milliamps for 18v25 version.
unsigned int G2MotorDriver18v25::getCurrentMilliamps()
{
    int gainValue = 10;
    return G2MotorDriver::getCurrentMilliamps(gainValue);
}

// Return current value in milliamps for 24v13 version.
unsigned int G2MotorDriver24v13::getCurrentMilliamps()
{
    int gainValue = 40;
    return G2MotorDriver::getCurrentMilliamps(gainValue);
}

// Return current value in milliamps for 24v21 version.
unsigned int G2MotorDriver24v21::getCurrentMilliamps()
{
    int gainValue = 20;
    return G2MotorDriver::getCurrentMilliamps(gainValue);
}
