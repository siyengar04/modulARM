/** 
 * Controlling 2 wire brushed DC motors via PWM
 * Hardware: 1 * Arduino or compatible board
 *         : 1 * G2 Motor Driver
 *         : 1 * Brushed DC Motor
 *         : 1 * 6.5 V to 30 V DC up to 40 Amp. external powersource for the brushed DC motor.
**/

#pragma once

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__) || \
    defined(__AVR_ATmega16U4__)
// Timers generally available for all boards.
    #define G2MOTORDRIVER_TIMER1_AVAILABLE
    #define G2MOTORDRIVER_TIMER2_AVAILABLE
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// Additional timmers for an Arduino Mega.
    #define G2MOTORDRIVER_TIMER3_AVAILABLE
    #define G2MOTORDRIVER_TIMER4_AVAILABLE
    #define G2MOTORDRIVER_TIMER5_AVAILABLE
#endif

#include <Arduino.h>

/**
 * This is the base class used to represent a connection to a G2 motor driver.  This class
 * provides high-level functions for sending commands to the G2 motor driver and reading
 * data from it.
 **/
class G2MotorDriver
{
    public:
        // CONSTRUCTORS
        // Default pin selection.
        G2MotorDriver();

        // User-defined pin selection.
        G2MotorDriver(
            unsigned char DIRPin,
            unsigned char PWMPin,
            unsigned char SLPPin, // Inverted sleep input: SLPPin must be driven logic high to enable the driver
            unsigned char FLTPin,
            unsigned char CSPin);

        // PUBLIC METHODS
        void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
        void setSpeed(int speed); // Set speed for Motor.
        void setBrake(int brake); // Brake for Motor.
        unsigned int getCurrentMilliamps(int gain); // Get current reading for Motor.
        unsigned int getCurrentReading();
        void calibrateCurrentOffset();
        unsigned char getFault(); // Get fault reading from Motor.
        void flip(boolean flip); // Flip the direction of the speed for M1.
        void Sleep(); // Put the motor driver to sleep
        void Wake(); // Wake up the motor driver

    protected:
        unsigned int _currentOffset;
    
    private:
        unsigned char _DIRPin;
        unsigned char _PWMPin;
        #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            // Code in here will only be compiled if an Arduino Uno (or older) is used.
            static const unsigned char _PWM_TIMER1_PIN_A = 9;
            static const unsigned char _PWM_TIMER1_PIN_B = 10;
            static const unsigned char _PWM_TIMER2_PIN_A = 11;
            static const unsigned char _PWM_TIMER2_PIN_B = 3;
        #endif
        #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
            // Code in here will only be compiled if an Arduino Leonardo is used.
            static const unsigned char _PWM_TIMER1_PIN_A = 9;
            static const unsigned char _PWM_TIMER1_PIN_B = 10;
            static const unsigned char _PWM_TIMER3_PIN_A = 5;
            static const unsigned char _PWM_TIMER4_PIN_A = 13;
            static const unsigned char _PWM_TIMER4_PIN_B = 6;
        #endif
        #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        // Code in here will only be compiled if an Arduino Mega is used.
            static const unsigned char _PWM_TIMER1_PIN_A = 11;
            static const unsigned char _PWM_TIMER1_PIN_B = 12;
            static const unsigned char _PWM_TIMER2_PIN_A = 10;
            static const unsigned char _PWM_TIMER2_PIN_B = 9;
            static const unsigned char _PWM_TIMER3_PIN_A = 5;
            static const unsigned char _PWM_TIMER3_PIN_B = 2;
            static const unsigned char _PWM_TIMER3_PIN_C = 3;
            static const unsigned char _PWM_TIMER4_PIN_A = 6;
            static const unsigned char _PWM_TIMER4_PIN_B = 7;
            static const unsigned char _PWM_TIMER4_PIN_C = 8;
            static const unsigned char _PWM_TIMER5_PIN_A = 46;
            static const unsigned char _PWM_TIMER5_PIN_B = 45;
            static const unsigned char _PWM_TIMER5_PIN_C = 44;
        #endif
        unsigned char _SLPPin;
        unsigned char _FLTPin;
        unsigned char _CSPin;
        static boolean _flip;
};


// Additional prototypes for 18v17 version.
class G2MotorDriver18v17 : public G2MotorDriver
{
    public:
        using G2MotorDriver::G2MotorDriver;
        unsigned int getCurrentMilliamps(); // Get current reading for Motor with preset gain
};

// Additional prototypes for 18v25 version.
class G2MotorDriver18v25 : public G2MotorDriver
{
    public:
        using G2MotorDriver::G2MotorDriver;
        unsigned int getCurrentMilliamps(); // Get current reading for Motor with preset gain
};

// Additional prototypes for 24v13 version.
class G2MotorDriver24v13 : public G2MotorDriver
{
    public:
        using G2MotorDriver::G2MotorDriver;
        unsigned int getCurrentMilliamps(); // Get current reading for Motor with preset gain
};

// Additional prototypes for 24v21 version.
class G2MotorDriver24v21 : public G2MotorDriver
{
    public:
        using G2MotorDriver::G2MotorDriver;
        unsigned int getCurrentMilliamps(); // Get current reading for Motor with preset gain
};
