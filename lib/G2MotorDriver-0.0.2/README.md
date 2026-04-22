# Arduino library for the Pololu G2 High-Power Motor Driver breakout board

Version: 0.0.2<br>
Release Date: 2019-04-06<br>
[![Build Status](https://travis-ci.org/photodude/G2MotorDriver.svg?branch=master)](https://travis-ci.org/photodude/G2MotorDriver)<br>
Pololu G2 High-Power Motor Driver breakout board https://www.pololu.com/product/2991<br>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that interfaces with the [Pololu G2 High-Power Motor Driver breakout board](https://www.pololu.com/product/2991).
It makes it simple to drive one brushed DC motor.

The library is highly influenced by the [Pololu Dual VNH5019 Motor Driver Shield](https://github.com/pololu/dual-vnh5019-motor-shield) with much of the code base and readme originating from that library. Additional functions are based on the [dual-g2-high-power-motor-shield](https://github.com/pololu/dual-g2-high-power-motor-shield). With that said extensive modifications have been made and many variable renamed to reflect use with the G2 High-Power Motor Driver breakout board

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.6.x or later; we have not tested it with earlier versions.  This library should support any Arduino-compatible board, including the [Pololu A-Star 32U4 controllers](https://www.pololu.com/category/149/a-star-programmable-controllers).

## Getting started

### Hardware

The [Pololu G2 High-Power Motor Driver breakout board](https://www.pololu.com/product/2991) can be purchased from Pololu's website.  Before continuing, careful reading of the product page is recommended.

### Software

If you are using version 1.6.2 or later of the [Arduino software (IDE)](https://www.arduino.cc/en/Main/Software), you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then "Manage Libraries...".
2. Search for "G2MotorDriver".
3. Click the G2MotorDriver entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub](https://github.com/photodude/G2MotorDriver/releases) and decompress it.
2. Rename the folder "G2MotorDriver-xxxx" to "G2MotorDriver".
3. Drag the "G2MotorDriver" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Example

An example sketch is available that shows how to use the library.  You can access it from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "G2MotorDriver".  If
you cannot find these examples, the library was probably installed incorrectly and you should retry the installation instructions above.

### Demo

The demo sketch example ramps the motor from stopped to full speed forward, ramps down
to full speed reverse, and back to stopped. Current readings for the motor are sent over serial and can be seen with the serial monitor.  If a fault is detected, a message is sent over serial.

## Documentation

- `G2MotorDriver()`<br> Default constructor, selects the default pins for the motor driver.
- `G2MotorDriver(
    unsigned char DIR,
    unsigned char PWM,
    unsigned char SLP,
    unsigned char FLT,
    unsigned char CS,
    )` <br>
Alternate constructor for G2 motor driver connections remapped by user. If PWM are remapped, it will try to use analogWrite instead of timer1.
- `void init()` <br> Initialize pinModes and timer1.
- `void setSpeed(int speed)` <br> Set speed and direction for motor.
  Speed should be between -400 and 400.  400 corresponds to motor current flowing from M1A to M1B.  -400 corresponds to motor current flowing from OUTB to OUTA.  0 corresponds to full coast.
- `void setBrake(int brake)` <br> Set brake for motor.  Brake should be between 0 and 400.  0 corresponds to full coast, and 400 corresponds to full brake.
- `unsigned int getCurrentMilliamps()` <br> Returns current reading from motor in milliamps.  See the notes in the "Current readings" section below.
- `unsigned char getFault()` <br> Returns 1 if there is a fault on the G2 motor driver, 0 if no fault.

### Current readings

The current readings returned by `getCurrentMilliamps` will be noisy and unreliable if you are using
a PWM frequency below about 5&nbsp;kHz.  We expect these readings to work fine if you haven't remapped the PWM pins and you are using a board based on the ATmega168, ATmega328P, ATmega328PB, or ATmega32U4, since this library uses 20&nbsp;kHz hardware PWM on those boards.

On other boards, this library uses `analogWrite` to generate PWM signals, which usually means that the PWM frequency will be too low to get reliable current measurements.  If `analogWrite` uses a frequency of 490&nbsp;Hz or more on your board, you can add a 1&nbsp;&micro;F
(or larger) capacitor between each current sense line you are using and GND.  To make `getCurrentMilliamps` work well, you would add the capacitor between CS and GND.

## Version history

* 0.0.1 (2019-23-03): untested alpha release.

### Notes about timers and conflicts:
-------------------

-   Timer0 is used by the functions millis(), delay(), micros() and delayMicroseconds(),
-   Timer1 is used by the servo.h library on Arduino Uno
-   Timer5 is used by the servo library on Arduino Mega
-   the Servo library uses timers and interrupts
    -   you can’t use the pwm pins used for servo timers
    -   UNO: when you use the Servo Library you can’t use PWM on Pin 9, 10
    -   MEGA: it is a bit more difficult
        -   For the first 12 servos timer 5 will be used (losing PWM on Pins 44,45,46)
        -   for 13-24 servos timer 5 and 1 will be used (losing PWM on Pins 11,12,44,45,46)
        -   for 25-36 servos timer 5, 1 and 3 will be used (losing PWM on Pins 2,3,5,11,12,44,45,46)
        -   For 37-48 servos all 16bit timers 5,1,3 and 4 will be used (losing all PWM pins).
-   timers for PWM underwrite the function analogWrite(). Manually setting up a timer will/may stop analogWrite() from working
    -   You can obtain more control than the analogWrite function provides By manipulating the chip’s timer registers directly (note this is very advanced).
    -   The PWM outputs generated on pins 5, 6 (uno) or Pins 4, 13 (mega) will have higher-than-expected duty cycles. This is because of interactions with the millis() and delay() functions, which share the same internal timer used to generate those PWM outputs. This will be noticed mostly on low duty-cycle settings (e.g 0 - 10) and may result in a value of 0 not fully turning off the output on pins 5, 6 (Uno) or pins 4,13 (mega).
-   on the Uno Pin 11 has shared functionality PWM and MOSI. MOSI is needed for the SPI interface, so you can’t use PWM on Pin 11 and the SPI interface at the same time.
    -   on the MEGA SPI related pins are 50 (MISO), 51 (MOSI), 52 (SCK), 53 (SS) so no conflict exists
-   the tone(), notone() functions uses timer2,
-   Timer 2 has a different set of prescale values from the other timers,
-   Timer 0 and 2 are 8bit timers while timer 1, 3, 4, and 5 are 16 bit timers;
-   Timer 3,4,5 are only available on Arduino Mega boards,
-   The digitalWrite() function turns off PWM output if called on a timer pin
-   The Uno has 3 Timers and 6 PWM output pins 3, 5, 6, 9, 10, and 11
    -   timer 0 —– Pins 5, 6 (time functions like millis(), and delay() )
    -   timer 1 —– Pins 9, 10 (servo library)
    -   timer 2 —– Pins 11, 3 (tone(), notone() functions)

| Uno Timer | Timer output  | Timer bit mode | PWM pin output |     Conflicts    |
|:---------:|:-------------:|:-------------:|:--------------:|:----------------:|
|  Timer 1  |     OCR1A     |     16 bit    |        9       |   servo library  |
|  Timer 1  |     OCR1B     |     16 bit    |       10       |   servo library  |
|  Timer 2  |     OCR2A     |     8 bit     |       11       | tone(), notone(), MOSI/SPI |
|  Timer 2  |     OCR2B     |     8 bit     |        3       | tone(), notone() |

-   The Arduino Mega has 6 timers and 15 PWM outputs pins 2 to 13 and 44 to 46
    -   timer 0 —– pin 4, 13 (time functions like millis(), and delay() )
    -   timer 1 —– pin 11, 12
    -   timer 2 —– pin 9, 10 (tone(), notone() functions)
    -   timer 3 —– pin 2, 3, 5
    -   timer 4 —– pin 6, 7, 8
    -   timer 5 —– pin 44, 45, 46 (servo library)

| Mega Timer | Timer output OCRnx | Timer bit mode | PWM pin output |     Conflicts    |
|:----------:|:------------------:|:-------------:|:--------------:|:----------------:|
|   Timer 1  |        OCR1A       |     16 bit    |       11       |         -        |
|   Timer 1  |        OCR1B       |     16 bit    |       12       |         -        |
|   Timer 2  |        OCR2A       |     8 bit     |       10       | tone(), notone() |
|   Timer 2  |        OCR2B       |     8 bit     |        9       | tone(), notone() |
|   Timer 3  |        OCR3A       |     16 bit    |        5       |         -        |
|   Timer 3  |        OCR3B       |     16 bit    |        2       |         -        |
|   Timer 3  |        OCR3C       |     16 bit    |        3       |         -        |
|   Timer 4  |        OCR4A       |     16 bit    |        6       |         -        |
|   Timer 4  |        OCR4B       |     16 bit    |        7       |         -        |
|   Timer 4  |        OCR4C       |     16 bit    |        8       |         -        |
|   Timer 5  |        OCR5A       |     16 bit    |       46       | servo library    |
|   Timer 5  |        OCR5B       |     16 bit    |       45       | servo library    |
|   Timer 5  |        OCR5C       |     16 bit    |       44       | servo library    |

-   The Arduino Leonardo has 4 timers and 7 PWM outputs 
    -   timer 0 —– Pins 3, 11 (time functions like millis(), and delay() )
    -   timer 1 —– Pins 9, 10 (servo library)
    -   timer 3 —– pin 5
    -   timer 4 —– pin 6,13

-   [You can manually implement a sudo PWM on any digital pin by repeatedly turning the pin on and off for the desired times](http://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM)
