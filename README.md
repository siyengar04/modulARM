# modulARM Microcontroller Code

PID library: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
G2MotorDriver: https://github.com/photodude/G2MotorDriver/tree/master

## Motor
http://www.cqrobot.wiki/index.php/DC_Gearmotor_SKU:_CQR37D
Wires:

White --> Encoder B output

Yellow --> Encoder A output

Blue --> Encoder VCC (3.3V - 24V)

Gray --> Encoder GND

Black --> Motor Power (can be + or -)

Red --> "                          "

## Arduino MEGA 2560 Pinmap
[Pinout](https://images.thingbits.net/eyJidWNrZXQiOiJ0aGluZ2JpdHMtbmV0Iiwia2V5IjoiZGVzY3JpcHRpb24vYXJkdWluby1tZWdhMjU2MC1waW5vdXQucG5nIn0=)
### Interrupts
Limit1 B (20)
Limit2 B (21)
Encoder1 A 
Encoder1 B 
Encoder2 A 
Encoder2 B 


Digital Pins 2, 3, 18, 19, 20, 21 
Avoid 14 - 19 inclusive as they are UART pins

### Timers
Motor1 PWM (2)
Motor2 PWM (3)

Can't use Timer 1, 2
#### Timer 3
3A: 5 (Motor1 PWM)<br> 
3B: 2 
3C: 3 

#### Timer 4
4A: 6 (Motor2 PWM) <br>
4B: 7  <br>
4C: 8 <br>


### Digital Pins
Motor1 DIR (22) <br>
Motor2 DIR (23) 

## Teensy 4.1 Documentation
https://www.pjrc.com/store/teensy41.html 

### Motors
#### Motor 1
Motor1 DIR: 22 <br>
Motor1 PWM: 5 <br> 
Encoder1 A: 2 <br>
Encoder1 B: 3 <br>
Limit1 B: 20 <br>

#### Motor 2:
Motor2 DIR: 23 <br>
Motor2 PWM: 6 <br>
Encoder2 A: 18 <br>
Encoder2 B: 19 <br>
Limit2 B: 21
