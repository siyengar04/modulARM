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
### Interrupts
Digital Pins 2, 3, 18, 19, 20, 21 
Avoid 14 - 19 inclusive as they are UART pins

### Motors
#### Motor 1
##### Driver:
- Direction = 7
- PWM = 11
- Sleep = 4
- FAULT = 6
- Current Sense = A0

##### Encoder:
- A: 2
- B: 3

Limit Switch: 21


#### Motor 2:
TODO


Limit Switch: 20