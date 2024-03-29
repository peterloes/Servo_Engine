﻿# SERVO Engine

A standalone platform for animal manipulation. 

This is the firmware for project Servo_2015, a small board which allows to
control a servo via its PWM input. Setup and control of the servo is done
via 3 keys, and, for selecting the two end positions, also via two external
inputs, please see below. The firmware supports soft-start, i.e. the servo
speed will be ramped-up during the start phase.

Technical specifications:
- Input voltage: 12-42V
- Standby current consumption: 3mA@12V
- Maximum current consuption: 1.5A
- Dimensions: 60 × 50 × 10 mm

Supported:
- HiTEC HS-5646WP
- HiTEC HS 785(sailing winds)

![My image](https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/2_Electronic_board.jpg)
 
 Setup Mode
 
 The servo setup procedure consists of 4 steps:
 - Adjust end position 1.  This can be the most left or the most right end
   point.  This position will be reached after power-up if valid servo data
   has been found in FLASH.
 - Adjust end position 2.  This is the opposite end position to position 1.
 - Adjust servo speed.  The servo permanently moves between the two end
   points.  The speed can be adjusted via S1 and S2.
 - Asserting S3 the 4th time stores the servo parameters into FLASH and
   returns to normal operation


![My image](https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/1_MOMO_SHUTTER.jpg)

Bird Feeder Application (MOMO)

![My image](https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/1_MOMO_SHUTTER_1.jpg)

Bird Feeder Application

https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/1_poster_overview.pdf

Mainboard: 

https://github.com/peterloes/MOMO

https://github.com/peterloes/TAMDL

Manufacture:

https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/3_Supplier.txt

EXTENSIONS:

Electrical applications for non-deadly animal traps.

https://github.com/peterloes/SmartAnimalTraps

Antenna (horizontal) rotation over 360 degrees (HiTEC HS 785//sailing winds; 12V@25mA).

https://github.com/peterloes/Servo_Engine/blob/master/Getting_Started_Tutorial/2_Servo_Antenna_3turns.jpg (New firmware is not uploaded.)

Actuator for linear movements.

https://github.com/peterloes/Linear_Engine
