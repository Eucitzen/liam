# liam - DIY Robot lawn mower

Updates by Eu_citzen 2024-10-10 In development:
- Working (BETA) on a stuck detection function using MPU9150
- Obsticle avoidance using 3 ultrasonic sensors
- Merging several libraries into distinct categories
- Due to increased pin requirements it's being geared towards an STM32 Blackpill module instead of the usual Arduino Uno


Liam is an arduino project for the DIY Robot Lawn Mover.
This program will control your mower and relies on a two coil
configuration (0 and 1) with an optional (2).

The mower is assumed to be rear wheel driven and have three
boundary wire reciever coils in the following configuration

    wheel
    ----------------
    |           (0)|
    |(2)           |  ----> Mowing direction
    |           (1)|
    ----------------
    wheel

Licensed under GNU GPL 3.0

(c) 2017 Jonas Forssell & team

Configuration
------
Most of the default values for your mower can be set in the
Definition.h file. Use this to configure your mower for best
performance.

Configurations can also be put in a file called
`LocalDefinition.h`, which will override `Definition.h` but
is not tracked by git. This might be useful if you like to
stay on top of any code updates, but don't want to remake
the same configuration changes all the time.


Compile
------
The following libraries must be added to your arduino IDE to be able to compile the source

  * HMC5883L
  * I2Cdev
  * LiquidCrystal_I2C
  * MPU9150
  * RTClib
  * RTIMULib

  The libraries you need are bundled with the liam code within each release.

Change log
------
Change log
------
2025-03:
- Further cleaning up library code
- Started working on PCB using STM32F407VET6TR & ESP32 module

2024-12:
- Remade the libraries into relevant categories (e.g. "sensors", "display", "motioncontrol")
- Started removing superflourous libraries for newer Arduino versions
- Cleaning up code for STM32 microcontrollers, found even the STM32 Blackpill has too few pins

2024-11:
- Remade to suite a STM32 Blackpill board, new version needs more pins
- Merged several libraries into distinct categories
- Beta stuck detection using MPU9150
- Added and test printed 3D models for a larger mower


Links
------

[Wiki - work in progress](https://github.com/sm6yvr/liam/wiki)

[Facebook group DIY Robot Lawn Mower](https://www.facebook.com/groups/319588508137220/)

[Build instructions Trello](https://trello.com/b/gYQjoWY5/liam)
