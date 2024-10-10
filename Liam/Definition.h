/* This is the definition for all the defaults
 
 Change log:
 2024.08.29 added Blackpill support and ultrasonic sensor
 2024.09-08 Changing wheelmotor control and overcurrent protection [intend to deprecate to drv8871 motordrivers with builtin protection]
*/
/* ============================================
Placed under the GNU license

===============================================
*/
#include <Arduino.h>
#include "Wheelmotor.h"
#include "CutterMotor.h"
#include "Battery.h"


#ifndef _DEFINITION_H_
#define _DEFINITION_H_

const int NUMBER_OF_SENSORS = 3;	// Number of BWF sensors can be 1-4 depending on shield

// Pin setup following Morgan 1.5 shield and up
// Motor A; left
// Motor B & D; right
// Motor C; front or front left
#define WHEEL_MOTOR_A_CURRENT_PIN		B1  //ADC pins
#define WHEEL_MOTOR_B_CURRENT_PIN		B0 // ADC Pins
#define SOC_PIN							A5 //ADC
#define CUTTER_CURRENT_PIN				A4 //ADC
#define I2C_SDA_PIN						B9
#define I2C_SDL_PIN						B8  // Done
// Digital pins
#define RX_PIN							A10
#define TX_PIN							A9
#define BWF_SENSOR_INPUT_PIN			2
#define BWF_SELECT_A_PIN				A12
#define BWF_SELECT_B_PIN				A11
#define DOCK_PIN						A8
#define CUTTER_PWM_PIN					A7
//#define BUMPER							8   Obselete
#define LIFT_SENSOR_PIN					B12
#define LED_PIN							PC13
#define WHEEL_MOTOR_A_PWM_PIN			A1
#define WHEEL_MOTOR_A_DIRECTION_PIN		B13
#define WHEEL_MOTOR_B_PWM_PIN			A0
#define WHEEL_MOTOR_B_DIRECTION_PIN		B10
#define WHEEL_MOTOR_C_PWM_PIN           A3
#define WHEEL_MOTOR_C_DIRECTION_PIN     B14

// Ultrasonic pins
#define trigPinLeft                     B7
#define echoPinLeft                     B6
#define trigPinRight                    B5
#define echoPinRight                    B4
#define trigPinFront                    B3
#define echoPinFront                    A15


// Cutter motor types
//#define BRUSHLESS						0
#define BRUSHED							1
//#define NIDEC							2

// Battery
#define LEADACID	0
#define NIMH		1
#define LIION		2

// Wheel motors, comment out motorcurrent to use DRV8871 or other current limited motordrivers [needs MPU-9150 to check if Liam is stuck]
#define motorcurrent
#define WHEELMOTOR_OVERLOAD		130
#define WHEELMOTOR_SMOOTHNESS	300

// CUTTER
#define CUTTER_OVERLOAD			100

// Cutter states
const int MOWING = 0;
const int LAUNCHING = 1;
const int DOCKING = 2;
const int CHARGING = 3;

// Turning details
#define TURN_INTERVAL					15000
#define REVERSE_DELAY					2
#define TURNDELAY						20 //Reduce for smaller turning angle

// BWF Detection method (true = always, false = only at wire)
#define BWF_DETECTION_ALWAYS			true
#define TIMEOUT							6000 //Time without signal before error

// Trigger value for the mower to leave the BWF when going home
// The higher value the more turns (in the same direction) the mower can make before leaving
#define BALANCE_TRIGGER_LEVEL			10000

// Code for inside and outside
//#define INSIDE_BWF						103,4,103
//#define OUTSIDE_BWF						103,107,103			

// Version 2 of the BWF transmitter
#define INSIDE_BWF					85
#define OUTSIDE_BWF					5			

#define MAJOR_VERSION				4
#define MINOR_VERSION_1				9
#define MINOR_VERSION_2				1

// Obsticle avoidance 

// If you have a bumper connected to pin8, uncomment the line below. Remember to cut the brake connection on your motor shield
//#define __Bumper__	

// If you have 3 ultrasonic sensors of HC-SR04 uncomment the line below
#define __SR04__


// If you have a lift sensor connection to front wheel (connected to pin9), uncomment this line
//#define __Lift_Sensor__

// Do you have a Sensor? If so, uncomment one of these lines
//#define __MS5883L__
#define __MS9150__          // MPU 9150 accelerometer
#define __averaging__       //Averages the values of the MPU-9150 for stability

// Do you have a Display? If so, uncomment one of these lines
//#define __OLED__
//#define __LCD__

// Do you have a clock? If so, uncomment the line below
#define __RTC_CLOCK__
#define GO_OUT_TIME					16, 00
#define GO_HOME_TIME				22, 00

// Tiltangle
#define TILTANGLE						45

// Flipangle (turn off cutter and stop everything)
#define FLIPANGLE						75

// Motor Speeds
#define FULLSPEED						100
#define SLOWSPEED						30
#define CUTTERSPEED						100

class DEFINITION {
    public:
        void definePinsInputOutput();
        void setDefaultLevels(BATTERY* battery, WHEELMOTOR* left, WHEELMOTOR* right, CUTTERMOTOR* cutter);
        
    private:
};

#endif /* _DEFINITION_H_ */
