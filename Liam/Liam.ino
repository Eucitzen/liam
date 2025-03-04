
/* 
   Welcome to the Liam5_1 program
   This program will control your mower and relies on a two coil
   configuration (0 and 1) with an optional (2). 
   
   The mower is assumed to be rear wheel driven and have three 
   boundary wire reciever coils in the following configuration
   
   	wheel
    ----------------
   |			(0)	|
   |(2)         	|  ----> Mowing direction
   |			(1)	|
    ----------------
	wheel

	Most of the default values for your mower can be set in the 
	Definition.h file. Use this to configure your mower for best
	performance.

	(c) Jonas Forssell & team
	Free to use for all.
	
	Changes in this version
  - Removed OzOLED Support for Arduino101 Compatibility
  - -----------------------------------------------------------
	- Ultrasound sensor bumper support						(Planned)
	- More robust shutdown of mower if wheel overload		(Planned)
	- Revised Error messages								(Planned)
	- Support for OLED Display								(Planned)
	- Signal sensitivity factor in Definition.h				(Planned)
	- Slower mowing if cutter motor is using much current	(Planned)
	---------------------------------------------------------------
*/

#include <Servo.h>
#include <Wire.h>
#include "Battery.h"
#include "Clock.h"
#include "Error.h"
#include "Motioncontrol.h"
#include "Sensors.h"
#include "Display.h"
#include "Definitions.h"


// Global variables
int state;
long time_at_turning = millis();
int turn_direction = 1;
int LCDi = 0;


// Set up all the defaults (check the Definition.h file for all default values)
DEFINITION Defaults;

// Please select which type of cutter motor you have
// Types available: BRUSHED (for all brushed motors, A3620 and NIDEC 22)
// 					BRUSHLESS (for all hobbyking motors with external ESC)
//					NIDEC (for NIDEC 24)
CUTTERMOTOR CutterMotor(BRUSHED, CUTTER_PWM_PIN, CUTTER_CURRENT_PIN);

// Wheelmotors
WHEELMOTOR rightMotor(WHEEL_MOTOR_A_PWM_PIN, WHEEL_MOTOR_A_DIRECTION_PIN, WHEEL_MOTOR_A_CURRENT_PIN, WHEELMOTOR_SMOOTHNESS);
WHEELMOTOR leftMotor(WHEEL_MOTOR_B_PWM_PIN, WHEEL_MOTOR_B_DIRECTION_PIN, WHEEL_MOTOR_B_CURRENT_PIN, WHEELMOTOR_SMOOTHNESS);

// Battery
// Battery types available are LIION, LEAD_ACID, NIMH
BATTERY Battery(LIION, SOC_PIN, DOCK_PIN);

// BWF Sensors
BWFSENSOR Sensor(BWF_SELECT_B_PIN, BWF_SELECT_A_PIN);

// Compass
#if defined __MS5883L__
	MS5883L Compass;
#elif defined __MS9150__ 
	MS9150 Compass;
#else
	MOTIONSENSOR Compass;
#endif

// Controller (pass adresses to the motors and sensors for the controller to operate on)
CONTROLLER Mower(&leftMotor, &rightMotor, &CutterMotor, &Sensor, &Compass);

// Display
#if defined __LCD__ 
	myLCD Display(&Battery, &leftMotor, &rightMotor, &CutterMotor, &Sensor, &Compass, &state);
#else;
	MYDISPLAY Display(&Battery, &leftMotor, &rightMotor, &CutterMotor, &Sensor, &Compass, &state);
#endif

// RTC klocka
#if defined __RTC_CLOCK__
	CLOCK myClock;
#endif

// Error handler
ERROR Error(&Display, LED_PIN, &Mower);


// ****************** Setup **************************************
void setup()
{
	char buffer [9]; //Format 09.00.00

	Serial.begin(115200); 						// Fast communication on the serial port for all terminal messages

	Defaults.definePinsInputOutput();			// Configure all the pins for input or output
	Defaults.setDefaultLevels(&Battery, &leftMotor, &rightMotor, &CutterMotor); // Set default levels (defined in Definition.h) for your mower

	Display.initialize();							// Start up the display

	CutterMotor.initialize();
	Battery.resetSOC();							// Set the SOC to current value
	Compass.initialize();
	
	#if defined __RTC_CLOCK__
		myClock.initialize();
		myClock.setGoOutTime(GO_OUT_TIME);
		myClock.setGoHomeTime(GO_HOME_TIME);
	#endif

	attachInterrupt(0,updateBWF, RISING);		// Run the updateBWF function every time there is a pulse on digital pin2
	Sensor.select(0);

	if (Battery.isBeingCharged())	{			// If Liam is in docking station then
		state = CHARGING;						// continue charging
		Mower.stopCutter();
	}
	else {										// otherwise
		state = MOWING;							// start mowing
		Mower.startCutter();					// Start up the cutter motor
		Mower.runForward(FULLSPEED);	
	}

}



// ***************** Main loop ***********************************
void loop()
{
	boolean in_contact;
	boolean mower_is_outside;

	LCDi++;  //Loops 0-10
  	if (LCDi % 25 == 0 ){
		Display.update();
  	}
  	
  	// Security check Mower is flipped/lifted.
	#if defined __MS9150__ || defined __MS5883L__
    	if (Mower.hasFlipped())
    	{
    	 	Serial.print("Mower has flipped ");
    	  	Mower.stopCutter();
    	  	Mower.stop();
    	  	Error.flag(9);
		}   
	#endif
	
	#if defined __Lift_Sensor__
		if (Mower.isLifted())
		{
			Serial.println("Mower is lifted");
			Mower.stopCutter();
			Mower.stop();
			delay(500);
			Mower.runBackward(FULLSPEED);
			delay(2000);
   	  		if(Mower.isLifted())
   	  			Error.flag(4);
			Mower.turnRight(90);
			//Mover.startCutter();
   	  		Mower.runForward(FULLSPEED);
   		}
	#endif
  	
  	// Check if stuck and trigger action
  	Mower.updateBalance();

	if (abs(Mower.getBalance()) > BALANCE_TRIGGER_LEVEL) {
		Mower.storeState();
		Mower.runBackward(FULLSPEED);
		delay(1000);
		Mower.stop();
		Mower.restoreState();
		Mower.resetBalance();
		}
	
	switch (state) {
		
		//------------------------- MOWING ---------------------------
		case MOWING:
		
		Battery.updateSOC();
		Display.update();
	
		Sensor.select(0);

		if (BWF_DETECTION_ALWAYS)
			mower_is_outside = !Sensor.isInside();
		else
			mower_is_outside = Sensor.isOutside();

		// Check left sensor (0) and turn right if needed
		if (mower_is_outside) {
			Serial.println("Left outside");
    		Serial.println(Battery.getSOC());
    		Mower.stop();
    		
    		if (Battery.mustCharge()) {
	      		Mower.stopCutter();
	      		Mower.runForward(FULLSPEED);
	      		delay(1000);
	      		Mower.stop();
	      		Sensor.select(0);
       			state = DOCKING;
       			break;
    		}
    		
    		// Tries to turn, but if timeout then reverse and try again
			if (int err = Mower.turnToReleaseRight(30) > 0) {
				Mower.runBackward(FULLSPEED);
				delay(1000);
				Mower.stop();
				if (int err = Mower.turnToReleaseRight(30) > 0)
					Error.flag(err);
				}
			
			Compass.setNewTargetHeading();

    		if (Mower.allSensorsAreOutside()) { 
				Mower.runBackward(FULLSPEED);
				delay(1000);
				Mower.stop();
				if (Mower.allSensorsAreOutside())
    				Error.flag(4);
    		}
		}
		
		Sensor.select(1);

		if (BWF_DETECTION_ALWAYS)
			mower_is_outside = !Sensor.isInside();
		else
			mower_is_outside = Sensor.isOutside();
		
		// Check right sensor (1) and turn left if needed		
		if (mower_is_outside) {
			Serial.println("Right Outside");
			Serial.println(Battery.getSOC());
    		Mower.stop();

    		// Tries to turn, but if timeout then reverse and try again
			if (int err = Mower.turnToReleaseLeft(30) > 0) {
				Mower.runBackward(FULLSPEED);
				delay(1000);
				Mower.stop();
				if (int err = Mower.turnToReleaseLeft(30) > 0)
					Error.flag(err);
				}
				
			Compass.setNewTargetHeading();

    		if (Mower.allSensorsAreOutside()) { 
				Mower.runBackward(FULLSPEED);
				delay(1000);
				Mower.stop();
				if (Mower.allSensorsAreOutside())
    				Error.flag(4);
    		}
		}

	
		Mower.runForward(FULLSPEED);

		// Adjust the speed of the mower to the grass thickness
 		Mower.compensateSpeedToCutterLoad();

    	// Adjust the speed of the mower to the compass heading
   		Compass.updateHeading();
		Mower.compensateSpeedToCompassHeading();        


    	// Check if mower has hit something
    	if (Mower.wheelsAreOverloaded())
    	{
    	 	Serial.print("Wheel overload ");
    	  	Mower.runBackward(FULLSPEED);
    	  	if(Mower.waitWhileInside(2000) == 0);
    	  		Mower.turnRight(90);
    	  	Compass.setNewTargetHeading();
    	  	Mower.runForward(FULLSPEED);
		}   

		// Check if bumper has triggered (providing you have one enabled)
		#if defined __Bumper__
    		if (Mower.hasBumped())
    		{
    	 		Serial.print("Mower has bumped ");
    	  		Mower.runBackward(FULLSPEED);
    	  		delay(2000);
    	  		Mower.turnRight(90);
    	  		Mower.runForward(FULLSPEED);
			}   
		#endif

		// Use distance sensor HC-SR04 to avoid collision
		#if defined __SR04__
		 avoidObstacles robot(trigPinLeft, echoPinLeft, trigPinRight, echoPinRight, trigPinFront, echoPinFront);
		 robot.avoidObstacles();
	#endif

}
		 }

		#if defined __Lift_Sensor__
			if (Mower.isLifted())
			{
				Serial.println("Mower is lifted");
				Mower.stopCutter();
				Mower.runBackward(FULLSPEED);
				delay(2000);
    	  		if(Mower.isLifted())
    	  			Error.flag(4);
				Mower.turnRight(90);
    	  		Mower.startCutter();
    	  		Mower.runForward(FULLSPEED);
			}
		#endif

		// Check if mower has tilted (providing you have one enabled)
		#if defined __MS9150__ || defined __MS5883L__
    		if (Mower.hasTilted())
    		{
    	 		Serial.print("Mower has tilted ");
    	  		Mower.runBackward(FULLSPEED);
    	  		delay(2000);
    	  		Mower.turnRight(90);
    	  		Mower.runForward(FULLSPEED);
    	  		delay(200);
			}   

		// Check if mower has flipped (providing you have one enabled)
    		if (Mower.hasFlipped())
    		{
    	 		Serial.print("Mower has flipped ");
    	  		Mower.stopCutter();
    	  		Mower.stop();
    	  		Error.flag(9);
			}   
		#endif


		break;


		//----------------------- LAUNCHING ---------------------------
		case LAUNCHING:

    	Mower.runBackward(FULLSPEED);

    	delay(7000);
    	Mower.stop();

    	// Turn right in random degree
    	Mower.turnRight(random(30,60));
    	Mower.startCutter();
    	Mower.waitWhileChecking(5000);      

      	Compass.setNewTargetHeading();

    	Mower.runForward(FULLSPEED);

    	state = MOWING;

    	// Reset the running average
    	Battery.resetSOC();  

		break;

		//----------------------- DOCKING -----------------------------
		case DOCKING:

			//Make the wheel motors extra responsive
			leftMotor.setSmoothness(10);
			rightMotor.setSmoothness(10);

			// If the mower hits something, reverse and try again
			if (Mower.wheelsAreOverloaded()){
				Mower.runBackward(FULLSPEED);
				delay(1000);
			}

			// Track the BWF by compensating the wheel motor speeds
			Mower.adjustMotorSpeeds();
			
			// Clear signal to allow the mower to track the wire closely
			Sensor.clearSignal();
			
			// Wait a little to avoid current spikes
			delay(100);
			
			// Stop the mower as soon as the charge plates come in contact
			if (Battery.isBeingCharged()) {
				// Stop
				Mower.stop();
				Mower.resetBalance();
				state = CHARGING;
				break;
			}


		break;

		//----------------------- CHARGING ----------------------------
		case CHARGING:
		// restore wheelmotor smoothness
		leftMotor.setSmoothness(WHEELMOTOR_SMOOTHNESS);
		rightMotor.setSmoothness(WHEELMOTOR_SMOOTHNESS);

		// Just remain in this state until battery is full
		#if defined __RTC_CLOCK__
			if (Battery.isFullyCharged() && myClock.timeToCut()) 
			state = LAUNCHING;
		#else
			if (Battery.isFullyCharged()) 
			state = LAUNCHING;
		#endif
		
		in_contact = false;
		
		// Spend 20 seconds collecting status if being charged
		for (int i=0; i<20; i++) {
			if (Battery.isBeingCharged())
				in_contact = true;
			delay(1000);
		}
		
		// If the mower is not being charged, jiggle it a bit
		if (!in_contact) { 
			Mower.runBackward(20); 	// Back away slow speed 
			delay(500);
			Mower.runForward(20);	// Dock again at slow speed
			delay(1000);
			Mower.stop();
		}

	 	Serial.print("SOC:");
		Serial.println(Battery.getSOC());

		break;

	}
}

// This function calls the sensor object every time there is a new signal pulse on pin2
void updateBWF() {
	Sensor.readSensor();
}

