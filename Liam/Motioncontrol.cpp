/* This is the library for motor control and obsticle avoidance

 Changelog:
    2014-12-12 - Initial version by Jonas
    2024-10-13 - Remodeled by Willie
*/
/* ============================================
Placed under the GNU license

===============================================
*/


#include "Motioncontrol.h"
#include "Definition.h"

/** Wheelmotor controller. */

CONTROLLER::Wheelmotor(Wheelmotor& left, Wheelmotor& right, Wheelmotor& front, Boundarysens& bwf, Motionsens& comp):
    leftMotor(left),
    rightMotor(right),
	frontMotor(front),
    cutter(cut),
    sensor(bwf),
    compass(comp)
{
    default_dir_fwd = 1;
    balance = 0;
}

boolean CONTROLLER::allSensorsAreOutside() {
	
	for(int i=0; i<NUMBER_OF_SENSORS; i++) {
		sensor.select(i);
		if (!sensor.isOutside())
			return false;
	}

	return true;	
}

int CONTROLLER::turnToReleaseLeft(int angle) {
	turnLeft(angle);
	
	for (int i=0; i<20; i++) {
		sensor.select(1);

		if (sensor.isInside()) {
			sensor.select(0);
			if (sensor.isInside())
				return 0;				// OK
		}
		
		if (wheelsAreOverloaded())
			return 1;					// Overloaded
		
		turnLeft(10);
	}
	
	return 2;							// Timed Out
}

int CONTROLLER::turnToReleaseRight(int angle) {
	turnRight(angle);
	
	for (int i=0; i<20; i++) {
		sensor.select(0);

		if (sensor.isInside()) {
			sensor.select(1);
			if (sensor.isInside())
				return 0;				// OK
		}
		
		if (wheelsAreOverloaded())
			return 1;					// Overloaded
		
		turnRight(10);
	}
	
	return 2;							// Timed Out
}

int CONTROLLER::turnRight(int angle) {
	int outcome = 0;
	
	leftMotor.setSpeed(default_dir_fwd*100);
	rightMotor.setSpeed(default_dir_fwd*-100);
	frontMotor.setSpeed(default_dir_fwd*100)
	
	delay(angle*TURNDELAY);

    return outcome;
}


int CONTROLLER::turnLeft(int angle) {
	int outcome = 0;
	
	leftMotor.setSpeed(default_dir_fwd*-100);
	rightMotor.setSpeed(default_dir_fwd*100);
	frontMotor.setSpeed(default_dir_fwd*100)
	
	delay(angle*TURNDELAY);

    return outcome;
}

int CONTROLLER::waitWhileChecking(int duration) {
  
  delay(200);   // Let any current spikes settle

  for (int i=0; i<duration/30; i++) {
    // check for problems
    if(leftMotor.isOverloaded()) 
      return 2;
    if(rightMotor.isOverloaded())
      return 2;
	if(frontMotor.isOverloaded())
	 return 2;
    if (sensor.isTimedOut())
      return 3;
    
    delay(turnDelay); 
    }

	// Successful delay
	return 0;
}

int CONTROLLER::waitWhileInside(int duration) {
  
  for (int k=0; k<duration/(NUMBER_OF_SENSORS*200); k++)
  	for (int i=0; i<NUMBER_OF_SENSORS; i++) {
    	sensor.select(i);
    	if(!sensor.isInside()) 
      		return 2;
  	}
  
  // Successful delay
  return 0;      
}

void CONTROLLER::startCutter() {
	for (int i=0; i<CUTTERSPEED; i++) 
		cutter.setSpeed(i);
}

void CONTROLLER::stopCutter() {
	cutter.setSpeed(0);
}

void CONTROLLER::storeState() {
	leftMotorSpeed = leftMotor.getSpeed();
	rightMotorSpeed = rightMotor.getSpeed();
	frontMotorSpeed = frontMotor.getSpeed();
	cutterSpeed = cutter.getSpeed();
}

void CONTROLLER::restoreState() {
	leftMotor.setSpeed(leftMotorSpeed);
	rightMotor.setSpeed(rightMotorSpeed);
	frontMotor.setSpeed(frontMotorSpeed);
	cutter.setSpeed(cutterSpeed);
}

void CONTROLLER::runForward(int speed) {
	leftMotor.setSpeed(default_dir_fwd*speed);
	rightMotor.setSpeed(default_dir_fwd*speed);
	frontMotor.setSpeed(default_dir_fwd*speed);
}

void CONTROLLER::runBackward(int speed) {
	leftMotor.setSpeed(default_dir_fwd*-speed);
	rightMotor.setSpeed(default_dir_fwd*-speed);
	frontMotor.setSpeed(default_dir_fwd-speed);
}

void CONTROLLER::setDefaultDirectionForward(bool fwd) {
	if (fwd == true)
		default_dir_fwd = 1;
	else
		default_dir_fwd = -1;
};

void CONTROLLER::adjustMotorSpeeds(); {
  int  lms = abs(leftMotor.getSpeed());
  int  rms = abs(rightMotor.getSpeed());
  int  fms = abs(frontMotor.getSpeed());


  if (!sensor.isInside()) {
  	lms = 100;
  	rms = 10;
	fms = 10;
  }
  else 
  if (sensor.isInside())
  {
	lms = 10;
	rms = 100;
	fms = 10
  }
  else {
    rms += 80;
    lms += 80;
	fms +=80;
  }

  if (rms > 100) rms = 100;
  if (lms > 100) lms = 100;
  if (fms > 100) fms = 100;
  if (rms < -50) rms = -50;
  if (lms < -50) lms = -50;
  if (fms < -50) fms = -50;
  leftMotor.setSpeed(default_dir_fwd*lms);
  rightMotor.setSpeed(default_dir_fwd*rms);
}

void CONTROLLER::updateBalance() {
	balance = balance + leftMotor.getSpeed() - rightMotor.getSpeed();
	
	if(balance > 0)
		balance-=10;
	else
		balance+=10;
}



void CONTROLLER::stop() {
	leftMotor.setSpeed(0);
	rightMotor.setSpeed(0);
	frontMotor.setSpeed(0);

}

int CONTROLLER::compensateSpeedToCutterLoad() {
	
}

int CONTROLLER::compensateSpeedToCompassHeading() {
	int  lms = abs(leftMotor.getSpeed());
	int  rms = abs(rightMotor.getSpeed());
	int  fms = abs(frontMotor.getSpeed());

    if (compass.headingVsTarget() < 0) {
    	rms = 0.9*rms;
        }
    else if (compass.headingVsTarget() > 0) {
        lms = 0.9*lms;
        }

	leftMotor.setSpeed(default_dir_fwd*lms);
	rightMotor.setSpeed(default_dir_fwd*rms);
    frontMotor.setSpeed(default_dir_fwd*fms);
}

olean CONTROLLER::wheelsAreOverloaded() {
	delay(200);				// Settle current spikes
	if (leftMotor.isOverloaded() | rightMotor.isOverloaded | frontMotor.isOverloaded())
		return true;
	else
		return false;

boolean CONTROLLER::hasBumped() {
	return !digitalRead(BUMPER);
}

boolean CONTROLLER::hasTilted() {
	return (compass.getTiltAngle() > TILTANGLE);
}

boolean CONTROLLER::hasFlipped() {
	return (compass.getTiltAngle() > FLIPANGLE);
}

boolean CONTROLLER::isLifted() {
	return !digitalRead(LIFT_SENSOR_PIN);
}

void CONTROLLER::resetBalance() {
	balance = 0;
}

int CONTROLLER::getBalance() {
	return balance;
}

int CONTROLLER::avoidObstacles(int trigPinLeft, int echoPinLeft, int trigPinRight, int echoPinRight, int trigPinFront, int echoPinFront) {
    _trigPinLeft = trigPinLeft;
    _echoPinLeft = echoPinLeft;
    _trigPinRight = trigPinRight;
    _echoPinRight = echoPinRight;
    _trigPinFront = trigPinFront;
    _echoPinFront = echoPinFront;
}

long avoidObstacles::getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

void avoidObstacles::avoidObstacles() {
    long distanceFront = getDistance(_trigPinFront, _echoPinFront);
    long distanceLeft = getDistance(_trigPinLeft, _echoPinLeft);
    long distanceRight = getDistance(_trigPinRight, _echoPinRight);
    
    if (distanceFront < 5) {
        Mower.stop();
		millis(200);
        if (distanceLeft > distanceRight) {
            turnLeft();
        } 
        if(distanceleft < distanceRight) {
            turnRight();
        }
        if(distanceFront < 5){
        millis(500);
        if(distanceFront < 5)
        turnRight();    
        }
    } else {
        Mower.runForward(FULLSPEED);
    }
}

/* Cutter Motor

The motor can be either brushed or brushless

Motor speed is defined as percentage of full speed.
A speed of 100 means full speed and 0 is stop.
Speed can also be negative if reverse direction is requested.

Current draw of the motor can be read using the getLoad() method

// Changelog:
//     2014-12-13 - Initial version by Jonas

============================================
Placed under the GNU license

===============================================
*/

CUTTERMOTOR::CUTTERMOTOR(int type_, int pwmpin_, int loadpin_) {
    type = type_;
    pwmpin = pwmpin_;
    loadpin = loadpin_;

}
void CUTTERMOTOR::initialize() {
	// Initialize if brushless with ESC
	if (type == 0) {
		cutter.attach(pwmpin);
		cutter.writeMicroseconds(2000);
		delay(400);
		cutter.writeMicroseconds(1000);
		delay(2000);
	}	
}


void CUTTERMOTOR::setSpeed(int setspeed) {
   speed = setspeed;
   if (speed > 100)	speed = 100;
   if (speed < 0) speed = 0;
   
   if (type == 0)			// brushless
   	pwm = abs(10*speed)+1000;
   else if (type == 1)		// brushed
   	pwm = abs(2.55*speed);
   else if (type == 2)		// Nidec
    pwm = 255 - abs(2.55*speed);
    
   braking = false;
   
   if (type == 0)
   	cutter.writeMicroseconds(pwm);
   else
   	analogWrite(pwmpin, pwm);
}


int CUTTERMOTOR::getSpeed() {
	return speed;
}


int CUTTERMOTOR::getLoad() {
	return analogRead(loadpin);
}



void CUTTERMOTOR::brake() {
	setSpeed(0);
	braking = true;
}


bool CUTTERMOTOR::isBraking() {
	return braking;
}

bool CUTTERMOTOR::isOverloaded() {
	return (getLoad() > overload_level);
}

void CUTTERMOTOR::setOverloadLevel(int level) {
	overload_level = level;
}

/* This part is for the wheelmotors

Motor speed is defined as percentage of full speed.
A speed of 100 means full speed and 0 is stop.
Speed can also be negative if reverse direction is requested.

Current draw of the motor can be read using the getLoad() method

// Changelog:
//     2014-12-13 - Initial version by Jonas

============================================
Placed under the GNU license

===============================================
*/

WHEELMOTOR::WHEELMOTOR(int pwmpin_, int dirpin_, int loadpin_, int smoothness):
    pwmpin_(pwmpin_),
    dirpin_(dirpin_),
    loadpin_(loadpin_),
    smoothness_delay(smoothness)
    {
    // Set up PWM frequency for pin 3 and 11 to 3921 Hz for smooth running
	// At default pwm, the frequency of the motors will disturb the BWFsensors
	// This is very evident if using chinese wheelmotors
    // 	TCCR2B = TCCR2B & 0b11111000 | 0x02;	
    }

// Sets speed of wheelmotors using a percentage of a PWM signal
    void WHEELMOTOR::setSpeed(int setspeed) {
  	int diff = 1-2*((setspeed-speed) < 0);
   	int stepnr = abs(setspeed-speed);

   	if (setspeed > 100)	setspeed = 100;
   	if (setspeed < -100) setspeed = -100;

   	dir = (setspeed > 0);

   	for (int i=0; i<stepnr; i++){
   		speed += diff;
	   	analogWrite(pwmpin, 2.55*abs(speed));
   		digitalWrite(dirpin, (speed > 0));
   		delayMicroseconds(smoothness_delay);					// Smooth ramping of motors
   	}
  
  	speed = setspeed;
}

int WHEELMOTOR::getSpeed() {
	return speed;
}


int WHEELMOTOR::getLoad() {
	load = 0;
	
	for (int i=0; i<10; i++) { 
		load += analogRead(loadpin);
		delay(1);
	}

	return load/10;
}


bool WHEELMOTOR::isOverloaded() {
	return (getLoad() > overload_level);
}

void WHEELMOTOR::setOverloadLevel(int level) {
	overload_level = level;
}

void WHEELMOTOR::setSmoothness(int level) {
	smoothness_delay = level;
}
