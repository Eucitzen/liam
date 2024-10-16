/// This is the library for a Controller
//
// Changelog:
//     2014-12-12 - Initial version by Jonas
//     2024-09-07 - Initial 3 wheel version and ultrasonic obsticle avoidance 

/* ============================================
Placed under the GNU license

===============================================
*/

#include "Controller.h"
#include "Definition.h"

/** Specific constructor.
 */
CONTROLLER::CONTROLLER(WHEELMOTOR* left, WHEELMOTOR* right, WHEELMOTOR* front, CUTTERMOTOR* cut, BWFSENSOR* bwf, MOTIONSENSOR* comp) {
    leftMotor = left;
    rightMotor = right;
	frontMotor = front;
    cutter = cut;
    sensor = bwf;
	compass = comp;
	default_dir_fwd = 1;
	balance = 0;
}


boolean CONTROLLER::allSensorsAreOutside() {
	
	for(int i=0; i<NUMBER_OF_SENSORS; i++) {
		sensor->select(i);
		if (!sensor->isOutside())
			return false;
	}

	return true;	
}

int CONTROLLER::turnToReleaseLeft(int angle) {
	turnLeft(angle);
	
	for (int i=0; i<20; i++) {
		sensor->select(1);

		if (sensor->isInside()) {
			sensor->select(0);
			if (sensor->isInside())
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
		sensor->select(0);

		if (sensor->isInside()) {
			sensor->select(1);
			if (sensor->isInside())
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
	
	leftMotor->setSpeed(default_dir_fwd*100);
	rightMotor->setSpeed(default_dir_fwd*-100);
	frontMotor->setSpeed(default_dir_fwd*100);
	
	delay(angle*TURNDELAY);

    return outcome;
}

int CONTROLLER::turnLeft(int angle) {
	int outcome = 0;
	
	leftMotor->setSpeed(default_dir_fwd*-100);
	rightMotor->setSpeed(default_dir_fwd*100);
	frontMotor->setSpeed(default_dir_fwd*100);
	
	delay(angle*TURNDELAY);

    return outcome;
}



int CONTROLLER::waitWhileChecking(int duration) {
  
  delay(200);   // Let any current spikes settle

  for (int i=0; i<duration/30; i++) {
    // check for problems
    if(leftMotor->isOverloaded()) 
      return 2;
    if(rightMotor->isOverloaded())
      return 2;
    if (sensor->isTimedOut())
      return 3;
    
    delay(turnDelay); 
    }

	// Successful delay
	return 0;
}



int CONTROLLER::waitWhileInside(int duration) {
  
  for (int k=0; k<duration/(NUMBER_OF_SENSORS*200); k++)
  	for (int i=0; i<NUMBER_OF_SENSORS; i++) {
    	sensor->select(i);
    	if(!sensor->isInside()) 
      		return 2;
  	}
  
  // Successful delay
  return 0;      
  }


void CONTROLLER::startCutter() {
	for (int i=0; i<CUTTERSPEED; i++) 
		cutter->setSpeed(i);
}

void CONTROLLER::stopCutter() {
	cutter->setSpeed(0);
}

void CONTROLLER::storeState() {
	leftMotorSpeed = leftMotor->getSpeed();
	rightMotorSpeed = rightMotor->getSpeed();
	cutterSpeed = cutter->getSpeed();
}

void CONTROLLER::restoreState() {
	leftMotor->setSpeed(leftMotorSpeed);
	rightMotor->setSpeed(rightMotorSpeed);
	cutter->setSpeed(cutterSpeed);
}

void CONTROLLER::runForward(int speed) {
	leftMotor->setSpeed(default_dir_fwd*speed);
	rightMotor->setSpeed(default_dir_fwd*speed);
	frontMotor->setSpeed(default_dir_fwd*speed);
}

void CONTROLLER::runBackward(int speed) {
	leftMotor->setSpeed(default_dir_fwd*-speed);
	rightMotor->setSpeed(default_dir_fwd*-speed);
	frontMotor->setSpeed(default_dir_fwd*-speed)
}

void CONTROLLER::setDefaultDirectionForward(bool fwd) {
	if (fwd == true)
		default_dir_fwd = 1;
	else
		default_dir_fwd = -1;
};

void CONTROLLER::adjustMotorSpeeds() {
  int  lms = abs(leftMotor->getSpeed());
  int  rms = abs(rightMotor->getSpeed());

  if (!sensor->isInside()) {
  	lms = 100;
  	rms = 10;
  }
  else 
  if (sensor->isInside())
  {
	lms = 10;
	rms = 100;
  }
  else {
    rms += 80;
    lms += 80;
  }

  if (rms > 100) rms = 100;
  if (lms > 100) lms = 100;
  if (rms < -50) rms = -50;
  if (lms < -50) lms = -50;

  leftMotor->setSpeed(default_dir_fwd*lms);
  rightMotor->setSpeed(default_dir_fwd*rms);
}

void CONTROLLER::updateBalance() {
	balance = balance + leftMotor->getSpeed() - rightMotor->getSpeed();
	
	if(balance > 0)
		balance-=10;
	else
		balance+=10;
}



void CONTROLLER::stop() {
	leftMotor->setSpeed(0);
	rightMotor->setSpeed(0);

}

int CONTROLLER::compensateSpeedToCutterLoad() {
	
}

int CONTROLLER::compensateSpeedToCompassHeading() {
	int  lms = abs(leftMotor->getSpeed());
	int  rms = abs(rightMotor->getSpeed());

    if (compass->headingVsTarget() < 0) {
    	rms = 0.9*rms;
        }
    else if (compass->headingVsTarget() > 0) {
        lms = 0.9*lms;
        }

	leftMotor->setSpeed(default_dir_fwd*lms);
	rightMotor->setSpeed(default_dir_fwd*rms);
}

boolean CONTROLLER::wheelsAreOverloaded() {
	delay(200);				// Settle current spikes
	if (leftMotor->isOverloaded() | rightMotor->isOverloaded())
		return true;
	else
		return false;
}

boolean CONTROLLER::hasBumped() {
	return !digitalRead(BUMPER);
}

boolean CONTROLLER::hasTilted() {
	return (compass->getTiltAngle() > TILTANGLE);
}

boolean CONTROLLER::hasFlipped() {
	return (compass->getTiltAngle() > FLIPANGLE);
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
        } else {
            turnRight();
        }
        millis(500);
    } else {
        Mower.runForward(FULLSPEED);
    }
}
