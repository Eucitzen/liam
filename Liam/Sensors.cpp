/// This is the collected sensor library for Liam
//
// Changelog:
//     2014-12-12 - Initial version by Jonas
// 	   2024-09-02 - MPU-9150 motion detection beta by Willie, collecting all sensors to one library

/* ============================================
Placed under the GNU license

===============================================
*/

#include "Sensors.h"
#include "Definition.h"

// MPU-9150 sensor library

boolean MS9150::initialize() {
	if (sensor == nullptr) {
		Serial.println(F("sensor is null"));
		return false;
	}

	boolean test;

  	Wire.begin();          				// Join I2C bus
  	Serial.begin(115200);   			// Set conmmunication rate
  	try {
  		sensor.initialize();  					// Initialize the compass
  		test = sensor.testConnection();
  		current_heading = getHeading();
  	} catch (const std::exception& e) {
  		Serial.print(F("Exception: "));
  		Serial.println(e.what());
  		return false;
  	}

	return test;						// true = connection OK
}

void MS9150::updateHeading() {
    current_heading = getHeading();
}

void MS9150::setNewTargetHeading() {
	for (int i=0; i<10; i++)			// take 10 readings to get a stable value
		target_heading = getHeading();
}

int MS9150::headingVsTarget() {				// Will return angle difference between heading and target
	
	return 	copysign(1.0,current_heading - target_heading) *
			copysign(1.0,abs(current_heading-target_heading)-180) *
			(180-abs(abs(current_heading-target_heading)-180));
	
}

int MS9150::getHeading() {
	int16_t nmx, nmy, nmz;

  	sensor.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &nmx, &nmy, &nmz);
  
  	mx += 0.2*(nmx-mx);
  	my += 0.2*(nmy-my);
  	
  	float heading = atan2(my,mx);
  	if (heading < 0)
    	heading += 2*3.1415;
    
  	return heading * 180/3.1415;
}

int MS9150::getTiltAngle() {
  sensor.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  
  float tiltxz = abs(atan2(ax,az));
  float tiltyz = abs(atan2(ay,az));

  if (tiltxz > tiltyz)
	return tiltxz * 180 / 3.1415;
  else
    return tiltyz * 180 / 3.1415;
} 

// MPU-9150 motion detection 

#ifndef __MS9150motion__
RobotStuckDetector::RobotStuckDetector(int stuckThreshold, int timeout) :
  _stuckThreshold(stuckThreshold), _timeout(timeout), _previousMillis(0) {}

void RobotStuckDetector::initialize() {
  boolean test = false;

  Wire.begin();          				// Join I2C bus
  Serial.begin(115200);   			// Set conmmunication rate
  if (sensor.initialize()) {
    test = sensor.testConnection();
    current_heading = getHeading();
  }

  return test;						// true = connection OK
}

RobotStuckDetector::RobotStuckDetector(int stuckThreshold, int timeout, int numReadings) :
  _stuckThreshold(stuckThreshold), _timeout(timeout), _numReadings(numReadings), _previousMillis(0) {}

void RobotStuckDetector::begin() {
  Wire.begin();
  if (MPU.begin() != INV_SUCCESS) {
    Serial.println("MPU-9150 initialization failed!");
    while (1);
  }
  
  MPU.calibrateMPU();
  Serial.println("MPU-9150 initialized successfully.");
}

void RobotStuckDetector::averageReadings(float &accelMagnitude, float &gyroMagnitude) {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < _numReadings; i++) {
    MPU.readMPU();
    axSum += MPU.getRawAccelX();
    aySum += MPU.getRawAccelY();
    azSum += MPU.getRawAccelZ();
    gxSum += MPU.getRawGyroX();
    gySum += MPU.getRawGyroY();
    gzSum += MPU.getRawGyroZ();
    
    delay(10); // Small delay between readings for stability
  }

  // Calculate the average magnitudes
  accelMagnitude = sqrt((axSum / _numReadings) * (axSum / _numReadings) +
                        (aySum / _numReadings) * (aySum / _numReadings) +
                        (azSum / _numReadings) * (azSum / _numReadings));

  gyroMagnitude = sqrt((gxSum / _numReadings) * (gxSum / _numReadings) +
                       (gySum / _numReadings) * (gySum / _numReadings) +
                       (gzSum / _numReadings) * (gzSum / _numReadings));
}

bool RobotStuckDetector::isStuck() {
  unsigned long currentMillis = millis();
  
  float accelMagnitude = 0;
  float gyroMagnitude = 0;

  // Get the average readings
  averageReadings(accelMagnitude, gyroMagnitude);
  
  // Check if the robot is stuck
  if (accelMagnitude < _stuckThreshold && gyroMagnitude < _stuckThreshold) {
    if (currentMillis - _previousMillis >= _timeout) {
      return true;
    }
  } else {
    // Reset the timer if movement is detected
    _previousMillis = currentMillis;
  }
  
  return false;
}
#endif

/// This is the library for an LCD display
//
// Changelog:
//     2014-12-12 - Initial version by Jonas

/* ============================================
Placed under the GNU license

===============================================
*/


boolean MS5883L::initialize() {
	boolean test;

  	Wire.begin();          				// Join I2C bus
  	Serial.begin(115200);   			// Set conmmunication rate
  	sensor.initialize();  					// Initialize the compass
  	test = sensor.testConnection();
  	current_heading = getHeading();

	return test;						// true = connection OK
}

void MS5883L::updateHeading() {
    current_heading = getHeading();
}

void MS5883L::setNewTargetHeading() {
	for (int i=0; i<10; i++)			// take 10 readings to get a stable value
		target_heading = getHeading();
}

int MS5883L::headingVsTarget() {				// Will return angle difference between heading and target
	
	return 	copysign(1.0,current_heading - target_heading) *
			copysign(1.0,abs(current_heading-target_heading)-180) *
			(180-abs(abs(current_heading-target_heading)-180));
	
}

int MS5883L::getHeading() {
	int16_t nmx, nmy, nmz;

  	sensor.getHeading(&nmx, &nmy, &nmz);
  
  	mx += 0.2*(nmx - mx);
  	my += 0.2*(nmy - my);
  
  	float heading = atan2(my,mx);
  	if (heading < 0)
		heading += 2*3.1415;
    
  	return heading * 180/3.1415;
}

int MS5883L::getTiltAngle() {
	return 0;
}

/// This is the library for the boundrary wire detection (BWF sensor)
//
// Changelog:
//     2014-12-14 - Initial version by Jonas

/* ============================================
Placed under the GNU license

===============================================
*/
int BWFSENSOR::outside_code[] = {OUTSIDE_BWF};
int BWFSENSOR::inside_code[] = {INSIDE_BWF};	

/** Specific constructor.
 */
BWFSENSOR::BWFSENSOR(int selA, int selB) :
	selpin_A(selA),
	selpin_B(selB)
{
BWFSENSOR::BWFSENSOR(int selA, int selB) {
	selpin_A = selA;
	selpin_B = selB;
}
// select this sensor to be active
void BWFSENSOR::select(int sensornumber) {
	if (selpin_A != 0) {
		digitalWrite(selpin_A, (sensornumber & 1) > 0 ? HIGH : LOW);
	}
	if (selpin_B != 0) {
		digitalWrite(selpin_B, (sensornumber & 2) > 0 ? HIGH : LOW);
	}
	clearSignal();
	delay(200);			// Wait a little to collect signal
}
   digitalWrite(selpin_A, (sensornumber & 1) > 0 ? HIGH : LOW);
   digitalWrite(selpin_B, (sensornumber & 2) > 0 ? HIGH : LOW);
   clearSignal();
   delay(200);			// Wait a little to collect signal
   }


void BWFSENSOR::clearSignal() {
	for (int i=0; i<arr_length; i++)
		arr[i]=0;
	signal_status = NOSIGNAL;
	pulse_count_inside = 0;
	pulse_count_outside = 0;
	arr_count = 0;
}


bool BWFSENSOR::isInside() {
	return (signal_status == INSIDE);
}

bool BWFSENSOR::isOutside() {
	return (signal_status == OUTSIDE);
}

bool BWFSENSOR::isTimedOut() {
	return (signal_status_checked + TIMEOUT_DELAY < millis());
}

bool BWFSENSOR::hasNoSignal() {
	return (signal_status_checked + NO_SIGNAL_DELAY < millis());
}

// This routine is run at every timer interrupt and updates the sensor status
void BWFSENSOR::readSensor() {
  volatile int pulse_unit = 0;
  
  // Calculate the time since last pulse
  pulse_length = int(micros() - pulse_time);
  pulse_time = micros(); 
  pulse_unit = (pulse_length+half_unit_length) / pulse_unit_length;


  // Store the numbers for debug printout
  arr[arr_count++] = pulse_unit;
  	if (arr_count>arr_length) arr_count=0;

  // Check if the latest pulse fits the code for inside
  if (abs(pulse_unit-inside_code[pulse_count_inside]) < 2) {
    pulse_count_inside++;
    // If the whole code sequence has been OK, then set signal status to 1
    if (pulse_count_inside >= sizeof(inside_code)/sizeof(inside_code[0])) {
  		signal_status = INSIDE;
  		signal_status_checked = millis();
      	pulse_count_inside=0;
    }
  }
  else
    pulse_count_inside=0;

  // Check if the latest pulse fits the code for outside
  if (abs(pulse_unit-outside_code[pulse_count_outside]) < 2) {
    pulse_count_outside++;
    if (pulse_count_outside >= sizeof(outside_code)/sizeof(outside_code[0])) {
 		signal_status = OUTSIDE;
  		signal_status_checked = millis();
      	pulse_count_outside=0;
    }
  }
  else
    pulse_count_outside=0;

}

void BWFSENSOR::printSignal() {
	
	for (int i=0; i<arr_length; i++) {
		Serial.print(arr[i]);
		Serial.print(" ");
	}

}

//This is the library for any type of motion sensor
//
// Changelog:
//     2014-12-12 - Initial version by Jonas

/* ============================================
Placed under the GNU license

===============================================
*/


boolean MOTIONSENSOR::initialize() {
	return true;						// true = connection OK
}


void MOTIONSENSOR::updateHeading() {
    current_heading = getHeading();
}

void MOTIONSENSOR::setNewTargetHeading() {
	for (int i=0; i<10; i++)			// take 10 readings to get a stable value
		target_heading = getHeading();
}

int MOTIONSENSOR::headingVsTarget() {				// Will return angle difference between heading and target
	return 	copysign(1.0,current_heading - target_heading) *
			copysign(1.0,abs(current_heading-target_heading)-180) *
			(180-abs(abs(current_heading-target_heading)-180));
}

int MOTIONSENSOR::getHeading() {
  return 0;
}

int MOTIONSENSOR::getTiltAngle() {
	return 0;
}
