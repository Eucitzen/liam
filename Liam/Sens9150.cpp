/*
 Liam - DIY Robot Lawn Mower

 MPU-9150 Motion Sensor Library

 Changelog:
 2024-09-07 - added a robot stuck reading with future plans to obselete current readings off motors

 ======================
  Licensed under GPLv3
 ======================
*/

#include "Sens9150.h"
#include "Definition.h"


boolean MS9150::initialize() {
	boolean test;

  	Wire.begin();          				// Join I2C bus
  	Serial.begin(115200);   			// Set conmmunication rate
  	sensor.initialize();  					// Initialize the compass
  	test = sensor.testConnection();
  	current_heading = getHeading();

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



RobotStuckDetector::RobotStuckDetector(int stuckThreshold, int timeout) {
  _stuckThreshold = stuckThreshold;
  _timeout = timeout;
  _previousMillis = 0;
}

void RobotStuckDetector::initialize() {
	boolean test;

  	Wire.begin();          				// Join I2C bus
  	Serial.begin(115200);   			// Set conmmunication rate
  	sensor.initialize();  					// Initialize the compass
  	test = sensor.testConnection();
  	current_heading = getHeading();

	return test;						// true = connection OK
}
#include "Sens9150.h"

RobotStuckDetector::RobotStuckDetector(int stuckThreshold, int timeout, int numReadings) {
  _stuckThreshold = stuckThreshold;
  _timeout = timeout;
  _numReadings = numReadings;
  _previousMillis = 0;
}

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
    
    millis(10); // Small delay between readings for stability
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
