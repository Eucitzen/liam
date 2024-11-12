// Section for 9150 compass/gyro/accelerometer combo
// Changelog:
//     2014-12-12 - Initial version by Jonas
// 	   2024-09-02 - robotstuck detection added by Willie [experimental]
/* ============================================
Placed under the GNU license

===============================================
*/
#include <Wire.h>  // For Compass
#include <I2Cdev.h>
#include <Sensors.h>
#include "Definition.h"
#include <Arduino.h>

#ifndef _MS9150_H_
#define _MS9150_H_
#ifndef RobotStuckDetector_h
#define RobotStuckDetector_h
#ifndef __averaging__
int numReadings = 5
#else
int numReadings = 1


class MS9150 : public MOTIONSENSOR{
    public:
        explicit MS9150(int numReadings = 5);
        virtual boolean initialize();
        virtual void updateHeading();
        virtual int getHeading();
        virtual void setNewTargetHeading();
        virtual int headingVsTarget();
        virtual int getTiltAngle();
        
    private:
    	MPU9150 sensor;
        int _numReadings;

};

class RobotStuckDetector {
  public:
    explicit RobotStuckDetector(int stuckThreshold = 1000, int timeout = 2000, int numReadings = 5);
    void begin();
    bool isStuck();
  
  private:
    MPU9150Lib MPU;
    int _stuckThreshold;
    int _timeout;
    int _numReadings;
    unsigned long _previousMillis;

    void averageReadings(float &accelMagnitude, float &gyroMagnitude);
};

#endif
#endif
#endif /* _MS9150_H_ */

// This is the library for a Compass
// It uses a 9150 compass/gyro/accelerometer combo
// Changelog:
//     2014-12-12 - Initial version by Jonas

/* ============================================
Placed under the GNU license

===============================================
*/

#ifndef _MS5883L_H_
#define _MS5883L_H_

class MS5883L : public MOTIONSENSOR {
    public:
        explicit MS5883L();
        virtual boolean initialize();
        virtual void updateHeading();
        virtual int getHeading();
        virtual void setNewTargetHeading();
        virtual int headingVsTarget();
        virtual int getTiltAngle();
        
    private:
    	MS5883L sensor;

}

#endif /* _COMPASS_H_ */

/* This is the library for a BWFSensor
// 
// Changelog:
//     2014-12-14 - Initial version by Jonas

SelA	selection pin for the sensor swith
SelB	selection pin for the sensor swith
in[]	an array of integers for the inside code
out[]	an array of integers for the outside code
todel	the time until time out is signalled
nodel	the time until the sensor has lost a signal

============================================
Placed under the GNU license

===============================================
*/


#ifndef _BWFSENSOR_H_
#define _BWFSENSOR_H_

#define INSIDE		1
#define	NOSIGNAL	0
#define OUTSIDE		-1

// BWF Code for timout and no signal (in milliseconds)
#define TIMEOUT_DELAY					20000
#define NO_SIGNAL_DELAY					4000

class BWFSENSOR {
    public:
        BWFSENSOR(int selA, int selB);
        
        void select(int sensornumber);
        
        void attach(int intpin);
    	void readSensor();
    	
        bool isTimedOut();
        bool isInside();
        bool isOutside();
        bool hasNoSignal();
        
        void printSignal();
        void clearSignal();
        

    private:
    // BWF Code for inside and outside the fence
		static int inside_code[];
		static int outside_code[];
    	int selpin_A;
    	int selpin_B;
    	int counter;
    	int signal_status;
    	long last_interrupt;
    	long signal_status_checked;
    	long pulse_length;
    	long pulse_time;
    	int pulse_count_inside;
    	int pulse_count_outside;
    	int sensor_number;
    	const static int pulse_unit_length = 100;
    	const static int half_unit_length = 50;
    	const static int arr_length=10;
    	int arr[arr_length];
    	int arr_count;

};

#endif /* _BWFSENSOR_H_ */



// This is the library for a Compass
// It uses a 9150 compass/gyro/accelerometer combo
// Changelog:
//     2014-12-12 - Initial version by Jonas

/* ============================================
Placed under the GNU license

===============================================
*/

#ifndef _MOTIONSENSOR_H_
#define _MOTIONSENSOR_H_

class MOTIONSENSOR {
    public:
        virtual boolean initialize();
        virtual void updateHeading();
        virtual int getHeading();
        virtual void setNewTargetHeading();
        virtual int headingVsTarget();
        virtual int getTiltAngle();
        
    protected:
    	int current_heading;
    	int target_heading;
    	int tilt_angle;
    	int16_t mx, gx, ax;
    	int16_t my, gy, ay;
    	int16_t mz, gz, az;
};

#endif /* _MOTIONSENSOR_H_ */

