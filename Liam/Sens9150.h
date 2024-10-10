// It uses a 9150 compass/gyro/accelerometer combo
// Changelog:
//     2014-12-12 - Initial version by Jonas
// 	   2024-09-02 - robotstuck detection added by Willie
/* ============================================
Placed under the GNU license

===============================================
*/

#ifndef _MS9150_H_
#define _MS9150_H_
#ifndef __averaging__
int numReadings = 5
#else
int numReadings = 1


#include "MotionSensor.h"
#include <Wire.h>  // For Compass
#include <I2Cdev.h>
#include <Sens9150.h>
#include "Definition.h"

class MS9150 : public MOTIONSENSOR{
    public:
        virtual boolean initialize();
        virtual void updateHeading();
        virtual int getHeading();
        virtual void setNewTargetHeading();
        virtual int headingVsTarget();
        virtual int getTiltAngle();
        
    private:
    	MPU9150 sensor;
};


class RobotStuckDetector {
  public:
    RobotStuckDetector(int stuckThreshold = 1000, int timeout = 2000, int numReadings = 5);
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
#endif /* _Sens9150_H_ */
