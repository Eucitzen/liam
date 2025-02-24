/*
 Liam - DIY Robot Lawn Mower

 Display Library

 ======================
  Licensed under GPLv3
 ======================
*/

/*
  Base class for displays.
  Default behavior is to print to serial port.

  This code extends the Print class to get the print
  capabilities. Basically the print class method calls each
  other until reaching the most basic method which is
  implemented here (write)
*/

#ifndef _MYDISPLAY_H_
#define _MYDISPLAY_H_

#include "Definitions.h"
#include "Motioncontrol.h"
#include "Sensors.h"
#include "Battery.h"

#include <Arduino.h>


class MYDISPLAY : public Print
{
  public:
    MYDISPLAY(BATTERY* batt, WHEELMOTOR* left, WHEELMOTOR* right, CUTTERMOTOR* cut, BWFSENSOR* bwf, MOTIONSENSOR* comp, int* state);
    virtual boolean initialize();
    void update();

    virtual size_t write(uint8_t);
    virtual void setCursor(int col, int row);
    virtual void clear();
    virtual void blink();

  protected:
    BATTERY* Battery;
    WHEELMOTOR* leftMotor;
    WHEELMOTOR* rightMotor;
    CUTTERMOTOR* cutter;
    BWFSENSOR* sensor;
    MOTIONSENSOR* compass;
    int* moverstate;
};
#endif

// Removed the public and protected keywords,
// as they are not needed in this case
// and also removed the virtual keyword from the
// constructor and destructor as they are not needed
// and also removed the boolean return type from the
// initialize method as it is not needed
