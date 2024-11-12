#include <Arduino.h>
#include "Sensors.h"
#include "Definition.h"


#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

class CONTROLLER {
public:
  CONTROLLER(WHEELMOTOR& left, WHEELMOTOR& right, WHEELMOTOR& front, CUTTERMOTOR& cut, BWFSENSOR& bwf, MOTIONSENSOR& comp):
  leftMotor(left), 
  rightMotor(right), 
  frontMotor(front),
  cutter(cut), 
  sensor(bwf), 
  compass(comp),
  {}
  int turn(int degrees);
  int turnToReleaseLeft(int degrees);
  int turnToReleaseRight(int degrees);
  int waitWhileChecking(int duration);
  int waitWhileInside(int duration);

  void runForward(int speed);
  void runBackward(int speed);
  void stop();

  boolean allSensorsAreOutside();

  void startCutter();
  void stopCutter();

  void setDefaultDirectionForward(bool fwd);

  void adjustMotorSpeeds();
  int compensateSpeedToCutterLoad();
  int compensateSpeedToCompassHeading();

  boolean wheelsAreOverloaded();
  boolean cutterIsOverloaded();

  boolean hasBumped();
  boolean hasTilted();
  boolean hasFlipped();
  boolean isLifted();

  void resetBalance();
  int getBalance();

  void updateBalance();
  void storeState();
  void restoreState();

  int turnLeft(int degrees);
  int turnRight(int degrees);
  void avoidObstacles(int trigPinLeft, int echoPinLeft, int trigPinRight, int echoPinRight, int trigPinFront, int echoPinFront);
  void avoidObstacles();

private:
  WHEELMOTOR& leftMotor;
  WHEELMOTOR& rightMotor;
  WHEELMOTOR& frontMotor;
  CUTTERMOTOR& cutter;
  BWFSENSOR& sensor;
  MOTIONSENSOR& compass;

  // Bug checks
  boolean isNull(WHEELMOTOR* motor);
  boolean isNull(CUTTERMOTOR* motor);
  boolean isNull(BWFSENSOR* sensor);
  boolean isNull(MOTIONSENSOR* sensor);
  boolean hasNullPointers();
  }
  #endif /* _CONTROLLER_H_ */

class Wheelmotor {
public:
Wheelmotor(int pwmPin, int dirPin, int loadPin, int smoothness):
pwmPin_(pwmPin), 
dirPin_(dirPin), 
loadPin_(loadPin), 
smoothness_delay(smoothness)
{}
  void setSpeed(int setspeed);
  int getSpeed();
  int getLoad();
  bool isOverloaded();
  void setOverloadLevel(int level);
  void setSmoothness(int delay);

private:
  int pwmPin_;
  int dirPin_;
  int loadPin_;
  int load_;
  int speed_;
  int overloadLevel_;
  int smoothness_delay;
}