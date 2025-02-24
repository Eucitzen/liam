/// This is the library for the defaults
//
// Changelog:

//     2014-12-12 - Initial version by Jonas
//     2024-10-13 - Added ultrasonic support by Willie
//     2025-02-12 correcting pins and expanding pin definitions for larger chip

/* ============================================
Placed under the GNU license

===============================================
*/

#include "Definitions.h"
#include "Motioncontrol.h"




/** Define all pins */
void DEFINITION::definePinsInputOutput() {
/* These are no longer in use due to built-in current protection*/
  //pinMode(WHEEL_MOTOR_A_CURRENT_PIN, INPUT);
  //pinMode(WHEEL_MOTOR_B_CURRENT_PIN, INPUT);
  //pinMode(CUTTER_CURRENT_PIN, INPUT);

  pinMode(SOC_PIN, INPUT); //State of charge for reading battery
  

  /*
    Some pins are better leave undefined as default
	pinMode(I2C_SDA_PIN, INPUT);
	pinMode(I2C_SDL_PIN, OUTPUT);

	pinMode(RX_PIN, INPUT);	
	pinMode(TX_PIN, OUTPUT);
  */
  pinMode(BWF_SENSOR_INPUT_PIN, INPUT);
  pinMode(WHEEL_MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(BWF_SELECT_A_PIN, OUTPUT);
  pinMode(DOCK_PIN, INPUT);
  pinMode(CUTTER_PWM_PIN, OUTPUT);
  pinMode(BWF_SELECT_B_PIN, OUTPUT);

#if defined __Bumper__
  pinMode(BUMPERfront, INPUT);
  pinMode(BUMPERleft, INPUT);
  pinMode(BUMPERright, INPUT);
  digitalWrite(BUMPERfront, HIGH);
  digitalWrite(BUMPERleft, HIGH);
  digitalWrite(BUMPERright, HIGH);
#else
  pinMode(BUMPERfront, OUTPUT);
  pinMode(BUMPERleft, OUTPUT);
  pinMode(BUMPERright, OUTPUT);
  digitalWrite(BUMPERfront, LOW);
  digitalWrite(BUMPERleft, LOW);
  digitalWrite(BUMPERright, LOW);
#endif

#if defined __SR04__
  pinMode(_trigPinLeft, OUTPUT);
  pinMode(_echoPinLeft, INPUT);
  pinMode(_trigPinRight, OUTPUT);
  pinMode(_echoPinRight, INPUT);
  pinMode(_trigPinFront, OUTPUT);
  pinMode(_echoPinFront, INPUT);
#endif

#if defined __Lift_Sensor__
  pinMode(LIFT_SENSOR_PIN, INPUT);
  digitalWrite(LIFT_SENSOR_PIN, HIGH);
#else
  pinMode(LIFT_SENSOR_PIN, OUTPUT);
  digitalWrite(LIFT_SENSOR_PIN, LOW);
#endif
  pinMode(LED_PIN, OUTPUT);
  pinMode(WHEEL_MOTOR_B_PWM_PIN, OUTPUT);
  pinMode(WHEEL_MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(WHEEL_MOTOR_B_DIRECTION_PIN, OUTPUT);
}

/*void DEFINITION::setDefaultLevels(BATTERY& battery, WHEELMOTOR& left, WHEELMOTOR& front, WHEELMOTOR& right, CUTTERMOTOR& cutter) {
  left->setOverloadLevel(WHEELMOTOR_OVERLOAD);
  right->setOverloadLevel(WHEELMOTOR_OVERLOAD);
  cutter->setOverloadLevel(CUTTER_OVERLOAD);
}*/


