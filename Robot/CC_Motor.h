/*
 * CC_Motor.h
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */
// CC_Motor.h

#ifndef _CC_MOTOR_h
#define _CC_MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class CC_MotorClass
{
 private:
	 uint8_t ApinPWM;
	 uint8_t ApinSense1;
	 uint8_t ApinSense2;

 public:

	typedef enum {RIGHT, LEFT} T_MOTOR_SENSE;

	void init(int pinpwm, int pinsense1, int pinsense2);
	void control_speed(int speedset);
	void control_sense(T_MOTOR_SENSE sense);
};

// extern CC_MotorClass CC_Motor;

#endif
