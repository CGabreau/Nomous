/*
 * CC_Motor.cpp
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */

//
//
//

#include "CC_Motor.h"

void CC_MotorClass::init(int pinpwm, int pinsense1, int pinsense2)
{
	ApinPWM = pinpwm;
	ApinSense1 = pinsense1;
	ApinSense2 = pinsense2;

	// CC motor init
	pinMode(ApinSense1,OUTPUT);
	pinMode(ApinSense2, OUTPUT);
	pinMode(ApinPWM, OUTPUT);
}

void CC_MotorClass::control_speed(int speedset)
{
	analogWrite(ApinPWM, speedset);
}

void CC_MotorClass::control_sense(T_MOTOR_SENSE sense)
{
	if (sense == RIGHT)
	{
		digitalWrite(ApinSense1, LOW);
		digitalWrite(ApinSense2, HIGH);
	}
	else
	{
		digitalWrite(ApinSense1, HIGH);
		digitalWrite(ApinSense2, LOW);
	}
}

CC_MotorClass CC_Motor;



