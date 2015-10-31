/*
 * Servo_Actuator.cpp
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */
//
//
//
#include "Servo_Actuator.h"


Servo_ActuatorClass::Servo_ActuatorClass(int pinout,int minpwm,int maxpwm,int startservoposition)
{
	APinout = pinout; //Arduino input port mapping
	AMinPwm = minpwm; AMaxPwm = maxpwm; // Servo min and max PWM commands results of the calibration
	AStartServoPosition=startservoposition; // Servo position when the engine starts
	AServoPosition = 0; // init of the output position of the servo
}

void Servo_ActuatorClass::init()
{
	// PWM servo on pin out Apinout
	Servo::attach(APinout,AMinPwm,AMaxPwm);
	// Send the default command
	Servo::write(AStartServoPosition);
}

void Servo_ActuatorClass::calibrate()
{


}

void Servo_ActuatorClass::position(int angle)
{
	// AStartServoPosition is the position 0
	// The servo position is computed from this position
	AServoPosition = AStartServoPosition-angle;
	Servo::write(AServoPosition);
}

int Servo_ActuatorClass::get_servo_position() const
{
	return AServoPosition;
}



