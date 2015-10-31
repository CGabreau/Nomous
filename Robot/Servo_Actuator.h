/*
 * Servo_Actuator.h
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */

// Servo_Actuator.h

#ifndef _SERVO_ACTUATOR_h
#define _SERVO_ACTUATOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"

#else
	#include "WProgram.h"
#endif

#include <Servo.h>

class Servo_ActuatorClass: public Servo
{
 public:
	// Methods
	Servo_ActuatorClass(int pinout, int minpwm, int maxpwm, int startservoposition);
	void init();
	void calibrate();
	void position(int angle);
	int get_servo_position() const;

 private:
	// Attributs
	int APinout;
	int AMinPwm,AMaxPwm;
	int AStartServoPosition;
	int AServoPosition;


};

//extern Servo_ActuatorClass Actuator;

#endif
