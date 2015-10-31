/*
 * Arduino_Mapping.h
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */

// Arduino_Mapping.h

#ifndef _ARDUINO_MAPPING_h
#define _ARDUINO_MAPPING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// mapping from Bluno_scematic.pdf

// Discrete I/O D0-D13
#define PD0_RXD 0PD1_TXD 1
#define PD2_INT0 2
#define PD3_INT1_PWM1 3
#define PD4_XCK_T0 4
#define PD5_T1_PWM2 5
#define PD6_AIN0_PWM3 6
#define PD7_AIN1 7
#define PB0_ICP 8
#define PB1_OC1A_PWM4 9
#define PB2_SS_OC1B_PWM5 10
#define PB3_MOSI_OC2_PWM6 11
#define PB4_MISO 12
#define PB5_SCK 13

// Analog I/O
#define ADC0_PC0 A0
#define ADC1_PC1 A1
#define ADC2_PC2 A2
#define ADC3_PC3 A3
#define ADC4_SDA_PC4 A4
#define ADC5_SCL_PC5 A5

// Mapping CC-Motor STEER
#define STEER_MOTOR_SENSE_1 PD7_AIN1
#define STEER_MOTOR_SENSE_2 PB0_ICP
#define STEER_MOTOR_PWM PD5_T1_PWM2

// Mapping CC-Motor DRIVE
#define DRIVE_MOTOR_SENSE_1 PD2_INT0
#define DRIVE_MOTOR_SENSE_2 PD4_XCK_T0
#define DRIVE_MOTOR_PWM PD3_INT1_PWM1


#endif
