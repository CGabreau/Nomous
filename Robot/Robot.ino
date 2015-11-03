
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT

// uncomment "BLUETOOTH_MODE" if you want to pilot the motors
// with the bluetooth remote controller
//#define BLUETOOTH_MODE

#include "Arduino_Mapping.h"
#include "CC_Motor.h"
#include "Servo_Actuator.h"
#include <Servo.h>
#include "utilities.h"
// library: scheduler (multi-tasking)
#include "SchedulerARMAVR.h"

//using namespace std;

#define STEER_DELAY     20
#define DRIVE_DELAY     20

int val_steer = 0;         // steer servo value
int val_drive = 0;         // drive servo value

// tourelle servo definition
Servo_ActuatorClass TourelleYaw(6,820,2100,90);
Servo_ActuatorClass TourellePitch(9, 820, 2100, 90);
int tourelle_yaw_true_position;
int tourelle_pitch_true_position;
int val_yaw = 0;           // yaw servo value
int val_pitch = 0;         // pitch servo value

//CC motor control definition
CC_MotorClass MotorSteer;
CC_MotorClass MotorDrive;

signed char BleBuffer[2];        // command buffer from BLE
int idx_buffer;                  // index of received data from BLE


// MPU6050 definition *********************************************************
// ****************************************************************************
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire\Wire.h>
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
digital I/O pin 2.
* ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

// end MPU6050 definition ******************************************************


void setup()
{
	// start serial communication to display
	Serial.begin(115200);


	// initialize the tourelle engines
	TourelleYaw.init();
	TourellePitch.init();
	//MotorSteer.init(STEER_MOTOR_SENSE_1, STEER_MOTOR_SENSE_2, STEER_MOTOR_PWM);
	//MotorDrive.init(DRIVE_MOTOR_SENSE_1, DRIVE_MOTOR_SENSE_2, DRIVE_MOTOR_PWM);

	// configure the steering and driving threads
	Scheduler.startLoop(loopSteer,128);
	Scheduler.startLoop(loopDrive,128);


	// MPU6050 SETUP **************************************************************
	// ****************************************************************************
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	// with the results of the MPU6050 calibration
	mpu.setXAccelOffset(-539);
	mpu.setYAccelOffset(933);
	mpu.setZAccelOffset(981);
	mpu.setXGyroOffset(59);
	mpu.setYGyroOffset(-72);
	mpu.setZGyroOffset(-2);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	// end MPU6050 SETUP **************************************************************


}

void loop()
{

	// MP6850

	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .

		// receive input from BLE (we expect three bytes: -128, x, y values)

#ifdef BLUETOOTH_MODE

		if (Serial.available())
		{
			signed char c = (signed char)Serial.read();

			// when we find out sentinal; reset the index
			if (c == -128) idx_buffer = 0;
			else  BleBuffer[idx_buffer++] = c;

		}
		else

			// if we somehow have lost connection - revert to center positions
		{
			//val_steer = 0;
			//val_drive = 0;

			//Serial.print(" lost ");
		}

		// have we received enough data?
		if (idx_buffer == 2)
		{
			// read input for steer (-127 .. 127)
			signed char cs = BleBuffer[0];
			val_yaw = cs * 90 / 127;
			val_steer = cs * 2;

			// read input for drive (-127 .. 127)
			signed char cd = BleBuffer[1];
			val_pitch = cd * 90 / 127;
			val_drive = cd * 2;

			// reset our index
			idx_buffer = 0;
			//Serial.print(cs); Serial.print(" raw "); Serial.print(cd); Serial.print(" ");
			//Serial.print(val_steer); Serial.print(" val "); Serial.print(val_drive);
		}


		//tourelle position feedback
		//tourelle_yaw_true_position = TourelleYaw.get_servo_position();
		//tourelle_pitch_true_position = TourellePitch.get_servo_position();

		//Serial.print("YAW "); Serial.print(tourelle_yaw_true_position, DEC);
		//Serial.print(" - PITCH "); Serial.print(tourelle_pitch_true_position, DEC);
#endif

	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		val_yaw = ypr[0] * 180 / M_PI;
		val_pitch = ypr[2] * 180 / M_PI;

		TourelleYaw.position(-val_yaw);
		TourellePitch.position(-val_pitch);

#ifdef OUTPUT_TEAPOT
		// display quaternion values in InvenSense Teapot demo format:
		teapotPacket[2] = fifoBuffer[0];
		teapotPacket[3] = fifoBuffer[1];
		teapotPacket[4] = fifoBuffer[4];
		teapotPacket[5] = fifoBuffer[5];
		teapotPacket[6] = fifoBuffer[8];
		teapotPacket[7] = fifoBuffer[9];
		teapotPacket[8] = fifoBuffer[12];
		teapotPacket[9] = fifoBuffer[13];
		Serial.write(teapotPacket, 14);
		teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
	}

	// end of MPU6050


	//delay(10);
	// let other threads have time to do something
	yield();
}



void loopSteer()
{
	if (val_steer > 0)
	{
		MotorSteer.control_sense(MotorSteer.RIGHT);
		MotorSteer.control_speed(val_steer);
	}
	else
	{
		MotorSteer.control_sense(MotorSteer.LEFT);
		MotorSteer.control_speed(-val_steer);
	}
	// let other threads have time to do something (with minimum delay)
	Scheduler.delay(STEER_DELAY);
}

void loopDrive()
{
	if (val_drive > 0)
	{
		MotorDrive.control_sense(MotorDrive.RIGHT);
		MotorDrive.control_speed(val_drive);
	}
	else
	{
		MotorDrive.control_sense(MotorDrive.LEFT);
		MotorDrive.control_speed(-val_drive);
	}

	// let other threads have time to do something (with minimum delay)
	Scheduler.delay(DRIVE_DELAY);
}

