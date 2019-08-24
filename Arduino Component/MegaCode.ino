/*
  Name:    MegaCode.ino
  Created: 1/9/2019 7:56:03 PM
  Author:  Anthony
*/
#include <Servo.h>
#include <HalfStepper.h>
#include <HX711/HX711.h>

//TEMPERATURE SENSOR---------------------------------------------------
const int outputPin = A2;
int rawVoltage = 0;
double voltage = 0;
double temperature = 0;

//LOAD CELL------------------------------------------------------------
const int DOUT = 45, CLK = 46;
HX711 scale(DOUT, CLK);
double calibrationFactor = -1500;
double mass = 0;

//STEPPER MOTORS-------------------------------------------------------
const int STEPS = 4096;
const int IN1_upper = 10, IN2_upper = 11, IN3_upper = 12, IN4_upper = 13;
const int IN1_fore = 4, IN2_fore = 5, IN3_fore = 6, IN4_fore = 7;
HalfStepper upperarm(STEPS / 2, IN1_upper, IN3_upper, IN2_upper, IN4_upper);
HalfStepper forearm(STEPS / 2, IN1_fore, IN3_fore, IN2_fore, IN4_fore);
int upperHalfSteps;
int foreHalfSteps; 

//SERVO MOTORS---------------------------------------------------------
Servo baseMotor;
Servo endEffector;
Servo camera;
Servo petri;

int getBaseMotorAngle(int theta) {
	return theta * -1.757107804 + 120.7174885;
}
int getUpperMotorHalfSteps(int theta) {
	return (theta - 67.75506745) / -1.60930283;
}
int getForeMotorHalfSteps(int theta) {
	return (theta + 0.4884012776) / 5.147229497;
}

void motorCommand(String data) {
	if (data.charAt(0) == 'D') {
		Serial.print("T");
		Serial.print(temperature);
		Serial.print("M");
		Serial.print(mass);
		delay(1500);
		Serial.println();
	}
	else if (data.charAt(0) == 'R') {
		camera.write(50);
		baseMotor.write(90);
		petri.write(140);
		delay(1000);
		petri.write(0);

	}
	else {
		int B, F, U;
		B = data.indexOf('B');
		F = data.indexOf('F');
		U = data.indexOf('U');
		int baseAngle1 = getBaseMotorAngle(data.substring(B + 1, F).toInt()), baseAngle0 = baseMotor.read();
		if (baseAngle0 < baseAngle1) {
			for (int i = baseAngle0; i <= baseAngle1; i++) {
				baseMotor.write(i);
				delay(50);
			}
		}
		else {
			for (int i = baseAngle0; i >= baseAngle1; i--) {
				baseMotor.write(i);
				delay(50);
			}
		}
		
		baseMotor.write(getBaseMotorAngle(data.substring(B + 1, F).toInt()));
		foreHalfSteps = getForeMotorHalfSteps(data.substring(F + 1, U).toInt());
		upperHalfSteps = getUpperMotorHalfSteps(data.substring(U + 1).toInt());
		for (int i = 0; i < foreHalfSteps; i++) {
			forearm.step(-STEPS / 2);
		}
		for (int i = 0; i < upperHalfSteps; i++) {
			upperarm.step(STEPS / 2);
		}
		for (int i = 0; i <= 150; i++) {
			endEffector.write(i);
			delay(50);
		}
		//RETURN MOTION
		for (int i = 0; i < upperHalfSteps; i++) {
			upperarm.step(-STEPS / 2);
		}
		for (int i = 0; i < foreHalfSteps-4; i++) {
			forearm.step(STEPS / 2);
		}
		if (baseAngle1 < getBaseMotorAngle(-10)) {
			for (int i = baseAngle1; i <= getBaseMotorAngle(-10); i++) {
				baseMotor.write(i);
				delay(50);
			}
		}
		else {
			for (int i = baseAngle1; i >= getBaseMotorAngle(-10); i--) {
				baseMotor.write(i);
				delay(50);
			}
		}
		for (int i = 150; i >= 0; i--) {
			endEffector.write(i);
			delay(10);
		}
		camera.write(25);
		for (int i = 0; i < 4; i++) {
			forearm.step(STEPS / 2);
		}
		baseMotor.write(90);
		
	}
}


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	//LOAD CELL
	scale.tare();
	scale.set_scale(calibrationFactor);


	//STEPPER MOTORS
	upperarm.setSpeed(4);
	forearm.setSpeed(4);

	//SERVO MOTORS
	baseMotor.attach(9);
	baseMotor.write(90);
	endEffector.attach(8);
	endEffector.write(0);
	camera.attach(2);
	camera.write(50);
	petri.attach(3);
	petri.write(0);

}

// the loop function runs over and over again until power down or reset
void loop() {
	if (Serial.available()) {
		motorCommand(Serial.readString());
	}

	//TEMPERATURE SENSOR
	rawVoltage = analogRead(outputPin);
	voltage = rawVoltage / 205.0;
	temperature = 100.0 * voltage - 50;
	delay(2000);
	//-------------------

	//LOAD CELL
	mass = fabs(scale.get_units());



}