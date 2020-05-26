/*
    Name:       linear_actuator.ino
    Created:	10/4/2019 11:55:23 AM
    Author:     AD\jasmersr
*/
#include "Stepper.h"

Stepper StepperA;

void setup()
{
	Serial.begin(115200);
	pinMode(12, INPUT);
	pinMode(13, INPUT);
}

void loop()
{
	//StepperA.full_step(1);
	bool Endstop1_state = digitalRead(12);
	bool Endstop2_state = digitalRead(13);

	Serial.print(Endstop1_state);
	Serial.print(", ");
	Serial.println(Endstop2_state);
}
