//
//
//

#include "L298N.h"

L298N::L298N(int ENABLE, int IN1, int IN2) {
	pinMode(ENABLE,		OUTPUT);
	pinMode(IN1,		OUTPUT);
	pinMode(IN2,		OUTPUT);

	this->ENABLE	= ENABLE;
	this->IN1		= IN1;
	this->IN2		= IN2;

}

void L298N::increment_up() {
	for (int i = 0; i < 255; i++) {
		analogWrite(this->ENABLE, i);
		delay(100);
	}
}

void L298N::increment_down() {
	for (int i = 255; i > 0; i--) {
		analogWrite(this->ENABLE, i);
		delay(100);
	}
}

void L298N::forward() {
	digitalWrite(this->IN1, HIGH);
	digitalWrite(this->IN2, LOW);
}

void L298N::reverse() {
	digitalWrite(this->IN1, HIGH);
	digitalWrite(this->IN2, LOW);
}

void L298N::stop() {
	digitalWrite(this->IN1, HIGH);
	digitalWrite(this->IN2, HIGH);
}

void L298N::set_speed(float speed) {

}