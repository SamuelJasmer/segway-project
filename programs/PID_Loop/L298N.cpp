//
//
//

#include "L298N.h"

#define ENA		 pin_out[0]
#define IN_A1	 pin_out[1]
#define IN_A2	 pin_out[2]
#define IN_B2	 pin_out[3]
#define IN_B1	 pin_out[4]
#define ENB		 pin_out[5]

L298N::L298N() {
	pinMode(ENA,	OUTPUT);
	pinMode(IN_A1,	OUTPUT);
	pinMode(IN_A2,	OUTPUT);
	pinMode(IN_B2,	OUTPUT);
	pinMode(IN_B1,	OUTPUT);
	pinMode(ENB,	OUTPUT);
}

void L298N::increment_up(int enable) {
	for (int i = 0; i < 255; i++) {
		analogWrite(enable, i);
		delay(100);
	}
}

void L298N::increment_down(int enable) {
	for (int i = 255; i > 0; i--) {
		analogWrite(enable, i);
		delay(100);
	}
}

void L298N::forward(int input1, int input2) {
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
}

void L298N::reverse(int input1, int input2) {
	digitalWrite(input2, HIGH);
	digitalWrite(input1, LOW);
}

void L298N::stop(int input1, int input2) {
	digitalWrite(input2, HIGH);
	digitalWrite(input1, HIGH);
}