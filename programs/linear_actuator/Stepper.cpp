//
//
//

#include "Stepper.h"
#include "L298N.h"

#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5

L298N CoilA(ENA, IN_A1, IN_A2);
L298N CoilB(ENB, IN_B1, IN_B2);

Stepper::Stepper() {
	CoilA.stop();
	CoilB.stop();
}

void Stepper::full_step(bool direction) {
	CoilA.set_speed(128);
	CoilB.set_speed(128);

	if (direction == 1) {
		CoilA.forward();
		CoilB.stop();

		delay(5);

		CoilA.stop();
		CoilB.forward();

		delay(5);

		CoilA.reverse();
		CoilB.stop();

		delay(5);

		CoilA.stop();
		CoilB.reverse();

		delay(5);

	}
	else {

		CoilA.forward();
		CoilB.stop();

		delay(5);

		CoilA.stop();
		CoilB.forward();

		delay(5);

		CoilA.reverse();
		CoilB.stop();

		delay(5);

		CoilA.stop();
		CoilB.reverse();

		delay(5);
	
	}

}


