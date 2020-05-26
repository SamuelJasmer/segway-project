//L298N.h
//
//

#ifndef _L298N_h
#define _L298N_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif




class L298N {

public:
	L298N(int ENABLE, int CC, int CCW);
	void increment_up();
	void increment_down();
	void forward();
	void reverse();
	void stop();
	void set_speed(float speed);

private:
	int ENABLE;//PWM pin
	int IN1;//Direction pin 1
	int IN2;//Direction pin 2
};

#endif
