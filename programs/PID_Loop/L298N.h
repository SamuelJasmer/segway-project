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


#endif

class L298N {

public:
	L298N();
	void increment_up(int enable);
	void increment_down(int enable);
	void forward(int input1, int input2);
	void reverse(int input1, int input2);
	void stop(int input1, int input2);

	int pin_out[6] = {};
private:


};