//Stepper.h
//
//

#ifndef _Stepper_h
#define _Stepper_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



class Stepper {

public:
	Stepper();
	void full_step(bool direction);
		
private:

};

#endif