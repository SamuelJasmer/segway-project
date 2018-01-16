// PID.h
//
//

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

class PID {

public:
	PID(float kp, float ki, float kd, int sensor_pin);
	float measure_sensor();
	int calculate_error(int current_point, int set_point);
	float p_loop(int error);
	float i_loop(int error, int delta_t);
	float d_loop(int error, int delta_t);
	float calculate_pid();
	float get_delta_t();
	int get_error();

	float kp;
	float ki;
	float kd;
	int sensor_pin;

private:
	float delta_t = 0;
	int error = 0;
	int error_final = 0;

};
