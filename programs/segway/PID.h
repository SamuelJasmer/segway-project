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

#include "ringbuffer.h"

class PID {

public:
	PID();
	void set_k_values(float kp, float ki, float kd);
	void calculate_error(float current_point, float set_point);
	void p_loop();
	void i_loop();
	void d_loop();
	float calculate_pid(float current_point, float set_point, float delta_t);
	float integrate(float current_point, float delta);
	
	float kp;
	float ki;
	float kd;

	int set_point;

	float p = 0;
	float i = 0;
	float d = 0;
	float pid = 0;
	float sum = 0;

	float output_initial = 0;
	float output_final = 0;

	float current_point = 0;

	
	

private:
	float error = 0;
	float delta_t = 0;
	float dt = 0.1;//time step
	float output_delta_t = 1;

	float error_initial = 0;
	float error_final = 0;
	float error_integral = 0;

	RingBuffer* integral_buffer;
};
