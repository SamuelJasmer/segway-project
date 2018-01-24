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
	PID(int sensor_pin);
	void set_k_values(float kp, float ki, float kd);
	float measure_sensor();
	int calculate_error(int current_point, int set_point);
	float p_loop();
	float i_loop();
	float d_loop();
	float calculate_pid(int set_point);
	float get_delta_t();
	int get_error();
	float pid_derivative();
	
	float kp;
	float ki;
	float kd;
	int sensor_pin;

	float p = 0;
	float i = 0;
	float d = 0;
	float pid = 0;

	float output_initial = 0;
	float output_final = 0;

	float cp = 0;

	int error = 0;
	

private:
	float delta_t = 0;
	float output_delta_t = 1;
	
	int error_final = 0;
	float error_integral = 0;
	float convert_sensor_in_to_degrees(int raw);

};
