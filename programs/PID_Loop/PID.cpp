// 
// 
// 

#include "PID.h"

PID::PID(float kp, float ki, float kd, int sensor_pin) {

	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->delta_t = 0;
	this->error = 0;
	this->error_final = 0;
}

float PID::measure_sensor() {

	int t_initial = micros();
	int sensor = analogRead(sensor_pin);
	int t_final = micros();

	delta_t = (t_final - t_initial) / 1000.0;

	return sensor;
}

int PID::calculate_error(int current_point, int set_point) {

	error = current_point - set_point;

	return error;
}

float PID::p_loop(int error) {

	int proportion;
	proportion = kp*error;

	return proportion;
}

float PID::i_loop(int error, int delta_t) {
	float error_integral;
	error_integral += ki*error*(delta_t);

	return error_integral;
}

float PID::d_loop(int error, int delta_t) {
	float error_initial = error;
	float error_derivative = kd*((this->error_final - error_initial) / delta_t);
	this->error_final = error_initial;

	return error_derivative;

}

float PID::calculate_pid() {
	
	float p = p_loop(get_error());
	float i = i_loop(get_error(), get_delta_t());
	float d = d_loop(get_error(), get_delta_t());

	float pid = p + i + d;
}

float PID::get_delta_t() {

	return this->delta_t;
}

int PID::get_error() {

	return this->error;
}
