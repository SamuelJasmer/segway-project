// 
// 
// 

#include "PID.h"

PID::PID() {

	this->delta_t = 0;
	this->error = 0;
	this->error_final = 0;
	this->error_integral = 0;
}

void PID::set_k_values(float kp, float ki, float kd) {

	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

//float PID::measure_sensor() {
//
//	int t_initial = micros();
//	float sensor = analogRead(sensor_pin);
//	int t_final = micros();
//
//	this->delta_t = (t_final - t_initial) / 1000.0;
//
//	this->cp = convert_sensor_to_degrees(sensor);
//
//	return this->cp;
//}

//float PID::convert_sensor_to_degrees(int raw) {
//
//	float current_point = map(raw, 57, 1023, -57, 237);
//
//	return current_point;
//}

float PID::calculate_error(float current_point, int set_point) {
	this->set_point = set_point;

	this->error = abs(current_point - this->set_point);

	return this->error;
}

float PID::p_loop() {

	float proportion;
	proportion = this->kp*this->error;

	return proportion;
}

float PID::i_loop() {

	if (this->current_point > this->set_point) {
		this->error_integral -= this->ki*this->error*this->delta_t;
	}
	else if (this->current_point < this->set_point) {
		this->error_integral += this->ki*this->error*this->delta_t;
	}
	else {
		error_integral = 0;
	}
	//this->error_integral = this->ki*this->error_integral;

	/*
	if(this->error_integral > 256) {

		this->error_integral = 256;
	}
	else {
		this->error_integral = this->error_integral;
	}
	*/
	

	return this->error_integral;
}

float PID::d_loop() {

	float error_initial = this->error;
	float error_derivative = this->kd*((this->error_final - error_initial) / this->delta_t);
	this->error_final = error_initial;

	return error_derivative;
}

float PID::calculate_pid(float current_point, int set_point) {

	this->error = PID::calculate_error(current_point, set_point);

	this->p		= PID::p_loop();
	this->i		= PID::i_loop();
	this->d		= PID::d_loop();
	this->pid	= this->p + this->i;

	return this->pid;
}

float PID::get_delta_t() {

	return this->delta_t;
}

int PID::get_error() {

	return this->error;
}
