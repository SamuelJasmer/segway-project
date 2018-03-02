// 
// 
// 

#include "PID.h"

PID::PID(int sensor_pin) {

	this->sensor_pin = sensor_pin;
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

float PID::measure_sensor() {

	int t_initial = micros();
	float sensor = analogRead(sensor_pin);
	int t_final = micros();

	this->delta_t = (t_final - t_initial) / 1000.0;

	this->cp = convert_sensor_to_degrees(sensor);

	return this->cp;
}

float PID::convert_sensor_to_degrees(int raw) {

	float cp = map(raw, 57, 1023, -57, 237);

	return cp;
}

int PID::calculate_error(int current_point, int set_point) {

	this->error = abs(current_point - set_point);

	return this->error;
}

float PID::p_loop() {

	float proportion;
	proportion = this->kp*this->error;

	return proportion;
}

float PID::i_loop() {
	
	this->error_integral += this->ki*this->error*this->delta_t;

	if (this->error = 0) {
		error_integral = 0;
	}
	else {
		error_integral = error_integral;
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

float PID::calculate_pid(int set_point) {

	int current_point = measure_sensor();
	this->error = PID::calculate_error(current_point, set_point);

	this->p		= PID::p_loop();
	this->i		= PID::i_loop();
	this->d		= PID::d_loop();
	this->pid	= this->p + this->i + this->d;

	float t_initial = micros();
	this->output_final = this->output_initial + this->pid;
	float t_final = micros();
	this->output_delta_t = t_final - t_initial;

	this->output_initial = this->output_final;
	
	float delta_pid = PID::pid_derivative();

	//conversions to velocity units

	return this->pid;

}

float PID::get_delta_t() {

	return this->delta_t;
}

int PID::get_error() {

	return this->error;
}

float PID::pid_derivative() {
	
	float pid_velocity = (this->output_final - this->output_initial);

	return pid_velocity;
}
