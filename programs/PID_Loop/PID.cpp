// 
// 
// 

#include "PID.h"

PID::PID(float kp, float ki, float kd, int sensor_pin) {

	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->sensor_pin = sensor_pin;
	this->delta_t = 0;
	this->error = 0;
	this->error_final = 0;
	this->error_integral = 0;
}

float PID::measure_sensor() {

	int t_initial = micros();
	float sensor = analogRead(sensor_pin);
	int t_final = micros();

	this->delta_t = (t_final - t_initial) / 1000.0;

	this->cp = convert_sensor_in_to_degrees(sensor);

	return this->cp;
}

float PID::convert_sensor_in_to_degrees(int raw) {

	float cp = map(raw, 57, 1023, -57, 237);

	return cp;
}

int PID::calculate_error(int current_point, int set_point) {

	this->error = current_point - set_point;

	return this->error;
}

float PID::p_loop() {

	float proportion;
	proportion = this->kp*this->error;

	return proportion;
}

float PID::i_loop() {
	
	this->error_integral += this->ki*this->error*this->delta_t;

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

	int t_initial = micros();
	this->output_final = this->output_initial + this->pid;
	int t_final = micros();
	this->output_delta_t = t_final - t_initial;

	this->output_initial = output_final;
	
	return this->output_initial;

}

float PID::get_delta_t() {

	return this->delta_t;
}

int PID::get_error() {

	return this->error;
}

float PID::pid_derivative() {
	
	float pid_velocity = (this->output_final - this->output_initial) / this->output_delta_t;

	return pid_velocity;
}
