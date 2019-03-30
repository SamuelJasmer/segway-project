// 
// 
// 

#include "PID.h"
#include "ringbuffer.h"

PID::PID() {

	this->delta_t = 0;
	this->error = 0;
	this->error_final = 0;
	this->error_integral = 0;

	this->integral_buffer = ringbuffer_new(2);
	ringbuffer_fill(this->integral_buffer, 0.0f);
}

/**
 * @brief  Sets the k constants for the PID loop
 * @note
 * @param  kp:proportional constant input, ki:integral constant input, kd:derivative constant input
 * @retval 
 */
void PID::set_k_values(float kp, float ki, float kd) {

	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

/**
 * @brief  Calculates the error between setpoint and current point
 * @note
 * @param  current_point:Current data point in time, set_point:"heading" for the PID loop
 * @retval
 */
float PID::calculate_error(float current_point, int set_point) {
	this->set_point = set_point;

	this->error = current_point - this->set_point;

	return this->error;
}

/**
 * @brief  Calculates the proportional response to the error
 * @note
 * @param
 * @retval
 */
float PID::p_loop() {

	float proportion;
	proportion = this->kp*this->error;

	return proportion;
}

/**
 * @brief  Calculates the integral response to the error
 * @note
 * @param
 * @retval
 */
float PID::i_loop() {

	/*
	if (this->current_point > this->set_point) {
		this->error_integral -= this->ki*this->error*this->delta_t;
	}
	else if (this->current_point < this->set_point) {
		this->error_integral += this->ki*this->error*this->delta_t;
	}
	else {
		error_integral = 0;
	}
	*/
	this->error_integral = this->ki*integrate(this->error, this->delta_t);

	return this->error_integral;
}

/**
 * @brief  Calculates an indefinite integral of the given input, starting at t=0 -> t=inf
 * @note   Uses Midpoint Rectangular Approximation
 * @param  current_point:Current data point in time, delta:Step size between data points, allowed to change
 * @retval 
 */
float PID::integrate(float current_point, float delta) {

	ringbuffer_dequeue(this->integral_buffer);
	ringbuffer_enqueue(this->integral_buffer, current_point);

	this->sum += 0.5*(ringbuffer_at(this->integral_buffer, 0) + ringbuffer_at(this->integral_buffer, 1)) * delta;

	return sum;
}

/**
 * @brief  Calculates the derivative response to the error
 * @note
 * @param
 * @retval
 */
float PID::d_loop() {

	this->error_initial = this->error;
	float error_derivative = this->kd * ((this->error_final - this->error_initial) / this->delta_t);
	this->error_final = this->error_initial;

	return error_derivative;
}

/**
 * @brief  Calculates the total PID response
 * @note
 * @param
 * @retval
 */
float PID::calculate_pid(float current_point, int set_point, float _delta_t) {

	this->delta_t = _delta_t;

	this->error = PID::calculate_error(current_point, set_point);

	this->p		= p_loop();
	this->i		= i_loop();
	this->d		= d_loop();
	this->pid	= this->p + this->i + this->d;

	return this->pid;
}