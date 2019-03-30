/*
 * orientation_library.h
 */

#ifndef _ORIENTATION_LIBRARY_h
#define _ORIENTATION_LIBRARY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
	#include "matrix.h"
	#include "ringbuffer.h"
	#include "vector3.h"

class Orientation {

	public:
		Orientation();
		vector measure_gyro();
		vector measure_accelerometer();
		vector measure_magnetometer();
		vector angular_acceleration();
		vector integrate_vector(vector u);
		vector smooth_gyro(vector gyro_vector);

		void init(bool enable_gyro, bool enable_accelmag);
		float calculate_Moment_of_Inertia();
		float integrate(float u);
		float moving_average_3n(float data_in[3]);

		vector gyro_vector;
		vector angular_velocity_initial;
		vector angular_velocity_final;

		float delta_t = 0;
	private:
};

class filter {

	public:
		void init(int n);
		vector sample_mean(vector input, int n);
		vector sample_variance(vector sample_mean, vector sample, int n);
		vector covariance(vector sample_mean, vector sample, int n);
		vector standard_deviation(vector sample_variance);

		vector least_squares_regression(vector input, int n);
		Matrix create_weight_Matrix(float current_variance, int n);
		Matrix create_error_Matrix(float current_variance, int n);
		Matrix weighted_least_squares_regression(float input, float variance, int n);
		vector lowess_smooth(vector input, int n);


	private:
		vector average;

		int average_n;

		RingBuffer* mean_buffer_x;
		RingBuffer* mean_buffer_y;
		RingBuffer* mean_buffer_z;

		RingBuffer* variance_buffer_x;
		RingBuffer* variance_buffer_y;
		RingBuffer* variance_buffer_z;

		RingBuffer* covariance_buffer_x;
		RingBuffer* covariance_buffer_y;
		RingBuffer* covariance_buffer_z;

		RingBuffer* error_buffer;
		RingBuffer* weight_buffer;
		RingBuffer* Y_buffer;
};

#endif
