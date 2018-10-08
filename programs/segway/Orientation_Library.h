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
	#include "ArduinoSTL.h"
	#include <deque>



struct vector {

	public:
		float x;
		float y;
		float z;
		float magnitude();

		void set_vector(float vector_array[]);
		void convert_to_degrees();
		void convert_to_radians();

		vector cross_product(vector v);

		float dot_product(vector v);
		float angle_between_vectors(vector v);
};

class Orientation {

	public:
		Orientation();
		vector measure_gyro();
		vector measure_accelerometer();
		vector measure_magnetometer();
		vector angular_acceleration();
		vector integrate_vector(vector u);
		vector smooth_gyro(vector gyro_vector);

		void init();
		float calculate_Moment_of_Inertia();
		float integrate(float u);
		float get_delta_t();
		float moving_average_3n(float data_in[3]);

		vector gyro_vector;
		vector angular_velocity_initial;
		vector angular_velocity_final;

	private:
		float delta_t = 0;	

		float gyro_buffer_x[3];
		float gyro_buffer_y[3];
		float gyro_buffer_z[3];
};

class filter {

	public:
		void init(int n);
		vector sample_mean(vector input, int n);
		vector sample_variance(vector sample_mean, vector current_point, int n);
		vector covariance(vector sample_mean, vector current_point, int n);
		vector standard_deviation(vector sample_variance);

		vector CLT(vector variance1, vector variance2, vector sensor1, vector sensor2);
		vector least_squares_regression(vector input, int n);
		Matrix create_weight_Matrix(float current_variance, int n);
		Matrix create_error_Matrix(float current_variance, int n);
		Matrix weighted_least_squares_regression(float input, float variance, int n);
		vector lowess_smooth(vector input, int n);


	private: 
		float Y_buffer[];
		vector buffer[];
		vector mean_buffer[];
		float variance_buffer[];
		vector covariance_buffer[];
		vector average;
		std::deque<float> buffer_x;
		std::deque<float> buffer_y;
		std::deque<float> buffer_z;

		std::deque<float> mean_buffer_x;
		std::deque<float> mean_buffer_y;
		std::deque<float> mean_buffer_z;
};

#endif