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


#endif

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

		void initialize();
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