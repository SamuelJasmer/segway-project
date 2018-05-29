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
		vector integrate_vector();

		float calculate_Moment_of_Inertia()
		float integrate(float u);
		float get_delta_t();
		void initialize();
		float low_pass_filter(float initial_data, float final_data, float beta);


	private:
		float delta_t = 0;
		
		
};