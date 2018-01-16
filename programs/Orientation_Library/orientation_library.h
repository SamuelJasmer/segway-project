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

		float dot_product(vector v);
		vector cross_product(vector v);
		float angle_between_vectors(vector v);

		void convert_to_degrees();
		void convert_to_radians();
};

class Orientation {

	public:
		Orientation();
		vector measure_gyro();
		vector measure_accelerometer();
		vector measure_magnetometer();
		float integrate(float u);
		vector integrate_vector(vector u);
		float get_delta_t();
		void initialize();
		float low_pass_filter(float initial_data, float final_data, float beta);

	private:
		float delta_t = 0;
		
		
};