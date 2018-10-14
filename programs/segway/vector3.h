#ifndef VECTOR3_H
#define VECTOR3_H

struct vector {

	public:
		float x;
		float y;
		float z;
		float magnitude();

		void set_vector(float vector_array[]);
		void clear();
		void convert_to_degrees();
		void convert_to_radians();

		vector cross_product(vector v);

		float dot_product(vector v);
		float angle_between_vectors(vector v);
};

#endif
