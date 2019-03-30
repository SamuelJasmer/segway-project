#include "vector3.h"

#include <math.h>
#include "Arduino.h"
#ifndef PI
#define PI 3.14159265358979323846
#endif

float vector::magnitude() {
	//calculate the magnitude of this vector:
	float magnitude = sqrt(sq(this->x) + sq(this->y) + sq(this->z));

	return magnitude;
}

float vector::dot_product(vector v) {
	//calculate the dot product of this vector with given vector v:
	float n = (this->x * v.x) + (this->y * v.y) + (this->z * v.z);
	return n;
}

float vector::angle_between_vectors(vector v) {
	//calculate the angle between this vector and given vector v:
	float theta = acos((this->dot_product(v))/(this->magnitude() * v.magnitude()));
	return theta;
}

vector vector::cross_product(vector v) {
	//calculate the cross product of this vector with given vector v:
	vector n;
	n.x = ((this->y * v.z) - (this->z * v.y));
	n.y = -1 * ((this->x * v.z) - (this->z * v.x));
	n.z = ((this->x * v.y) - (this->y * v.x));

	return n;
}

void vector::convert_to_degrees() {

	this->x = (this->x * 180) / PI;
	this->y = (this->y * 180) / PI;
	this->z = (this->z * 180) / PI;
}

void vector::convert_to_radians() {

	this->x = (this->x * PI) / 180;
	this->y = (this->y * PI) / 180;
	this->z = (this->z * PI) / 180;
}

void vector::set_vector(float vector_array[]) {
	//Takes a one-dimensional array and converts it to a vector

	this->x = vector_array[0];
	this->y = vector_array[1];
	this->z = vector_array[2];
}

void vector::clear() {
	//Clear
	this->x = 0;
	this->y = 0;
	this->z = 0;
}
