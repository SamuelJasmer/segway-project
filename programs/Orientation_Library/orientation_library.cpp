/*
 * orientation_library.cpp
 */

#include "orientation_library.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Wire.h>

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

Orientation::Orientation() {

	this->delta_t = 0;
}

void Orientation::initialize() {
	//Serial.println("Gyroscope Test"); Serial.println("");

	if (!gyro.begin()) {
		Serial.println("There was a problem detecting the FXAS21002C ... check your connections");
		while (1);
	}

	//Serial.println("FXOS8700 Test"); Serial.println("");

	if (!accelmag.begin(ACCEL_RANGE_4G)) {
		Serial.println("There was a problem detecting the FXOS8700 ... check your connections");
		while (1);
	}
}

vector Orientation::measure_gyro() {

	sensors_event_t event;
	gyro.getEvent(&event);

	int t_initial = micros();
	float x = event.gyro.x;
	float y = event.gyro.y;
	float z = event.gyro.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);


	vector gyro_vector;
	gyro_vector.x = x;
	gyro_vector.y = y;
	gyro_vector.z = z;

	return gyro_vector;
}

vector Orientation::measure_accelerometer() {

	sensors_event_t aevent;
	accelmag.getEvent(&aevent);

	int t_initial = micros();
	float x = aevent.acceleration.x;
	float y = aevent.acceleration.y;
	float z = aevent.acceleration.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);
	

	vector accelerometer_vector;
	accelerometer_vector.x = x;
	accelerometer_vector.y = y;
	accelerometer_vector.z = z;

	return accelerometer_vector;
}

vector Orientation::measure_magnetometer() {

	sensors_event_t mevent;
	accelmag.getEvent(&mevent);

	int t_initial = micros();
	float x = mevent.magnetic.x;
	float y = mevent.magnetic.y;
	float z = mevent.magnetic.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);

	vector magnetometer_vector;
	magnetometer_vector.x = x;
	magnetometer_vector.y = y;
	magnetometer_vector.z = z;

	return magnetometer_vector;
}

float Orientation::integrate(float u) {

	float int_u;
	int_u += u * (this->delta_t);

	return int_u;
}

vector Orientation::integrate_vector(vector u) {

	vector integral_u;
	integral_u.x = Orientation::integrate(u.x);
	integral_u.y = Orientation::integrate(u.y);
	integral_u.z = Orientation::integrate(u.z);

	return integral_u;
}

float Orientation::get_delta_t() {
	
	return this->delta_t;
}

float vector::magnitude() {

	float magnitude = sqrt(sq(this->x) + sq(this->y) + sq(this->z));

	return magnitude;
}

float vector::dot_product(vector v) {

	float n = (this->x * v.x) + (this->y * v.y) + (this->z * v.z);
	return n;
}

float vector::angle_between_vectors(vector v) {

	float theta = acos((this->dot_product(v))/(this->magnitude() * v.magnitude()));
	return theta;
}

vector vector::cross_product(vector v) {

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

	this->x = vector_array[0];
	this->y = vector_array[1];
	this->z = vector_array[2];
}

float Orientation::low_pass_filter(float initial_data, float final_data, float beta) {

	float filtered_data = (beta*final_data) + ((1 - beta)*(initial_data));
	
	return filtered_data;
}

