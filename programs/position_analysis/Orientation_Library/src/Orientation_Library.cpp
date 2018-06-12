/*
 * orientation_library.cpp
 */

#include "Orientation_Library.h"
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
	
	if (!gyro.begin()) {
		Serial.println("There was a problem detecting the FXAS21002C ... check your connections");
		while (1);
	}
	
	if (!accelmag.begin(ACCEL_RANGE_4G)) {
		Serial.println("There was a problem detecting the FXOS8700 ... check your connections");
		while (1);
	}

	this->angular_velocity_initial.x = 0;
	this->angular_velocity_initial.y = 0;
	this->angular_velocity_initial.z = 0;

	this->angular_velocity_final.x = 0;
	this->angular_velocity_final.y = 0;
	this->angular_velocity_final.z = 0;

	this->gyro_buffer_x[0] = 0;
	this->gyro_buffer_y[0] = 0;
	this->gyro_buffer_z[0] = 0;

	this->gyro_buffer_x[1] = 0;
	this->gyro_buffer_y[1] = 0;
	this->gyro_buffer_z[1] = 0;
}

vector Orientation::measure_gyro() {

	//initialize sensor:
	sensors_event_t event;
	gyro.getEvent(&event);

	//read from gyroscope sensor:
	int t_initial = micros();
	float x = event.gyro.x;
	float y = event.gyro.y;
	float z = event.gyro.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);

	//create gyro vector:
	this->gyro_vector.x = x;
	this->gyro_vector.y = y;
	this->gyro_vector.z = z;

	return gyro_vector;
}

vector Orientation::smooth_gyro(vector gyro_vector) {

	this->gyro_buffer_x[2] = gyro_vector.x;
	this->gyro_buffer_y[2] = gyro_vector.y;
	this->gyro_buffer_z[2] = gyro_vector.z;

	float averaged_gyro_x = Orientation::moving_average_3n(gyro_buffer_x);
	float averaged_gyro_y = Orientation::moving_average_3n(gyro_buffer_y);
	float averaged_gyro_z = Orientation::moving_average_3n(gyro_buffer_z);

	this->gyro_buffer_x[0] = this->gyro_buffer_x[1];
	this->gyro_buffer_y[0] = this->gyro_buffer_y[1];
	this->gyro_buffer_z[0] = this->gyro_buffer_z[1];

	this->gyro_buffer_x[1] = this->gyro_buffer_x[2];
	this->gyro_buffer_y[1] = this->gyro_buffer_y[2];
	this->gyro_buffer_z[1] = this->gyro_buffer_z[2];

}

vector Orientation::angular_acceleration() {

	//Derivative Approximation of angular acceleration from angular velocity measure by the gyroscope
	this->angular_velocity_final.x = this->gyro_vector.x;
	this->angular_velocity_final.y = this->gyro_vector.y;
	this->angular_velocity_final.z = this->gyro_vector.z;


	vector angular_acceleration;
	angular_acceleration.x = (this->angular_velocity_final.x - angular_velocity_initial.x) / this->delta_t;
	angular_acceleration.y = (this->angular_velocity_final.y - angular_velocity_initial.y) / this->delta_t;
	angular_acceleration.z = (this->angular_velocity_final.z - angular_velocity_initial.z) / this->delta_t;

	//Reset initial coniditions
	this->angular_velocity_initial.x = this->angular_velocity_final.x;
	this->angular_velocity_initial.y = this->angular_velocity_final.y;
	this->angular_velocity_initial.z = this->angular_velocity_final.z;

	return angular_acceleration;
}

vector Orientation::measure_accelerometer() {

	//initialize sensor:
	sensors_event_t aevent;
	accelmag.getEvent(&aevent);

	//read accelerometor sensor:
	int t_initial = micros();
	float x = aevent.acceleration.x;
	float y = aevent.acceleration.y;
	float z = aevent.acceleration.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);
	
	//create acceleration vector:
	vector accelerometer_vector;
	accelerometer_vector.x = x;
	accelerometer_vector.y = y;
	accelerometer_vector.z = z;

	return accelerometer_vector;
}

vector Orientation::measure_magnetometer() {

	//initialize sensor:
	sensors_event_t mevent;
	accelmag.getEvent(&mevent);

	//read magnetometer sensor:
	int t_initial = micros();
	float x = mevent.magnetic.x;
	float y = mevent.magnetic.y;
	float z = mevent.magnetic.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);

	//create magnetic vector:
	vector magnetometer_vector;
	magnetometer_vector.x = x;
	magnetometer_vector.y = y;
	magnetometer_vector.z = z;

	return magnetometer_vector;
}

float Orientation::calculate_Moment_of_Inertia() {
	//calculate the moment of inertia of the rod:

	//measured in units of meters
	float radius = 0.008f;//8mm
	float length = 0.32f;//32cm
	float volume = (PI * radius * radius) * length;
	float density = 0.6e3;//kg/m^3
	float mass = density / volume;

	Serial.println(mass);

	float Moment_of_Inertia = (0.25)*(mass*pow(radius,2.0)) + (0.08333)*(mass*pow(length,2.0));//kg*m^2

	Serial.println(Moment_of_Inertia);

	return Moment_of_Inertia;
}

float Orientation::integrate(float u) {
	//integrate a given scalr value u:
	float int_u;
	int_u += u * (this->delta_t);

	return int_u;
}

vector Orientation::integrate_vector(vector u) {
	//integrate a given vector u:

	vector integral_u;
	integral_u.x = Orientation::integrate(u.x);
	integral_u.y = Orientation::integrate(u.y);
	integral_u.z = Orientation::integrate(u.z);

	return integral_u;
}

float Orientation::get_delta_t() {
	
	return this->delta_t;
}

float Orientation::moving_average_3n(float data_in[3]) {
	//Calculates the moving average with a sample window of 3 data points

	int n = 3;
	float buffer[n];
	buffer[0] = data_in[0];
	buffer[1] = data_in[1];
	buffer[2] = data_in[2];

	float MA = (buffer[0] + buffer[1] + buffer[2]) / n;

	return MA;

}

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

