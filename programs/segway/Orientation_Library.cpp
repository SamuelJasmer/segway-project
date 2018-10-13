/*
 * orientation_library.cpp
 */

#include "Orientation_Library.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Wire.h>
#include "matrix.h"
#include <stdio.h>
#include "ringbuffer.h" 

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

Orientation::Orientation() {

	this->delta_t = 0;
}

void Orientation::init() {

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

	//gyro buffer
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

	/*this->gyro_buffer_x[2] = gyro_vector.x;
	this->gyro_buffer_y[2] = gyro_vector.y;
	this->gyro_buffer_z[2] = gyro_vector.z;

	

	averaged_gyro.x = Orientation::moving_average_3n(gyro_buffer_x);
	averaged_gyro.y = Orientation::moving_average_3n(gyro_buffer_y);
	averaged_gyro.z = Orientation::moving_average_3n(gyro_buffer_z);

	this->gyro_buffer_x[0] = this->gyro_buffer_x[1];
	this->gyro_buffer_y[0] = this->gyro_buffer_y[1];
	this->gyro_buffer_z[0] = this->gyro_buffer_z[1];

	this->gyro_buffer_x[1] = this->gyro_buffer_x[2];
	this->gyro_buffer_y[1] = this->gyro_buffer_y[2];
	this->gyro_buffer_z[1] = this->gyro_buffer_z[2];
*/
	vector averaged_gyro;
	return averaged_gyro;

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

	const int n = 3;
	float buffer[n];
	buffer[0] = data_in[0];
	buffer[1] = data_in[1];
	buffer[2] = data_in[2];

	float MA = (buffer[0] + buffer[1] + buffer[2]) / n;

	return MA;
}

/*
* Vector Methods
*/

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


/*
 * Filter Methods
 */

void filter::init(int n) {

	for (int i = 1; i <= n; i++) {
		this->average_n += i;
	}
	this->average_n = this->average_n / n;

	this->mean_buffer_x       = ringbuffer_new(n);
	this->mean_buffer_y       = ringbuffer_new(n);
	this->mean_buffer_z       = ringbuffer_new(n);
	this->variance_buffer_x   = ringbuffer_new(n);
	this->variance_buffer_y   = ringbuffer_new(n);
	this->variance_buffer_z   = ringbuffer_new(n);
	this->covariance_buffer_x = ringbuffer_new(n);
	this->covariance_buffer_y = ringbuffer_new(n);
	this->covariance_buffer_z = ringbuffer_new(n);
	this->error_buffer		  = ringbuffer_new(n);
	this->weight_buffer		  = ringbuffer_new(n);
	this->Y_buffer			  = ringbuffer_new(n);

	printf("0x%04x\n", (int)this->mean_buffer_x);
	printf("0x%04x\n", (int)this->mean_buffer_y);
	printf("0x%04x\n", (int)this->mean_buffer_z);
	printf("0x%04x\n", (int)this->variance_buffer_x);
	printf("0x%04x\n", (int)this->variance_buffer_y);
	printf("0x%04x\n", (int)this->variance_buffer_z);
	printf("0x%04x\n", (int)this->covariance_buffer_x);
	printf("0x%04x\n", (int)this->covariance_buffer_y);
	printf("0x%04x\n", (int)this->covariance_buffer_z);
	printf("0x%04x\n", (int)this->error_buffer);
	printf("0x%04x\n", (int)this->weight_buffer);
	printf("0x%04x\n", (int)this->Y_buffer);

	ringbuffer_fill(this->mean_buffer_x, 0.0f);
	ringbuffer_fill(this->mean_buffer_y, 0.0f);
	ringbuffer_fill(this->mean_buffer_z, 0.0f);
	ringbuffer_fill(this->variance_buffer_x, 0.0f);
	ringbuffer_fill(this->variance_buffer_y, 0.0f);
	ringbuffer_fill(this->variance_buffer_z, 0.0f);
	ringbuffer_fill(this->covariance_buffer_x, 0.0f);
	ringbuffer_fill(this->covariance_buffer_y, 0.0f);
	ringbuffer_fill(this->covariance_buffer_z, 0.0f);
	ringbuffer_fill(this->error_buffer, 0.0f);
	ringbuffer_fill(this->weight_buffer, 0.0f);
	ringbuffer_fill(this->Y_buffer, 0.0f);

}

vector filter::sample_mean(vector input, int n) {
	//Calculate the sample mean of n number of data points

	ringbuffer_dequeue(this->mean_buffer_x);
	ringbuffer_enqueue(this->mean_buffer_x, input.x);
	
	ringbuffer_dequeue(this->mean_buffer_y);
	ringbuffer_enqueue(this->mean_buffer_y, input.y);

	ringbuffer_dequeue(this->mean_buffer_z);
	ringbuffer_enqueue(this->mean_buffer_z, input.z);

	vector sum;
	sum.clear();

	if (this->mean_buffer_x->count < n || this->mean_buffer_y->count < n || this->mean_buffer_z->count < n) {
		Serial.println("Sample mean buffer size is less than n");
	}
	else {
		for (int i = 0; i < n; i++) {
			sum.x += ringbuffer_at(this->mean_buffer_x, i);
			sum.y += ringbuffer_at(this->mean_buffer_y, i);
			sum.z += ringbuffer_at(this->mean_buffer_z, i);
		}
	}

	vector average;
	average.x = sum.x / n;
	average.y = sum.y / n;
	average.z = sum.z / n;

	return average;
}

vector filter::sample_variance(vector sample_mean, vector sample, int n) {

	//shift buffer
	ringbuffer_dequeue(this->variance_buffer_x);
	ringbuffer_enqueue(this->variance_buffer_x, sample.x);
	
	//shift buffer
	ringbuffer_dequeue(this->variance_buffer_y);
	ringbuffer_enqueue(this->variance_buffer_y, sample.y);

	//shift buffer
	ringbuffer_dequeue(this->variance_buffer_z);
	ringbuffer_enqueue(this->variance_buffer_z, sample.z);

	vector sum;
	sum.clear();

	//sum incoming data
	for (int i = 0; i < n; i++) {
		sum.x += pow((ringbuffer_at(variance_buffer_x, i) - sample_mean.x), 2);
		sum.y += pow((ringbuffer_at(variance_buffer_y, i) - sample_mean.y), 2);
		sum.z += pow((ringbuffer_at(variance_buffer_z, i) - sample_mean.z), 2);
	}

	vector sample_variance;
	sample_variance.x = sum.x / (n - 1);
	sample_variance.y = sum.y / (n - 1);
	sample_variance.z = sum.z / (n - 1);

	return sample_variance;

}

vector filter::covariance(vector sample_mean, vector sample, int n) {

	//Shift buffer
	ringbuffer_dequeue(this->covariance_buffer_x);
	ringbuffer_enqueue(this->covariance_buffer_x, sample.x);

	//Shift buffer
	ringbuffer_dequeue(this->covariance_buffer_y);
	ringbuffer_enqueue(this->covariance_buffer_y, sample.y);

	//Shift buffer
	ringbuffer_dequeue(this->covariance_buffer_z);
	ringbuffer_enqueue(this->covariance_buffer_z, sample.z);

	vector sum;
	sum.clear();

	for (int i = 0; i < n; i++) {
		sum.x += ((ringbuffer_at(covariance_buffer_x, i) - sample_mean.x) * (i - average_n));
		sum.y += ((ringbuffer_at(covariance_buffer_y, i) - sample_mean.y) * (i - average_n));
		sum.z += ((ringbuffer_at(covariance_buffer_z, i) - sample_mean.z) * (i - average_n));
	}

	vector cov;
	cov.x = sum.x / n;
	cov.y = sum.y / n;
	cov.z = sum.z / n;

	return cov;
}

vector filter::standard_deviation(vector sample_variance) {
	
	vector std_dev;

	std_dev.x = sqrt(sample_variance.x);
	std_dev.y = sqrt(sample_variance.y);
	std_dev.z = sqrt(sample_variance.z);

	return std_dev;

}

vector filter::least_squares_regression(vector input, int n) {

	vector mean = this->sample_mean(input, n);
	vector cov = this->covariance(mean, input, n);
	vector variance = this->sample_variance(mean, input, n);

	vector b;
	b.x = cov.x / variance.x;
	b.y = cov.y / variance.y;
	b.z = cov.z / variance.z;

	vector a;
	a.x = mean.x - b.x * this->average_n;
	a.y = mean.y - b.y * this->average_n;
	a.z = mean.z - b.z * this->average_n;

	vector output;

	output.x = a.x + b.x * (n / 2);
	output.y = a.y + b.y * (n / 2);
	output.z = a.z + b.z * (n / 2);

	return output;
}

Matrix filter::create_error_Matrix(float current_variance, int n) {

	//Shift buffer
	ringbuffer_dequeue(this->error_buffer);
	ringbuffer_enqueue(this->error_buffer, current_variance);

	Matrix Error_matrix;
	Error_matrix = Matrix(n, 1);

	//Create Error Matrix
	for (int i = 0; i < n; i++) {
		//Create a column vector of length n with each position as the current variance at i
		Error_matrix.set(i, 0, ringbuffer_at(error_buffer, i));
	}

	return Error_matrix;
}

Matrix filter::create_weight_Matrix(float current_variance, int n) {

	//Shift buffer
	ringbuffer_dequeue(this->weight_buffer);
	ringbuffer_enqueue(this->weight_buffer, current_variance);

	Matrix Weight_matrix;
	Weight_matrix = Matrix(n, n);

	//Create Weight Matrix
	for (int row = 0; row < n; row++) {
		//row
		for (int col = 0; col < n; col++) {
			//column
			if (row == col) {
				//set diagonal positions to ( 1 / current variance ) at that row
				Weight_matrix.set(row, col, 1 / ringbuffer_at(weight_buffer, row));
			}
			else {
				//set non diagonal positions to 0
				Weight_matrix.set(row, col, 0);
			}
		}
	}

	return Weight_matrix;
}

Matrix filter::weighted_least_squares_regression(float input, float variance, int n) {

	ringbuffer_dequeue(this->Y_buffer);
	ringbuffer_enqueue(this->Y_buffer, input);

	Matrix X_Matrix;
	X_Matrix = Matrix(n, 2);

	Matrix Y_Matrix;
	Y_Matrix = Matrix(n, 1);

	Matrix B_Matrix;
	B_Matrix = Matrix(2, 1);

	Matrix Error_Matrix = create_error_Matrix(variance, n);
	Matrix Weight_Matrix = create_weight_Matrix(variance, n);

	for (int i = 0; i < n; i++) {
		X_Matrix.set(i, 0, i);
		X_Matrix.set(i, 1, i);
		Y_Matrix.set(i, 0, ringbuffer_at(Y_buffer, i));
	}

	//Calculate slope

	B_Matrix = (X_Matrix.transpose() * Weight_Matrix * X_Matrix).inverse() * (X_Matrix.transpose() * Weight_Matrix * Y_Matrix);

	return B_Matrix;
}

vector filter::lowess_smooth(vector input, int n) {

	vector mean = this->sample_mean(input, n);
	vector variance = this->sample_variance(mean, input, n);

	Matrix B_Matrix_x;
	B_Matrix_x = weighted_least_squares_regression(input.x, variance.x, n);

	Matrix B_Matrix_y;
	B_Matrix_y = weighted_least_squares_regression(input.y, variance.y, n);

	Matrix B_Matrix_z;
	B_Matrix_z = weighted_least_squares_regression(input.z, variance.z, n);

	vector output;
	output.x = B_Matrix_x.get(0, 0) * (n / 2) + B_Matrix_x.get(1, 0);
	output.y = B_Matrix_y.get(0, 0) * (n / 2) + B_Matrix_y.get(1, 0);
	output.z = B_Matrix_z.get(0, 0) * (n / 2) + B_Matrix_z.get(1, 0);

	return output;
}