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
#include "vector3.h"

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

Orientation::Orientation() {

}

void Orientation::init(bool enable_gyro, bool enable_accelmag) {

	if (enable_gyro) {
		if (!gyro.begin()) {
			Serial.println("There was a problem detecting the FXAS21002C ... check your connections");
			while (1);
		}
	}

	if (enable_accelmag) {
		if (!accelmag.begin(ACCEL_RANGE_4G)) {
			Serial.println("There was a problem detecting the FXOS8700 ... check your connections");
			while (1);
		}
	}

	//calibrate magnetometer
	setup_magnetometer();

	float i[]{ 1,0,0 };
	float j[]{ 0,1,0 };
	float k[]{ 0,0,1 };

	this->i_vector.set_vector(i);
	this->j_vector.set_vector(j);
	this->k_vector.set_vector(k);

	/*
	this->angular_velocity_initial.x = 0;
	this->angular_velocity_initial.y = 0;
	this->angular_velocity_initial.z = 0;

	this->angular_velocity_final.x = 0;
	this->angular_velocity_final.y = 0;
	this->angular_velocity_final.z = 0;
	*/

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

	//create acceleration vector:
	vector accelerometer_vector;

	//read accelerometor sensor:
	int t_initial = micros();
	accelerometer_vector.x = aevent.acceleration.x;
	accelerometer_vector.y = aevent.acceleration.y;
	accelerometer_vector.z = aevent.acceleration.z;
	int t_final = micros();
	this->delta_t = (t_final - t_initial);

	if (this->delta_t == 0) {
		this->delta_t = 1;
	}
	
	//Measured from calibration curve
	vector zero_G_Bias;
	zero_G_Bias.x = 0.0;//-0.1068;
	zero_G_Bias.y = 0.0;//0.4023;
	zero_G_Bias.z = 0.0;

	vector sensitivity;
	sensitivity.x = 1.0;//1.0985;
	sensitivity.y = 1.0;//1.0745;
	sensitivity.z = 1.0;

	accelerometer_vector.x = (accelerometer_vector.x - zero_G_Bias.x) / sensitivity.x;
	accelerometer_vector.y = (accelerometer_vector.y - zero_G_Bias.y) / sensitivity.y;
	accelerometer_vector.z = (accelerometer_vector.z - zero_G_Bias.z) / sensitivity.z;

	return accelerometer_vector;
}

void Orientation::calibrate_acclerometer() {

	int count = 0;
	int angle[36];
	float data_x[36];
	float data_y[36];
	float data_z[36];

	for (int i = 0; i <= 180; i += 5) {
		//button_wait();

		angle[count] = i;

		vector mean_accel;
		//average measurements
		for (int j = 0; j < 10; j++) {
			vector acceleration = this->measure_accelerometer();
			//accel_angles = get_accel_angles(acceleration);
			mean_accel.x += acceleration.x;
			mean_accel.y += acceleration.y;
			mean_accel.z += acceleration.z;
		}

		mean_accel.x = mean_accel.x / 10;
		mean_accel.y = mean_accel.y / 10;
		mean_accel.z = mean_accel.z / 10;

		data_x[count] = mean_accel.x;
		data_y[count] = mean_accel.y;
		data_z[count] = mean_accel.z;

		Serial.print(angle[count]);
		Serial.print(",");
		Serial.print(data_x[count]);
		Serial.print(",");
		Serial.print(data_y[count]);
		//Serial.print(",");
		//Serial.print(data_z[count]);
		Serial.println();
		delay(800);

		count++;
	}
	Serial.println("end calibration");
}

vector Orientation::get_accel_angles(vector acceleration) {
	vector angles;
	angles.x = acceleration.angle_between_vectors(this->i_vector);
	angles.y = acceleration.angle_between_vectors(this->j_vector);
	angles.z = acceleration.angle_between_vectors(this->k_vector);

	angles.convert_to_degrees();

	return angles;
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

	//calibrate magnetometer
	magnetometer_raw.set(0, 0, magnetometer_vector.x);
	magnetometer_raw.set(1, 0, magnetometer_vector.y);
	magnetometer_raw.set(2, 0, magnetometer_vector.z);

	magnetometer_calibrated = magnetic_transformation_matrix * (magnetometer_raw - magnetic_offset_matrix);

	magnetometer_vector.x = magnetometer_calibrated[0][0];
	magnetometer_vector.y = magnetometer_calibrated[1][0];
	magnetometer_vector.z = magnetometer_calibrated[2][0];

	return magnetometer_vector;
}

void Orientation::setup_magnetometer() {

	magnetic_transformation_matrix = Matrix(3, 3);
	magnetic_offset_matrix = Matrix(3, 1);
	magnetometer_raw = Matrix(3, 1);
	magnetometer_calibrated = Matrix(3, 1);

	magnetic_offset_matrix.set(0, 0, 0.0f);
	magnetic_offset_matrix.set(1, 0, 0.0f);
	magnetic_offset_matrix.set(2, 0, 0.0f);

	magnetic_transformation_matrix.set(0, new float[3]{ 1.014f, -0.020f, 0.012f });
	magnetic_transformation_matrix.set(1, new float[3]{ -0.020f, 0.985f, 0.005f });
	magnetic_transformation_matrix.set(2, new float[3]{ 0.012f, 0.005f, 1.002f });
}

vector Orientation::magnetism() {

}

vector Orientation::get_mag_angles(vector magnetism) {
	vector angles;
	angles.x = magnetism.angle_between_vectors(this->i_vector);
	angles.y = magnetism.angle_between_vectors(this->j_vector);
	angles.z = magnetism.angle_between_vectors(this->k_vector);

	angles.convert_to_degrees();

	return angles;
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
		sum.x += sq((ringbuffer_at(variance_buffer_x, i) - sample_mean.x));
		sum.y += sq((ringbuffer_at(variance_buffer_y, i) - sample_mean.y));
		sum.z += sq((ringbuffer_at(variance_buffer_z, i) - sample_mean.z));
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
