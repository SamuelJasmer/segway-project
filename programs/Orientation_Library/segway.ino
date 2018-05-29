
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Wire.h>
#include "orientation_library.h"
#include "matrix.h"

Orientation segway_orientation;

vector i_vector;
vector j_vector;
vector k_vector;

vector accel_angles;
vector mag_angles;
vector angular_velocity;
vector angular_displacement;

vector mag_raw;
vector accel_raw;

vector magnetic;
vector acceleration;

Matrix magnetic_transformation_matrix;
Matrix magnetic_offset_matrix;
Matrix magnetometer_raw;
Matrix magnetometer_calibrated;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
	Serial.begin(9600);

	inputString.reserve(200);

	segway_orientation.initialize();
	
	float i[]{1,0,0};
	float j[]{0,1,0};
	float k[]{0,0,1};

	magnetic_transformation_matrix = Matrix(3,3);
	magnetic_offset_matrix = Matrix(3,1);	
	magnetometer_raw = Matrix(3,1);
	magnetometer_calibrated = Matrix(3,1);

	magnetic_offset_matrix.set(0, 0, 0.0f);
	magnetic_offset_matrix.set(1, 0, 0.0f);
	magnetic_offset_matrix.set(2, 0, 0.0f);

	magnetic_transformation_matrix.set(0, new float[3]{ 1.014f, -0.020f, 0.012f });
	magnetic_transformation_matrix.set(1, new float[3]{ -0.020f, 0.985f, 0.005f });
	magnetic_transformation_matrix.set(2, new float[3]{ 0.012f, 0.005f, 1.002f });

	i_vector.set_vector(i);
	j_vector.set_vector(j);
	k_vector.set_vector(k);
}

void loop() {
	//calculate_acceleration();
	//print_acceleration();
	//Serial.print(" , ");
	//print_accel_angles();

	calculate_magnetism();
	print_mag_cal(magnetometer_calibrated);
	Serial.print(" , ");
	print_mag_angles();

	//calculate_angular_motion();
	//print_angular_velocity();
	//Serial.print(" , ");
	//print_theta();

	Serial.println();
}

void calculate_acceleration() {
	accel_raw = segway_orientation.measure_accelerometer();
	get_acceleration(accel_raw);
	get_accel_angles(acceleration);
	accel_angles.convert_to_degrees();
}

void get_accel_angles(vector acceleration) {
	accel_angles.x = acceleration.angle_between_vectors(i_vector);
	accel_angles.y = acceleration.angle_between_vectors(j_vector);
	accel_angles.z = acceleration.angle_between_vectors(k_vector);
}

void calculate_magnetism() {
	mag_raw = segway_orientation.measure_magnetometer();
	get_magnetic(mag_raw);
	calibrate_magnetometer(magnetic);
	vector magnetometer_calibrated_vector;
	magnetometer_calibrated_vector.x = magnetometer_calibrated[0][0];
	magnetometer_calibrated_vector.y = magnetometer_calibrated[1][0];
	magnetometer_calibrated_vector.z = magnetometer_calibrated[2][0];

	get_mag_angles(magnetometer_calibrated_vector);
	mag_angles.convert_to_degrees();
}

void calculate_angular_motion() {
	vector angular_velocity = segway_orientation.measure_gyro();
	vector angular_displacement = segway_orientation.integrate_vector(angular_velocity);
	angular_displacement.convert_to_degrees();
}

void calibrate_magnetometer(vector magnetic) {
	magnetometer_raw.set(0, 0, magnetic.x);
	magnetometer_raw.set(1, 0, magnetic.y);
	magnetometer_raw.set(2, 0, magnetic.z);

	magnetometer_calibrated = magnetic_transformation_matrix * (magnetometer_raw - magnetic_offset_matrix);
}

void get_mag_angles(vector magnetometer_cal_vector) {
	mag_angles.x = magnetometer_cal_vector.angle_between_vectors(i_vector);
	mag_angles.y = magnetometer_cal_vector.angle_between_vectors(j_vector);
	mag_angles.z = magnetometer_cal_vector.angle_between_vectors(k_vector);
}

void get_magnetic(vector mag) {
	magnetic.x = mag.x;
	magnetic.y = mag.y;
	magnetic.z = mag.z;
}

void get_acceleration(vector accel) {
	acceleration.x = accel.x;
	acceleration.y = accel.y;
	acceleration.z = accel.z;
}

void print_acceleration() {
	Serial.print(acceleration.x);
	Serial.print(",");
	Serial.print(acceleration.y);
	Serial.print(",");
	Serial.print(acceleration.z);
}

void print_accel_angles() {
	Serial.print(accel_angles.x);
	Serial.print(",");
	Serial.print(accel_angles.y);
	Serial.print(",");
	Serial.print(accel_angles.z);
}

void print_mag_angles() {
	Serial.print(mag_angles.x);
	Serial.print(",");
	Serial.print(mag_angles.y);
	Serial.print(",");
	Serial.print(mag_angles.z);
}

void print_mag_raw(vector magnetic) {
	Serial.print(magnetic.x);
	Serial.print(",");
	Serial.print(magnetic.y);
	Serial.print(",");
	Serial.print(magnetic.z);
}

void print_mag_cal(Matrix magnetometer_cal) {
	Serial.print(magnetometer_cal[0][0]);
	Serial.print(",");
	Serial.print(magnetometer_cal[1][0]);
	Serial.print(",");
	Serial.print(magnetometer_cal[2][0]);
}

void print_theta() {
	Serial.print(angular_displacement.x);
	Serial.print(",");
	Serial.print(angular_displacement.y);
	Serial.print(",");
	Serial.print(angular_displacement.z);
}

void print_angular_velocity() {
	Serial.print(angular_velocity.x);
	Serial.print(",");
	Serial.print(angular_velocity.y);
	Serial.print(",");
	Serial.print(angular_velocity.z);
}

