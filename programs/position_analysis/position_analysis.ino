#include "Adafruit_Sensor.h"
#include "Adafruit_FXOS8700.h"
#include "Adafruit_FXAS21002C.h"
#include "Wire.h"
#include "Orientation_Library.h"
#include "matrix.h"
#include "ArduinoSTL.h"
#include <cstring>
#include <iostream>
#include "stdlib.h"
#include <string.h>
#include <vector>

/*
 *Declare variables and constants
 */

//Instantiate Orientation Library:
Orientation segway_orientation;

float MoI;

//Unit Vectors:
vector i_vector;
vector j_vector;
vector k_vector;

//Acceleration Vectors:
vector accel_raw;
vector acceleration;//unnecessary, mearly converts name of accel_raw to acceleration
vector accel_angles;//doesn't work as intended

//Angular Velocity Vectors:
vector angular_velocity;
vector angular_displacement;

//Magnetic Vectors:
vector mag_raw;
vector magnetic;//unnecessary, mearly converts name of mag_raw to magnetic
vector mag_angles;//doesn't work as intended

//Magnetic Calibration Matricies:
Matrix magnetic_transformation_matrix;
Matrix magnetic_offset_matrix;
Matrix magnetometer_raw;
Matrix magnetometer_calibrated;

//Serial Plotter Communication:
std::vector<char>* serialInputBuf;
void checkSerial();
void parseCSV(std::string input, std::vector<std::string>* argv);

/*
 *Setup Loop
 */

void setup() {
	Serial.begin(19200);

	serialInputBuf = new std::vector<char>();

	segway_orientation.initialize();

	//Moment of Interia:
	MoI = segway_orientation.calculate_Moment_of_Inertia();
	
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

/*
 *Main Loop
 */

void loop() {

	checkSerial();
	
	vector angular_velocity = segway_orientation.measure_gyro();
	vector angular_acceleration = segway_orientation.angular_acceleration();
	//vector delta_theta = new_angle_approx(angular_acceleration, segway_orientation.angular_velocity_final, segway_orientation.angular_velocity_initial);

	//Serial.print(segway_orientation.angular_velocity_final.x);
	//Serial.print(" , ");
	//Serial.print(segway_orientation.angular_velocity_final.y);
	//Serial.print(" , ");
	//Serial.print(segway_orientation.angular_velocity_final.z);

	//Serial.print("    ");

	//Serial.print(segway_orientation.angular_velocity_initial.x);
	//Serial.print(" , ");
	//Serial.print(segway_orientation.angular_velocity_initial.y);
	//Serial.print(" , ");
	//Serial.print(segway_orientation.angular_velocity_initial.z);

	//Serial.print("    ");

	//Serial.print(delta_theta.x * 100000);
	//Serial.print(" , ");
	//Serial.print(delta_theta.y * 100000);
	//Serial.print(" , ");
	//Serial.print(delta_theta.z * 100000);
	//Serial.println();

	//calculate_acceleration();
	//print_acceleration();
	//Serial.println();
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
}


/*
 *
 */

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

vector new_angle_approx(vector angular_acceleration, vector angular_velocity_final, vector angular_velocity_initial) {
	vector delta_theta;
	//delta_theta.x = (1 / (2 * angular_acceleration.x)) * (pow(angular_velocity_final.x, 2.0) - pow(angular_velocity_initial.x, 2.0));
	//delta_theta.y = (1 / (2 * angular_acceleration.y)) * (pow(angular_velocity_final.y, 2.0) - pow(angular_velocity_initial.y, 2.0));
	//delta_theta.z = (1 / (2 * angular_acceleration.z)) * (pow(angular_velocity_final.z, 2.0) - pow(angular_velocity_initial.z, 2.0));

	delta_theta.x = ((angular_velocity_final.x * angular_velocity_final.x) - (angular_velocity_initial.x * angular_velocity_initial.x));
	delta_theta.y = ((angular_velocity_final.y * angular_velocity_final.y) - (angular_velocity_initial.y * angular_velocity_initial.y));
	delta_theta.z = ((angular_velocity_final.z * angular_velocity_final.z) - (angular_velocity_initial.z * angular_velocity_initial.z));

	return delta_theta;
}

/*
 *Display Functions:
 */

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

void checkSerial()
{
	char c;
	while (Serial.available() > 0)
	{
		c = (char)Serial.read();
		if (c >= 0x20 && c < 0x7f)
		{
			serialInputBuf->push_back(c);
		}
		else if (c == '\n')
		{
			std::vector<std::string> inputSep;
			std::string input(serialInputBuf->begin(), serialInputBuf->end());
			serialInputBuf->clear();

			parseCSV(input, &inputSep);
		}
	}
}

void parseCSV(std::string input, std::vector<std::string>* argv)
{
	std::string current = "";
	for (int i = 0; i < input.size(); ++i)
	{
		if (input[i] == ',')
		{
			argv->push_back(current);
			current = "";
		}
		else
		{
			current += input[i];
		}
	}
	argv->push_back(current);
}