/*
    Name:       segway.ino
    Created:	6/5/2018 9:47:06 PM
    Author:     AD\jasmersr
*/

#include <Servo.h>
#include "PID.h"
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
#include <string>
#include <vector>
#include "L298N.h"

#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5
#define ANALOG_0 0
#define ANALOG_1 1
#define ANALOG_2 2
#define ANALOG_3 3

L298N MotorA(ENA, IN_A1, IN_A2);
L298N MotorB(ENB, IN_B1, IN_B2);

//Instantiate Orientation Library:
Orientation segway_orientation;

//Unit Vectors:
vector i_vector;
vector j_vector;
vector k_vector;

//Acceleration Vectors:
vector accel_raw;
vector acceleration;//unnecessary, mearly converts name of accel_raw to acceleration
vector accel_angles;

//Angular Velocity Vectors:
vector angular_velocity;
vector angular_displacement;
vector zero_point;

//Magnetic Vectors:
vector mag_raw;
vector magnetic;//unnecessary, mearly converts name of mag_raw to magnetic
vector mag_angles;

vector averaged_angles;

//Magnetic Calibration Matricies:
Matrix magnetic_transformation_matrix;
Matrix magnetic_offset_matrix;
Matrix magnetometer_raw;
Matrix magnetometer_calibrated;

//Instantiate PID Object:
PID pid(ANALOG_0);

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

int set_point = 90;
float pid_speed = 0;

//Setup Serial Plotter:
std::vector<char>* serialInputBuf;
void checkSerial();
void parseCSV(std::string input, std::vector<std::string>* argv);

void setup() {

	Serial.begin(19200);	
	while (!Serial);
	serialInputBuf = new std::vector<char>();

	//Setup Segway:
	segway_orientation.initialize();

	float i[]{ 1,0,0 };
	float j[]{ 0,1,0 };
	float k[]{ 0,0,1 };

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

	i_vector.set_vector(i);
	j_vector.set_vector(j);
	k_vector.set_vector(k);

	//Run Gyroscope Calibration:
	//Serial.println(F("Calibrating Gyro"));
	calibrate_gyro();

	//Motor Test:


}

void loop() {

	checkSerial();

	angular_velocity = segway_orientation.measure_gyro();

	//Serial.print(angular_velocity.x);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.y);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.z);

	//Serial.print("    ");

	calculate_magnetism();
	calculate_acceleration();
	averaged_angles = calculate_average_angles();

	//print_mag_angles();
	//Serial.print("  ");
	//print_accel_angles();
	//Serial.print("  ");
	//print_averaged_angles();

	pid.set_k_values(kp, ki, kd);
	pid_speed = abs(pid.calculate_pid(averaged_angles.y, set_point));

	if (pid_speed == 0) {

	}
	else if (pid_speed > 255) {
		pid_speed = 255;
	}
	else if (pid_speed < 80) {
		pid_speed = 0;
	}
	if (averaged_angles.y > set_point) {
		MotorA.forward();
		MotorA.set_speed(pid_speed);
	}
	else if (averaged_angles.y < set_point) {
		MotorA.reverse();
		MotorA.set_speed(pid_speed);
	}
	else {
		MotorA.stop();
	}

	print_pid_data();



	/*
	vector angular_velocity = segway_orientation.measure_gyro();
	//vector angular_acceleration = segway_orientation.angular_acceleration();
	//vector acceleration = segway_orientation.measure_accelerometer();

	//Serial.print(angular_velocity.x);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.y);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.z);

	//Serial.print("    ");

	angular_velocity.x = angular_velocity.x - zero_point.x;
	angular_velocity.y = angular_velocity.y - zero_point.y;
	angular_velocity.z = angular_velocity.z - zero_point.z;

	vector theta = segway_orientation.integrate_vector(angular_velocity);

	vector smoothed_angular_velocity = segway_orientation.smooth_gyro(angular_velocity);

	Serial.print(theta.x * 10000);
	Serial.print(" , ");
	Serial.print(theta.y * 10000);
	Serial.print(" , ");
	Serial.print(theta.z * 10000);

	Serial.println();

	//Serial.print(angular_velocity.x);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.y);
	//Serial.print(" , ");
	//Serial.print(angular_velocity.z);

	//Serial.print("    ");

	//Serial.print(smoothed_angular_velocity.x);
	//Serial.print(" , ");
	//Serial.print(smoothed_angular_velocity.y);
	//Serial.print(" , ");
	//Serial.print(smoothed_angular_velocity.z);

	/*

	Serial.print(angular_acceleration.x * 10000);
	Serial.print(" , ");
	Serial.print(angular_acceleration.y * 10000);
	Serial.print(" , ");
	Serial.print(angular_acceleration.z * 10000);

	Serial.print("    ");

	Serial.print(acceleration.x);
	Serial.print(" , ");
	Serial.print(acceleration.y);
	Serial.print(" , ");
	Serial.print(acceleration.z);
	
	*/


}

/*
 *
 */

void calibrate_gyro() {
	int n = 1;
	vector sum;

	while (millis() < 5000) {
		//Calculate the zero point of the gyroscope in each direction.
		//Lay the sensor flat and run this function.

		vector angular_velocity = segway_orientation.measure_gyro();

		sum.x += angular_velocity.x;
		sum.y += angular_velocity.y;
		sum.z += angular_velocity.z;

		zero_point.x = sum.x / n;
		zero_point.y = sum.y / n;
		zero_point.z = sum.z / n;

		n++;
	}
}

vector calculate_average_angles() {
	vector averaged_angles;
	averaged_angles.x = (accel_angles.x + mag_angles.x) / 2;
	averaged_angles.y = (accel_angles.y + mag_angles.y) / 2;
	averaged_angles.z = (accel_angles.z + mag_angles.z) / 2;

	return averaged_angles;
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

void print_averaged_angles() {
	Serial.print(averaged_angles.x);
	Serial.print(" , ");
	Serial.print(averaged_angles.y);
	Serial.print(" , ");
	Serial.print(averaged_angles.z);

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
	Serial.print(" , ");
	Serial.print(acceleration.y);
	Serial.print(" , ");
	Serial.print(acceleration.z);
}

void print_accel_angles() {
	Serial.print(accel_angles.x);
	Serial.print(" , ");
	Serial.print(accel_angles.y);
	Serial.print(" , ");
	Serial.print(accel_angles.z);
}

void print_mag_angles() {
	Serial.print(mag_angles.x);
	Serial.print(" , ");
	Serial.print(mag_angles.y);
	Serial.print(" , ");
	Serial.print(mag_angles.z);
}

void print_mag_raw(vector magnetic) {
	Serial.print(magnetic.x);
	Serial.print(" , ");
	Serial.print(magnetic.y);
	Serial.print(" , ");
	Serial.print(magnetic.z);
}

void print_mag_cal(Matrix magnetometer_cal) {
	Serial.print(magnetometer_cal[0][0]);
	Serial.print(" , ");
	Serial.print(magnetometer_cal[1][0]);
	Serial.print(" , ");
	Serial.print(magnetometer_cal[2][0]);
}

void print_theta() {
	Serial.print(angular_displacement.x);
	Serial.print(" , ");
	Serial.print(angular_displacement.y);
	Serial.print(" , ");
	Serial.print(angular_displacement.z);
}

void print_angular_velocity() {
	Serial.print(angular_velocity.x);
	Serial.print(" , ");
	Serial.print(angular_velocity.y);
	Serial.print(" , ");
	Serial.print(angular_velocity.z);
}

void print_pid_data() {
	Serial.print(averaged_angles.y);
	Serial.print(",");
	Serial.print(pid.p);
	Serial.print(",");
	Serial.print(pid.i);
	Serial.print(",");
	Serial.print(pid.d);
	Serial.print(",");
	Serial.print(pid.pid);
	Serial.print(",");
	Serial.print(pid_speed);
	Serial.print(",");
	Serial.print(kp);
	Serial.print(",");
	Serial.print(ki);
	Serial.print(",");
	Serial.println(kd);
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

			if (inputSep.size() >= 3)
			{
				kp = atof(inputSep[0].c_str());
				ki = atof(inputSep[1].c_str());
				kd = atof(inputSep[2].c_str());
			}
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
