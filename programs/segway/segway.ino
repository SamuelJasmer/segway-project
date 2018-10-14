/*
    Name:       segway.ino
    Created:	6/5/2018 9:47:06 PM
    Author:     AD\jasmersr
*/

#include "PID.h"
#include "L298N.h"
#include "Orientation_Library.h"
#include "matrix.h"
#include "vector3.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_FXOS8700.h"
#include "Adafruit_FXAS21002C.h"
#include "Wire.h"
#include <stdlib.h>

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

//Instantiate Orientation:
Orientation segway_orientation;

//Instantiate filter:
filter accelerometer_filter;
filter magnetometer_filter;

//Unit Vectors:
vector i_vector;
vector j_vector;
vector k_vector;

//Acceleration Vectors:
vector acceleration;
vector accel_angles;
vector accel_smoothed;
vector sample_mean;
vector sample_variance;
vector covariance;

//Angular Velocity Vectors:
vector angular_velocity;
vector angular_displacement;
vector zero_point;

//Magnetic Vectors:
vector magnetism;
vector mag_angles;

//Sensor Smoothing Vectors:
const int n = 4; //size of buffers

//Magnetic Calibration Matricies:
Matrix magnetic_transformation_matrix;
Matrix magnetic_offset_matrix;
Matrix magnetometer_raw;
Matrix magnetometer_calibrated;

//Instantiate PID Object:
PID pid;

float kp = 2.5;
float ki = 0.0;
float kd = 0.0;

int set_point = 90;
float pid_speed = 0;

int t_initial = 0;
int delta_t = 10;

//Setup Serial Plotter:
//std::vector<char>* serialInputBuf;
void checkSerial();
//void parseCSV(std::string input, std::vector<std::string>* argv);

void setup() {

	Serial.begin(19200);
	while(!Serial);
	Serial.println(">>> Begin Setup");
	//serialInputBuf = new std::vector<char>();

	//Setup Segway:
	segway_orientation.init();
	accelerometer_filter.init(n);

	float i[]{ 1,0,0 };
	float j[]{ 0,1,0 };
	float k[]{ 0,0,1 };

	i_vector.set_vector(i);
	j_vector.set_vector(j);
	k_vector.set_vector(k);

	//Calibrate Magnetometer
	//setup_magnetometer();

	//Run Gyroscope Calibration:
	//Serial.println(F("Calibrating Gyro"));
	//calibrate_gyro();



	Serial.println(">>> End Setup");
}

void loop() {

	while (millis() - t_initial < delta_t);

	t_initial = millis();
	//checkSerial();

	//angular_velocity = segway_orientation.measure_gyro();


	acceleration = calculate_acceleration();

	accel_angles = get_accel_angles(acceleration);
	//Serial.println(accel_angles.x);
	//accel_averaged_angles = accelerometer_filter.sample_mean(accel_angles, n);
	//accel_smoothed = accelerometer_filter.least_squares_regression(accel_angles, n);
	//accel_smoothed = accelerometer_filter.least_squares_regression(accel_smoothed, n);

	//accel_angles = accelerometer_filter.lowess_smooth(accel_angles, n);
	sample_mean = accelerometer_filter.sample_mean(accel_angles, n);
	sample_variance = accelerometer_filter.sample_variance(sample_mean, accel_angles, n);
	//covariance = accelerometer_filter.covariance(sample_mean, accel_angles, n);

	Serial.print(accel_angles.x);
	Serial.print(",");
	//Serial.print(accel_angles.y);
	//Serial.print(",");
	//Serial.print(accel_angles.z);
	//Serial.print(",");

	///Serial.print(sample_mean.x);
	///Serial.print(",");
	///Serial.print(sample_mean.y);
	///Serial.print(",");
	///Serial.print(sample_mean.z);

	Serial.print(sample_variance.x);
	//Serial.print(",");
	//Serial.print(sample_variance.y);
	//Serial.print(",");
	//Serial.print(sample_variance.z);

	///Serial.print(covariance.x);
	///Serial.print(",");
	///Serial.print(covariance.y);
	///Serial.print(",");
	///Serial.print(covariance.z);

	Serial.println();

	//accel_angles_variance = accelerometer_filter.sample_variance(accel_averaged_angles, accel_angles, accel_angles_variance_buffer, n);

	//magnetism = calculate_magnetism();
	//mag_angles = get_mag_angles(magnetism);
	//mag_averaged_angles = magnetometer_filter.sample_mean(mag_angles, n);
	//mag_angles_variance = magnetometer_filter.sample_variance(mag_averaged_angles, mag_angles, mag_angles_variance_buffer, n);

	//angles = accelerometer_filter.CLT(accel_angles_variance, mag_angles_variance, accel_angles, mag_angles);
	//angles.x = (accel_averaged_angles.x + mag_averaged_angles.x) / 2;
	//angles.y = (accel_averaged_angles.y + mag_averaged_angles.y) / 2;
	//angles.x = (accel_averaged_angles.z + mag_averaged_angles.z) / 2;

	//Serial.print(angles.y);
//
	//Serial.print(",");
//
	//Serial.print(accel_angles.y);
	//Serial.print(",");
	//Serial.print(accel_averaged_angles.y);
	//Serial.print(",");
	//Serial.print(accel_angles_variance.y);
	//Serial.print(",");
//
	//Serial.print(mag_angles.y);
	//Serial.print(",");
	//Serial.print(mag_averaged_angles.y);
	//Serial.print(",");
	//Serial.print(mag_angles_variance.y);
//
	//Serial.println();


	/*
	pid.set_k_values(kp, ki, kd);
	pid_speed = abs(pid.calculate_pid(angles.y, set_point));

	Serial.println(pid_speed);

	if (pid_speed == 0) {
		//do nothing, i.e.:stop moving
		MotorA.stop();
	}
	else if (pid_speed > 255) {
		//limit upper bound of pid speed
		pid_speed = 255;
	}
	else if (pid_speed < 80) {
		//limit lower bound of pid speed
		pid_speed = 0;
	}
	if (angles.y > set_point) {
		MotorA.forward();
		MotorA.set_speed(pid_speed);
	}
	else if (angles.y < set_point) {
		MotorA.reverse();
		MotorA.set_speed(pid_speed);
	}
	else {
		//do nothing, i.e.:stop moving
		MotorA.stop();
	}
	*/


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

void setup_magnetometer() {

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

vector calculate_acceleration() {
	vector acceleration;

	acceleration = segway_orientation.measure_accelerometer();

	return acceleration;
}

vector get_accel_angles(vector acceleration) {
	vector angles;
	angles.x = acceleration.angle_between_vectors(i_vector);
	angles.y = acceleration.angle_between_vectors(j_vector);
	angles.z = acceleration.angle_between_vectors(k_vector);

	angles.convert_to_degrees();

	return angles;
}

vector calculate_magnetism() {
	vector magnetism;
	magnetism = segway_orientation.measure_magnetometer();
	calibrate_magnetometer(magnetism);
	magnetism.x = magnetometer_calibrated[0][0];
	magnetism.y = magnetometer_calibrated[1][0];
	magnetism.z = magnetometer_calibrated[2][0];

	return magnetism;
}

void calibrate_magnetometer(vector magnetism) {
	magnetometer_raw.set(0, 0, magnetism.x);
	magnetometer_raw.set(1, 0, magnetism.y);
	magnetometer_raw.set(2, 0, magnetism.z);

	magnetometer_calibrated = magnetic_transformation_matrix * (magnetometer_raw - magnetic_offset_matrix);
}

vector get_mag_angles(vector magnetism) {
	vector angles;
	angles.x = magnetism.angle_between_vectors(i_vector);
	angles.y = magnetism.angle_between_vectors(j_vector);
	angles.z = magnetism.angle_between_vectors(k_vector);

	angles.convert_to_degrees();

	return angles;
}

void calculate_angular_motion() {
	vector angular_velocity = segway_orientation.measure_gyro();
	vector angular_displacement = segway_orientation.integrate_vector(angular_velocity);
	angular_displacement.convert_to_degrees();
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

/*
void print_pid_data() {
	Serial.print(angles.y);
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
*/

void checkSerial()
{
	//char c;
	//while (Serial.available() > 0)
	//{
	//	c = (char)Serial.read();
	//	if (c >= 0x20 && c < 0x7f)
	//	{
	//		serialInputBuf->push_back(c);
	//	}
	//	else if (c == '\n')
	//	{
	//		std::vector<std::string> inputSep;
	//		std::string input(serialInputBuf->begin(), serialInputBuf->end());
	//		serialInputBuf->clear();
	//
	//
	//		parseCSV(input, &inputSep);

	//		/*
	//		if (inputSep.size() >= 3)
	//		{
	//			kp = atof(inputSep[0].c_str());
	//			ki = atof(inputSep[1].c_str());
	//			kd = atof(inputSep[2].c_str());
	//		}
	//		*/
	//
	//	}
	//}
}

/*
void parseCSV(std::string input, std::vector<std::string>* argv)
{
	/*std::string current = "";
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
	argv->push_back(current);*/
//}*/
