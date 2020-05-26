/*
    Name:       segway.ino
    Created:	6/5/2018 9:47:06 PM
    Author:     AD\jasmersr
*/

//#include <Adafruit_SensorLab.h>
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
#include "filter2.h"

#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5

#define ENABLE_Sensor 1
#define ENABLE_Potentiometer 0

L298N MotorB(ENB, IN_B1, IN_B2);
L298N MotorA(ENA, IN_A1, IN_A2);

//Instantiate Orientation:
Orientation segway_orientation;

//Instantiate filter:
filter2 accelerometer_filter;
filter2 magnetometer_filter;

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

//Size of buffers
const int n = 10; 

//Instantiate PID Object:
PID pid;

float initial_angle_pot = 0;
float initial_velocity_pot = 0;

float kp = 2.0;
float ki = 0.0;
float kd = 0.0;

int set_point = 90;
float pid_speed = 0;

int t_initial = 0;

//Setup Serial Plotter:
//std::vector<char>* serialInputBuf;
//void checkSerial();
//void parseCSV(std::string input, std::vector<std::string>* argv);

void setup() {

	Serial.begin(19200);
	while(!Serial);
	Serial.println(">>> Begin Setup");
	//serialInputBuf = new std::vector<char>();

	//Setup Segway:
#if ENABLE_Sensor == 1

	Serial.println("Sensor Enabled");
	segway_orientation.init(0,1);

	accelerometer_filter.init(n);
	magnetometer_filter.init(n);



	

	//Calibrate Magnetometer
	//segway_orientation.setup_magnetometer();

	//Serial.println(">>> Start Accelerometer Calibration");
	//calibrate_acclerometer();

#elif ENABLE_Sensor == 0
	Serial.println("Sensor Disabled");
#endif

	//Serial.println(">>> Sample Data?");
	//delay(1000);

	//button_wait();

	//sample_data();


	//Run Gyroscope Calibration:
	//Serial.println(F("Calibrating Gyro"));
	//calibrate_gyro();

	//MotorA.forward();
	//MotorA.set_speed(255);

	//MotorB.forward();
	//MotorB.set_speed(255);
}

void loop() {

#if ENABLE_Potentiometer == 1
	int t_initial = micros();
	float angle_pot = angular_position_pot();
	int t_final = micros();
	int delta_t = t_final - t_initial;

	Serial.print(angle_pot);
	Serial.print(",");

	float velocity_pot = angular_velocity_pot(angle_pot, delta_t);

	Serial.print(velocity_pot);
	Serial.print(",");

	int acceleration_pot = angular_acceleration_pot(velocity_pot, delta_t);

	Serial.print(acceleration_pot);
	Serial.println();
#elif ENABLE_Potentiometer == 0

#endif

#if ENABLE_Sensor == 1
	//ACCELERATION CALCLUATIONS
	vector acceleration = segway_orientation.measure_accelerometer();
	accel_angles = segway_orientation.get_accel_angles(acceleration);
	accelerometer_filter.add_sample(accel_angles);
	accel_angles = accelerometer_filter.sample_mean();

	//MAGNETISM CALCULATIONS
	//vector magnet = segway_orientation.measure_magnetometer();
	//mag_angles = segway_orientation.get_mag_angles(magnet);
	//magnetometer_filter.add_sample(mag_angles);
	//mag_angles = magnetometer_filter.sample_mean();

	//PRINTS
	if (millis() % 10 == 0) {
		#define Print_Acceleration 0
		#if Print_Acceleration == 1
			Serial.print(acceleration.x);
			Serial.print(",");
			Serial.print(acceleration.y);
			Serial.print(",");
			Serial.print(acceleration.z);
		#elif Print_Acceleration == 0
		
		#endif
		
		#define Print_Accel_Angles 0
		#if Print_Accel_Angles == 1
			Serial.print(accel_angles.x);
			Serial.print(",");
			Serial.print(accel_angles.y);
			Serial.print(",");
			Serial.print(accel_angles.z);
		#elif Print_Accel_Angles == 0

		#endif

		//Serial.println();
	}

	run_PID(accel_angles.y);


#elif ENABLE_Sensor == 0

#endif
	
	
}

//

float angular_position_pot() {

	float angle_raw = analogRead(A0);
	float angle = map(angle_raw, 710, 3285, 0, 180);
	return angle;
}

float angular_velocity_pot(float current_angle, int delta_t) {

	float velocity = (current_angle - initial_angle_pot) / delta_t;
	initial_angle_pot = current_angle;
	return velocity;
}

float angular_acceleration_pot(float current_velocity, int delta_t) {

	float acceleration = (current_velocity - initial_velocity_pot) / delta_t;
	initial_velocity_pot = current_velocity;
	return acceleration;
}

void pot_motor_control(float speed) {

	MotorA.forward();
	MotorA.set_speed(speed);
}

void button_wait() {
	while (digitalRead(PIN_BTN1) == HIGH) {
		//do nothing
	}
	while (digitalRead(PIN_BTN1) == LOW) {
		//do nothing
	}
}

/*
 *
 */

void run_PID(float angle) {
	pid.set_k_values(kp, ki, kd);
	pid_speed = abs(pid.calculate_pid(angle, set_point, segway_orientation.delta_t));


	Serial.print(segway_orientation.delta_t);
	Serial.print(pid_speed);
	Serial.println();

	if (pid_speed == 0) {
		//do nothing, i.e.:stop moving
		MotorB.stop();
	}
	else if (pid_speed > 255) {
		//limit upper bound of pid speed
		pid_speed = 255;
	}
	else if (pid_speed < 80) {
		//limit lower bound of pid speed
		pid_speed = 0;
	}
	if (accelerometer_filter.sample_mean().y > set_point) {
		MotorB.forward();
		MotorB.set_speed(pid_speed);
	}
	else if (accelerometer_filter.sample_mean().y < set_point) {
		MotorB.reverse();
		MotorB.set_speed(pid_speed);
	}
	else {
		//do nothing, i.e.:stop moving
		MotorB.stop();
	}



	//vector angular_velocity = segway_orientation.measure_gyro();
	//vector angular_acceleration = segway_orientation.angular_acceleration();
	//vector acceleration = segway_orientation.measure_accelerometer();

	//angular_velocity.x = angular_velocity.x - zero_point.x;
	//angular_velocity.y = angular_velocity.y - zero_point.y;
	//angular_velocity.z = angular_velocity.z - zero_point.z;

	//vector theta = segway_orientation.integrate_vector(angular_velocity);

	//vector smoothed_angular_velocity = segway_orientation.smooth_gyro(angular_velocity);

}

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

/*
void checkSerial()
{
	char c;

	while (Serial.available() > 0) {
		c = (char)Serial.read();
		if (c >= 0x20 && c < 0x7f) {
			serialInputBuf->push_back(c);
		}
		else if (c == '\n') {
			std::vector<std::string> inputSep;
			std::string input(serialInputBuf->begin(), serialInputBuf->end());
			serialInputBuf->clear();
	
	
			parseCSV(input, &inputSep);

			
			if (inputSep.size() >= 3) {
				kp = atof(inputSep[0].c_str());
				ki = atof(inputSep[1].c_str());
				kd = atof(inputSep[2].c_str());
			}
		}
	}
}
*/

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
