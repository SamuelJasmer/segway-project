#include "PID.h"
#include "L298N.h"

#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5
#define ANALOG_0 0

L298N MotorA(ENA, IN_A1, IN_A2);
L298N MotorB(ENB, IN_B1, IN_B2);

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

int set_point = 90;
float motor_point = 0;

PID pid(kp, ki, kd, ANALOG_0);
float pid_speed = 0;
float speed = 0;

void setup() {
	Serial.begin(19200);
}

void loop() {
	//Serial.print(analogRead(0));
	pid_speed = pid.calculate_pid(set_point);
	//Serial.print(pid_speed);
	//Serial.print(",");
	pid_speed = map(pid_speed, -57, 237, 0, 256);
	//Serial.println(pid_speed);

	if(abs(pid.error) < 3) {
		MotorA.stop();
	}

	else {
		if(pid.get_error() > 0){
			MotorA.forward();
		
			MotorA.set_speed(pid_speed);	
		}
		else if(pid.get_error() < 0){
			MotorA.reverse();
			MotorA.set_speed(pid_speed);
		}	
		else {
			MotorA.stop();
		}
	}

	print_pid_data();

	
	
	//move_motor_B();
	//move_motor_A();

}

void move_motor_A() {

	int raw = analogRead(1);
	
	if(raw < 500){
		MotorA.reverse();
		int speed = map(raw, 500, 0, 0, 256);
		MotorA.set_speed(speed);
	}
	else if(raw > 524){
		MotorA.forward();
		int speed = map(raw, 524, 1024, 256, 0);
		MotorA.set_speed(speed);
	}
	else{
		MotorA.stop();
	}
}

void move_motor_B() {

	int raw = analogRead(1);
	
	if(raw < 500){
		MotorB.reverse();
		int speed = map(raw, 0, 500, 256, 0);
		MotorB.set_speed(speed);
	}
	else if(raw > 524){
		MotorB.forward();
		int speed = map(raw, 524, 1024, 0, 256);
		MotorB.set_speed(speed);
	}
	else{
		MotorB.stop();
	}
}

void print_pid_data(){
	Serial.print(pid.error);
	Serial.print(",");
	Serial.print(pid.p);
	Serial.print(",");
	Serial.print(pid.i);
	Serial.print(",");
	Serial.print(pid.d);
	Serial.print(",");
	Serial.println(pid.pid);

}

/*
void setMotor(int angle){
	myServo.write(angle);
}


int readPot(){

	int t_initial = micros();
	int potIn = analogRead(potPin);
	int t_final = micros();

	delta_t = (t_final - t_initial)/1000.0;
	
	return potIn;
}


int calculate_error(int potIn, int sp){
	
	int cp = map(potIn, 300, 840, 0, 180);
	Serial.print(cp);
	Serial.print(" ");
	
	int error = cp - sp;
	
	return error;

	
}


float calculate_pid(int error, float delta_t, float kp, float ki, float kd){

	float p = p_loop(error, kp);
	float i = i_loop(error, delta_t, ki);
	float d = d_loop(error, delta_t, kd);
	float pid = p + i+ d;
	
	return pid;
}

int calculate_output_angle(float pid, int sp){
	int output_angle = sp + pid;
	
	return output_angle;
}


int p_loop(int error, float kp){
	int p;
	p = kp*error;

		Serial.print(p);
		Serial.print(" ");
  
	return p;
}


float i_loop(int error, int delta_t, float ki){
	float error_integral;
	error_integral += ki*error*(delta_t);

		Serial.print(error_integral);
		Serial.print(" ");

	return error_integral;
}


float d_loop(int error, float delta_t, float kd){
	float error_initial = error;
	float error_derivative = kd*((error_final - error_initial) / delta_t);
	error_final = error_initial;

	Serial.print(error_derivative);
	Serial.print(" ");

	return error_derivative;
}

*/

