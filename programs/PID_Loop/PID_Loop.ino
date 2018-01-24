#include "PID.h"
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

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

int set_point = 90;
float motor_point = 0;

PID pid(ANALOG_0);
float pid_speed = 0;
float speed = 0;

void setup() {
	Serial.begin(19200);
}

void loop() {
	kp = mapfloat(analogRead(ANALOG_1), 0, 1023, .0001, 10.00);
	ki = mapfloat(analogRead(ANALOG_2), 0, 1023, .0001, 10.00);
	kd = mapfloat(analogRead(ANALOG_3), 0, 1023, .0001, 10.00);

	pid.set_k_values(kp, ki, kd);

	pid_speed = pid.calculate_pid(set_point);
	pid_speed = map(pid_speed, -57, 237, 0, 128);

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

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) 
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
	Serial.print(pid.pid);
	Serial.print(",");
	Serial.print(kp);
	Serial.print(",");
	Serial.print(ki);
	Serial.print(",");
	Serial.println(kd);

}