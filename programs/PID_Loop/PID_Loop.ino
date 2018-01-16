#include "PID.h"
#include "L298N.h"

#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5

L298N MotorA(ENA, IN_A1, IN_A2);
L298N MotorB(ENB, IN_B1, IN_B2);

void setup() {
	Serial.begin(9600);
	

}

void loop() {


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

