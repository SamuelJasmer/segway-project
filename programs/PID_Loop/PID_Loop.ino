#include <Servo.h>
#include <PID.h>
#include <L298N.h>

Servo myServo;

int servoPin = 9;
int potPin = 0;

int sp = 90;
float mp = 0.0;
float error_final = 0;
float delta_t = 0;

float kp = -1.0;
float ki = 0.0;
float kd = 0.0;

void setup() {
	Serial.begin(9600);
	pinMode(servoPin, OUTPUT);
	myServo.attach(servoPin);
	myServo.write(sp);
}

void loop() {

	/*
	int pot_in = readPot();
		//Serial.print(delta_t);
		//Serial.print(" ");
		//Serial.print(pot_in);
		//Serial.print(" ");

	int error = calculate_error(pot_in, sp);
		Serial.print(error);
		Serial.print(" ");

	float pid = calculate_pid(error, delta_t, kp, ki, kd);
		Serial.print(pid);
		Serial.print(" ");

	int output_angle = calculate_output_angle(pid, sp);
		Serial.print(output_angle);
		Serial.println();

	setMotor(output_angle);
	*/	
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

