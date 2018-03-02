#include "PID.h"
#include "L298N.h"
#include <ArduinoSTL.h>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

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

std::vector<char>* serialInputBuf;
void checkSerial();
void parseCSV(std::string input, std::vector<std::string>* argv);

void setup() {
	Serial.begin(19200);
	serialInputBuf = new std::vector<char>();
}

void loop() {

	checkSerial();

	pid.set_k_values(kp, ki, kd);

	pid_speed = pid.calculate_pid(set_point);
	//pid_speed = map(pid_speed, 0, 512, 64, 256);

	if(pid.error < 2) {
		MotorA.stop();
	}

	else {
		if(pid.cp > set_point){
			if(pid_speed < 64){
				pid_speed = 64;
			}
			else{
				pid_speed = pid_speed;
			}

			MotorA.forward();
			MotorA.set_speed(pid_speed);	
		}
		else if(pid.cp < set_point){
			if(pid_speed < 64){
				pid_speed = 64;
			}
			else{
				pid_speed = pid_speed;
			}
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
	Serial.print(pid.cp);
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
	//Serial.println(",");
	//Serial.println(analogRead(0));

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
