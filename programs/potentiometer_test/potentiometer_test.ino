#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Wire.h>

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

#define A0 0

float initial_angle = 0;
float initial_velocity = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!gyro.begin()) {
      Serial.println("There was a problem detecting the FXAS21002C ... check your connections");
      while (1);
    }
  if (!accelmag.begin(ACCEL_RANGE_4G)) {
      Serial.println("There was a problem detecting the FXOS8700 ... check your connections");
      while (1);
    }
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int t_initial = micros();
  float angle = angular_position();
  int t_final = micros();
  int delta_t = t_final - t_initial;

  //Serial.print(angle);
  //Serial.print(",");

  float velocity = angular_velocity(angle, delta_t);

  Serial.print(velocity);
  Serial.print(",");

  int acceleration = angular_acceleration(velocity, delta_t);

  Serial.print(acceleration);
  Serial.println();
}

float angular_position(){

  float angle_raw = analogRead(A0);
  float angle = map(angle_raw, 208, 886, 0 , 180);
  return angle;
}

float angular_velocity(float current_angle, int delta_t){

  float velocity = (current_angle - initial_angle) / delta_t;
  initial_angle = current_angle;
  return velocity;
  
}

float angular_acceleration(float current_velocity, int delta_t){
  
  float acceleration = (current_velocity - initial_velocity) / delta_t;
  initial_velocity = current_velocity;
  return acceleration;
}
