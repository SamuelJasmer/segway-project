#define ENA		 10
#define IN_A1	 9
#define IN_A2	 8
#define IN_B2	 7
#define IN_B1	 6
#define ENB		 5

void setup()
{
	Serial.begin(9600);
	pinMode(ENA,	OUTPUT);
	pinMode(IN_A1,	OUTPUT);
	pinMode(IN_A2,	OUTPUT);
	pinMode(IN_B2,	OUTPUT);
	pinMode(IN_B1,	OUTPUT);
	pinMode(ENB,	OUTPUT);

	forward(IN_A1, IN_A2);

}

void loop()
{
	increment_up(ENA);
	delay(100);
	increment_down(ENA);
	delay(100);
}

void increment_up(int enable) {
	for (int i = 0; i < 255; i++) {
		analogWrite(enable, i);
		delay(100);
	}
}

void increment_down(int enable) {
	for (int i = 255; i > 0; i--) {
		analogWrite(enable, i);
		delay(100);
	}
}

void forward(int input1, int input2) {
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
}

void reverse(int input1, int input2) {
	digitalWrite(input2, HIGH);
	digitalWrite(input1, LOW);
}

void stop(int input1, int input2) {
	digitalWrite(input2, HIGH);
	digitalWrite(input1, HIGH);
}