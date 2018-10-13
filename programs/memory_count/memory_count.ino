#include <stdlib.h>


void setup() {
	Serial.begin(19200);
  // put your setup code here, to run once:
	int count = 0;
	void* ptr;

	while ((ptr = malloc(100)) != NULL)
	{
		count++;
	}
	Serial.println(count);

	//std::cout << "mallocs: " << count << std::endl;
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
