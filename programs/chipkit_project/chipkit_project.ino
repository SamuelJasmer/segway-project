/*
    Name:       chipkit_project.ino
    Created:	10/13/2018 12:27:48 PM
    Author:     AD\jasmersr
*/


void setup()
{
	pinMode(13, OUTPUT);

}

void loop()
{
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(500);

}
