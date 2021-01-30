#include "SensorGPS.hpp"

#define HW_SERIAL Serial2
#define BAUDRATE 9600
#define TIMEOUT 1000 // sensor will timeout after 1 second trying to get a GPS reading 
#define SAMPLING_INTERVAL 1000 // run every second

SensorGPS sensor(HW_SERIAL, BAUDRATE, TIMEOUT, SAMPLING_INTERVAL);

void setup() {
	Serial.begin(115200);
	sensor.Init();
}

void loop() {
	sensor.Update();
	float flat, flon;
	unsigned long age, date, time;
	float speed, course;

	sensor.GetLocation(&flat, &flon);
	sensor.GetTime(&age, &date, &time);
	speed = sensor.GetSpeed();
	course = sensor.GetCourse();
	Serial.print("GPS Location -> ");
	Serial.print("LAT { "); Serial.print(flat); Serial.print(" }, ");
	Serial.print("LON { "); Serial.print(flon); Serial.print(" }.");
	Serial.println();

	Serial.print("GPS Time -> ");
	Serial.print("age { "); Serial.print(age); Serial.print(" }, ");
	Serial.print("date { "); Serial.print(date); Serial.print(" }, ");
	Serial.print("time { "); Serial.print(time); Serial.print(" }.");
	Serial.println();

	Serial.print("GPS Movement -> ");
	Serial.print("speed { "); Serial.print(speed); Serial.print(" }, ");
	Serial.print("course { "); Serial.print(course); Serial.print(" }.");
	Serial.println();

	delay(200);
}
