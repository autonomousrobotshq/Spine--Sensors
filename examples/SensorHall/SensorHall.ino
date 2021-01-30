#include "SensorHall.hpp"

#define INTERRUPT_PIN 10
#define INTERRUPT_INDEX 10
#define COUNTS_PER_REVOLUTION 50 // test using Tachometer
#define DISTANCE_PER_REVOLUTION 50 // test using Tachometer

SensorHall sensor(INTERRUPT_PIN, INTERRUPT_INDEX, COUNTS_PER_REVOLUTION, DISTANCE_PER_REVOLUTION);

void setup() {
	sensor.Init();
}

void loop() {
	sensor.CounterReset();
	delay(100);
	uint16_t distance = sensor.CounterGetDistance();

	Serial.print("Distance : ");
	Serial.println(distance);

	delay(200);
}
