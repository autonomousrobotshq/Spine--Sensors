#include "SensorTemp.hpp"

#define PIN 10
#define SAMPLING_INTERVAL 1000 // update every second

SensorTemp sensor(PIN, SAMPLING_INTERVAL);

void setup() {
	sensor.Init();
}

void loop() {
	sensor.Update();
	int8_t temp = sensor.GetTemp();

	Serial.print("Temperature : ");
	Serial.println(temp);

	delay(200);
}
