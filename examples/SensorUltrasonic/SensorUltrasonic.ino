#include "SensorUltrasonic.hpp"

#define PIN 10
#define MAX_DEPTH 520 // 520 cm is the rated maximum distance of the ultrasonic sensor's range
#define SAMPLE_COUNT 10 // take ten samples and use the median value
#define SAMPLING_INTERVAL 1000 // run every second

SensorUltrasonic sensor(PIN, MAX_DEPTH, SAMPLE_COUNT, SAMPLING_INTERVAL);

void setup() {
	sensor.Init();
}

void loop() {
	sensor.Update();
	int16_t distance = sensor.GetDistance();

	Serial.print("Distance : ");
	Serial.println(distance);

	delay(200);
}
