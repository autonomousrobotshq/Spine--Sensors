#include "SensorIMU.hpp"
#include "Vec3.hpp"

#define SAMPLE_COUNT 10 // take ten samples for navigation angle and use the median value
#define SAMPLING_INTERVAL 1000 // run every second

SensorIMU sensor(SAMPLE_COUNT, SAMPLING_INTERVAL);

void setup() {
	IMU::cal_t mag_cal = {0, 0, 0, 0, 0, 0};
	sensor.Init(mag_cal);
}

void loop() {
	sensor.Update();
	Vec3 acc_data = sensor.GetAccelerometerData();
	Vec3 mag_data = sensor.GetMagnetometerData();
	int16_t nav_angle = sensor.GetNavigationAngle();

	Serial.print("Acceleration -> ");
	Serial.print("x { "); Serial.print(acc_data.x); Serial.print(" }, ");
	Serial.print("y { "); Serial.print(acc_data.y); Serial.print(" }, ");
	Serial.print("z { "); Serial.print(acc_data.z); Serial.print(" } ");
	Serial.println();

	Serial.print("Orientation -> ");
	Serial.print("x { "); Serial.print(mag_data.x); Serial.print(" }, ");
	Serial.print("y { "); Serial.print(mag_data.y); Serial.print(" }, ");
	Serial.print("z { "); Serial.print(mag_data.z); Serial.print(" } ");
	Serial.println();

	Serial.print("Navigation angle : ");
	Serial.print(nav_angle);
	Serial.println();

	delay(200);
}
