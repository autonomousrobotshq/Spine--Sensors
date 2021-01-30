//
// Spine - Spine - MCU code for robotics.
// Copyright (C) 2019-2021 Codam Robotics
//
// This file is part of Spine.
//
// Spine is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Spine is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Spine.  If not, see <http://www.gnu.org/licenses/>.
//

#include <Arduino.h>
#include <ArduinoUnitTests.h>

#include "SensorUltrasonic.hpp"

unittest(Ultrasonic_min)
{
	GodmodeState* state = GODMODE();   // get access to the state

	const int pin = 10;
	const int max_depth = 520;
	const int sample_count = 10;
	const int sampling_interval = 0;

	SensorUltrasonic sensor(pin, max_depth, sample_count, sampling_interval);
	assertTrue(sensor.Init());

	analogWrite(pin, 0);
	assertTrue(sensor.Update());
	assertEqual(0, sensor.GetDistance());
}

unittest(Ultrasonic_max)
{
	GodmodeState* state = GODMODE();   // get access to the state

	const int pin = 10;
	const int max_depth = 520;
	const int sample_count = 10;
	const int sampling_interval = 0;

	SensorUltrasonic sensor(pin, max_depth, sample_count, sampling_interval);
	assertTrue(sensor.Init());

	analogWrite(pin, 255);
	assertTrue(sensor.Update());
	assertEqual(max_depth, sensor.GetDistance());
}

unittest_main()
