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
#include "Sensor.hpp"

Sensor::Sensor()
	: _timer(0)
{
}

Sensor::Sensor(const unsigned long sampling_interval)
    : _timer(sampling_interval)
{
}

Sensor::~Sensor()
{
}

bool Sensor::Init()
{
	return (true);
}

bool Sensor::Update()
{
	return (true);
}

SensorData	&Sensor::RetreiveData()
{
	return (_data);
}
