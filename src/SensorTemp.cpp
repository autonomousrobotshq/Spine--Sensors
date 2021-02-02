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

#include "SensorTemp.hpp"

SensorTemp::SensorTemp(const uint8_t pin, const unsigned long  sampling_interval)
    : Sensor(sampling_interval)
#ifndef ARDUINO_CI
    , _wire(pin)
    , _dallas(&_wire)
#endif
{
}

SensorTemp::~SensorTemp()
{
}

bool SensorTemp::Init()
{
#ifndef ARDUINO_CI
    _dallas.begin();
#endif
	return (true);
}

int16_t SensorTemp::GetTemp() const
{
    return (_data._celsius);
}

bool SensorTemp::Update()
{
	bool error_occured = false;

    if (!_timer.Unlock())
        return (true);

#ifndef ARDUINO_CI
    _dallas.requestTemperatures();
    _data._celsius = (int)_dallas.getTempCByIndex(0);
#endif

	if (_data._monitoring_enabled) {
		if (_data._celsius < _data._lower_limit) {
			_data._errno = SensorDataTemp::TEMP_CAP_LOWER;
			error_occured = true;
		} else if (_data._celsius < _data._lower_limit) {
			_data._errno = SensorDataTemp::TEMP_CAP_UPPER;
			error_occured = true;
		}
	}

    return (error_occured == false);
}

SensorDataTemp &SensorTemp::RetreiveData()
{
	return _data;
}

void SensorTemp::SetMonitoringParameters(const uint16_t lower_limit, const uint16_t upper_limit)
{
	_data._lower_limit = lower_limit;
	_data._upper_limit = upper_limit;
}

/*
 * SensorDataTemp
 */

SensorDataTemp::SensorDataTemp()
{
}

SensorDataTemp::~SensorDataTemp()
{
}

int16_t SensorDataTemp::GetTemp() const
{
    return (_celsius);
}

#ifdef ROS
void SensorDataTemp::Publish()
{

}
#endif
