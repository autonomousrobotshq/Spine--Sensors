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
#include "SensorGPS.hpp"

/*
** SensorGPS
*/

SensorGPS::SensorGPS(	HardwareSerial& serial,
						const uint16_t baudrate,
						const uint16_t timeout,
						const unsigned long sampling_interval)
    : Sensor(sampling_interval)
    , _ss(serial)
	, _baudrate(baudrate)
	, _timeout(timeout)
{
}

SensorGPS::~SensorGPS()
{
}

bool SensorGPS::Init()
{
    _ss.begin(_baudrate);
	return (true);
}

bool SensorGPS::Update()
{
	bool error_occured = false;

    if (!_timer.Unlock())
		return (true);

    bool newData = false;
    for (unsigned long start = millis(); millis() - start < _timeout;) {
        while (_ss.available()) {
            char c = _ss.read();
            if (_gps.encode(c))
                newData = true;
        }
    }
    if (newData) {
        _gps.f_get_position(&_data._flat, &_data._flon, &_data._age);
        _gps.stats(&_data._chars, &_data._sentences, &_data._checksum);
        _gps.crack_datetime(&_data._year, &_data._month, &_data._day, &_data._hour,
            &_data._minute, &_data._second, &_data._hundredths, &_data._age);
        _data._kmph = _gps.f_speed_kmph();
        _data._course = _gps.f_course();
        if (_data._age == TinyGPS::GPS_INVALID_AGE) {
			_data._errno = SensorDataGPS::GPS_INVALID_AGE;
			error_occured = true;
        }
        if (_data._flat == TinyGPS::GPS_INVALID_F_ALTITUDE) {
			_data._errno = SensorDataGPS::GPS_INVALID_AGE;
			error_occured = true;
        }
        if (_data._kmph == TinyGPS::GPS_INVALID_F_SPEED) {
			_data._errno = SensorDataGPS::GPS_INVALID_F_SPEED;
			error_occured = true;
        }
        if (_data._course == TinyGPS::GPS_INVALID_F_ANGLE) {
			_data._errno = SensorDataGPS::GPS_INVALID_F_ANGLE;
			error_occured = true;
        }
#ifdef ROS
		if (_data.IsPublishingEnabled())
			_data.Publish();
#endif
    }
    return (error_occured == false);
}

/*
** SensorDataGPS
*/

SensorDataGPS &SensorGPS::RetreiveData()
{
	return (_data);
}

void SensorGPS::GetLocation(float* flat, float* flon) const
{
    *flat = _data._flat;
    *flon = _data._flon;
}

float SensorGPS::GetSpeed() const
{
    return (_data._kmph);
}

float SensorGPS::GetCourse() const
{
    return (_data._course);
}

void SensorGPS::GetTime(unsigned long* age, unsigned long* date, unsigned long* time) const
{
    *age = _data._age;
    *date = _data._date;
    *time = _data._time;
}

#ifdef ROS
void SensorDataGPS::Publish()
{
	this->GetLocation(&_msg_gps.lat, &_msg_gps.lon);
    PublishMsg(&_msg_gps);
}
#endif

void SensorDataGPS::GetLocation(float* flat, float* flon) const
{
    *flat = _flat;
    *flon = _flon;
}

float SensorDataGPS::GetSpeed() const
{
    return (_kmph);
}

float SensorDataGPS::GetCourse() const
{
    return (_course);
}

void SensorDataGPS::GetTime(unsigned long* age, unsigned long* date, unsigned long* time) const
{
    *age = _age;
    *date = _date;
    *time = _time;
}

