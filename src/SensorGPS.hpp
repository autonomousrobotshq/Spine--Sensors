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

#ifndef SENSOR_GPS_HPP
#define SENSOR_GPS_HPP

// TinyGPS documentation:
// https://github.com/neosarchizo/TinyGPS/blob/master/examples/test_with_gps_device/test_with_gps_device.ino

#include <SoftwareSerial.h>
#include <TinyGPS.h>

#ifdef ROS
#include "spine_msg/msg_gps.h"
#endif

#include <Sensor.hpp>

class SensorDataGPS : public SensorData {
	public:
		enum errors {
			GPS_INVALID_AGE,
			GPS_INVALID_F_ALTITUDE,
			GPS_INVALID_F_SPEED,
			GPS_INVALID_F_ANGLE
		};
    	void GetLocation(float* flat, float* flon);
    	void GetTime(unsigned long* age, unsigned long* date, unsigned long* time);
    	float GetSpeed();
    	float GetCourse();
	private:
		friend class SensorGPS;
    	float _flat,
    	    _flon,
    	    _kmph,
    	    _course;
    	unsigned long _age,
    	    _date,
    	    _time,
    	    _chars = 0;
    	unsigned short _sentences = 0,
    	               _checksum = 0;
    	int _year;
    	uint8_t _month,
    	    _day,
    	    _hour,
    	    _minute,
    	    _second,
    	    _hundredths;
#ifdef ROS
	public:
		void Publish();
	private:
		spine_msg::msg_gps _msg_gps;
#endif
};

class SensorGPS : public Sensor {
public:
    SensorGPS(	HardwareSerial& serial,
				const uint16_t baudrate,
				const uint16_t timeout,
				const unsigned long sampling_interval);
    ~SensorGPS();
	bool Init();
    bool Update();
	SensorDataGPS &RetreiveData();
    void GetLocation(float* flat, float* flon);
    void GetTime(unsigned long* age, unsigned long* date, unsigned long* time);
    float GetSpeed();
    float GetCourse();
private:
    HardwareSerial& _ss;
    TinyGPS _gps;
	const uint16_t _baudrate, _timeout;

	SensorDataGPS _data;
};

#endif
