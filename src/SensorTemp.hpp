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

#ifndef SENSOR_TEMPERATURE_HPP
#define SENSOR_TEMPERATURE_HPP

#ifndef ARDUINO_CI
# include <DallasTemperature.h>
# include <OneWire.h>
#endif

#include "Sensor.hpp"

class SensorDataTemp : public SensorData 
{
	public:
		enum error {
			TEMP_CAP_LOWER,
			TEMP_CAP_UPPER
		};
    	int16_t GetTemp();
	private:
		friend class SensorTemp;
    	int16_t _celsius = 0;
		int16_t _lower_limit = 0;
		int16_t _upper_limit = 0;

#ifdef ROS
    public:
        void Publish();
    private:
        //spine_msg::msg_temp _temp_msg;
#endif

};

class SensorTemp : public Sensor {
public:
    SensorTemp(const uint8_t pin, const unsigned long sampling_interval);
    ~SensorTemp();
	bool Init();
    bool Update();
	SensorDataTemp &RetreiveData();
	void SetMonitoringParameters(const uint16_t lower_limit, const uint16_t upper_limit);
    int16_t GetTemp();

private:

#ifndef ARDUINO_CI
    OneWire _wire;
    DallasTemperature _dallas;
#endif

	SensorDataTemp _data;
};

#endif
