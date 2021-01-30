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

#ifndef SENSOR_ULTRASONIC_HPP
#define SENSOR_ULTRASONIC_HPP

#ifdef ROS
# include "ros.h"
#endif

#include <Arduino.h>
#include "Sensor.hpp"
#include "MedianFilter.hpp"

class SensorDataUltrasonic : public SensorData
{
	public:
		SensorDataUltrasonic(const uint16_t max_depth);
		enum error {
			DISTANCE_CAP_LOWER,
			DISTANCE_CAP_UPPER
		};
		uint16_t GetDistance();
	private:
		friend class SensorUltrasonic;
    	uint16_t _distance;
		const uint16_t _max_depth;
		uint16_t _lower_limit;
		uint16_t _upper_limit;

#ifdef ROS                                                                      
    public:                                                                     
        void Publish();                                                         
    private:                                                                    
        //spine_msg::msg_current _sonar_msg;                                  
#endif

};

class SensorUltrasonic : public Sensor {
public:
    SensorUltrasonic(const uint8_t pin, const uint16_t max_depth, const uint16_t sample_count, const unsigned long sampling_interval);
    ~SensorUltrasonic();
	bool Init();
    bool Update();
	SensorDataUltrasonic &RetreiveData();
	void SetMonitoringParameters(const uint16_t lower_limit, const uint16_t upper_limit);
	
    /*!
	**	@brief Gets calculated distance from sensor.
	**	@return int16_t calculated distance in centimeters.
	*/
    uint16_t GetDistance();

private:
    const uint8_t _analog_pin;
	MedianFilter<uint16_t> _filter;
	const uint16_t _sampling_count;
	SensorDataUltrasonic _data;
};

#endif
