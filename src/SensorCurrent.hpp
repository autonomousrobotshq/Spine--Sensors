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

#ifndef SENSOR_CURRENT_HPP
#define SENSOR_CURRENT_HPP

#include "Sensor.hpp"
#include "MedianFilter.hpp"

class SensorCurrentData : public SensorData {
	public:
		enum error {
			CURRENT_CAP_LOWER,
			CURRENT_CAP_UPPER
		};
		uint8_t GetCurrentAmps();
		uint16_t GetCurrentMilliAmps();
	private:
		friend class SensorCurrent;
		uint16_t _current = 0;
		uint16_t _lower_limit = 0;
		uint16_t _upper_limit = 0;

#ifdef ROS
	public:
		void Publish();
	private:
		//spine_msg::msg_current _current_msg;
#endif

};

class SensorCurrent : public Sensor {
public:
    SensorCurrent(const uint8_t analogPin, const uint8_t sample_count, const unsigned long sampling_interval);
    ~SensorCurrent();
	bool Init();
    bool Update();
	void SetMonitoringParameters(const uint16_t lower_limit, const uint16_t upper_limit);
	SensorCurrentData &RetreiveData();
private:
    const uint8_t _analogPin;
	MedianFilter<uint16_t> _filter;
	const uint8_t _sample_count;

    const static int _mVperAmp = 100; // use 185 for 5A Module, and 66 for 30A Module -> Trial by error testing.
    long  ReadReferenceVoltage();
    uint16_t ReadDCCurrent();
    float _vref = 0;

	SensorCurrentData _data;
};

#endif
