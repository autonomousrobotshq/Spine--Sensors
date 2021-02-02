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

#include <Wire.h>
#include "SensorIMU.hpp"

/*
** SensorDataIMU
*/

int16_t SensorDataIMU::GetNavigationAngle() const
{
    return (_navigation_angle);
}

Vec3<int16_t> SensorDataIMU::GetAccelerometerData() const
{
    return (Vec3<int16_t>(_accelero.x, _accelero.y, _accelero.z));
}

void SensorDataIMU::GetAccelerometerData(int16_t* x, int16_t* y, int16_t* z) const
{
    *x = _accelero.x;
    *y = _accelero.y;
    *z = _accelero.z;
}

Vec3<int16_t> SensorDataIMU::GetMagnetometerData() const
{
    return (Vec3<int16_t>(_magneto.x, _magneto.y, _magneto.z));
}

void SensorDataIMU::GetMagnetometerData(int16_t* x, int16_t* y, int16_t* z) const
{
    *x = _magneto.x;
    *y = _magneto.y;
    *z = _magneto.z;
}

#ifdef ROS
void SensorDataIMU::Publish()
{
	this->GetMagnetometerData(&_msg_imu.mag_x, &_msg_imu.mag_y, &_msg_imu.mag_z);
	this->GetAccelerometerData(&_msg_imu.accel_x, &_msg_imu.accel_y, &_msg_imu.accel_z);
	_msg_imu.angle = this->GetNavigationAngle();
	PublishMsg(&_msg_imu);
}
#endif

/*
** SensorIMU
*/

SensorIMU::SensorIMU(const uint16_t sample_count, const unsigned long sampling_interval)
    : Sensor(sampling_interval)
    , _filter(sample_count)
	, _sample_count(sample_count)
{
}

SensorIMU::~SensorIMU()
{
}

bool SensorIMU::Init()
{
    Wire.begin();
    if (!_compass.init()) {
		return (false);
    }
    _compass.enableDefault();
	return (true);
}

bool SensorIMU::Update()
{
	bool error_occured = false;

    if (!_timer.Unlock())
        return (true);
    _filter.Reset();
    for (uint8_t i = 0; i < _sample_count; i++) {
        _compass.read();
        _filter.NewReading(_compass.heading());
    }
    _data._navigation_angle = _filter.GetFilteredSignal();
	_data._magneto = this->GetMagnetometerData();
	_data._accelero = this->GetAccelerometerData();

	// should a filtered Vec3<int16_t> object also be presented?

	if (_data._monitoring_enabled) {
		if (abs(_data._accelero.x) > _data._max_acceleration.x
			|| abs(_data._accelero.y) > _data._max_acceleration.y
			|| abs(_data._accelero.z) > _data._max_acceleration.z) {
			_data._errno = SensorDataIMU::ACCELERATION_CAP;
			error_occured = true;
		}
		else if (_data._magneto.x > _data._max_magneto.x
			|| _data._magneto.y > _data._max_magneto.y
			|| _data._magneto.z > _data._max_magneto.z) {
			_data._errno = SensorDataIMU::MAGNETO_CAP_UPPER;
			error_occured = true;
		}
		else if (_data._magneto.x < _data._min_magneto.x
			|| _data._magneto.y < _data._min_magneto.y
			|| _data._magneto.z < _data._min_magneto.z) {
			_data._errno = SensorDataIMU::MAGNETO_CAP_LOWER;
			error_occured = true;
		}
	}

#ifdef ROS
	if (_data.IsPublishingEnabled())
		_data.Publish();
#endif

    return (error_occured == false);
}

SensorDataIMU &SensorIMU::RetreiveData()
{
	return (_data);
}

void SensorIMU::SetCalibrationParameters(const IMU::cal_t &mag_cal)
{
    _compass.m_min = (LSM303::vector<int16_t>) { mag_cal.x_min, mag_cal.y_min, mag_cal.z_min };
    _compass.m_max = (LSM303::vector<int16_t>) { mag_cal.x_max, mag_cal.y_max, mag_cal.z_max };
}

void SensorIMU::SetMonitoringParameters(const Vec3<uint16_t> &max_acceleration, const Vec3<int16_t> &min_magneto, const Vec3<int16_t> &max_magneto)
{
	_data._max_acceleration = max_acceleration;
	_data._min_magneto = min_magneto;
	_data._max_magneto = max_magneto;
}

int16_t SensorIMU::GetNavigationAngle() const
{
    return (_data._navigation_angle);
}

Vec3<int16_t> SensorIMU::GetAccelerometerData() const
{
    return (Vec3<int16_t>(_compass.a.x, _compass.a.y, _compass.a.z));
}

void SensorIMU::GetAccelerometerData(int16_t* x, int16_t* y, int16_t* z) const
{
    *x = _compass.a.x;
    *y = _compass.a.y;
    *z = _compass.a.z;
}

Vec3<int16_t> SensorIMU::GetMagnetometerData() const
{
    return (Vec3<int16_t>(_compass.m.x, _compass.m.y, _compass.m.z));
}

void SensorIMU::GetMagnetometerData(int16_t* x, int16_t* y, int16_t* z) const
{
    *x = _compass.m.x;
    *y = _compass.m.y;
    *z = _compass.m.z;
}
