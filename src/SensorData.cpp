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

#include "Arduino.h"
#include "SensorData.hpp"

#ifndef ROS 

SensorData::SensorData()
{
}

SensorData::~SensorData()
{
}

#else

SensorData::SensorData()
{
}

SensorData::~SensorData()
{
	delete _pub;
}
SensorData::SensorData(const char *topic_name, ros::Msg *msg)
	: _pub(new ros::Publisher(topic_name, msg))
{
}

void	SensorData::Publish()
{
}

void	SensorData::PublishMsg(const ros::Msg * msg)
{
	_pub->publish(msg);
}

ros::Publisher *SensorData::GetPublisher()
{
	return (_pub);
}

void SensorData::SetPublisher(const char *topic_name, ros::Msg *msg)
{
	_pub = new ros::Publisher(topic_name, msg);
}

void	SensorData::EnablePublishing()
{
	_publishing_enabled = true;
}

void	SensorData::DisablePublishing()
{
	_publishing_enabled = true;
}

bool	SensorData::IsPublishingEnabled()
{
	return (_publishing_enabled);
}

#endif

void	SensorData::UpdateTimestamps()
{
	_time_since_last_execution = millis() - _timestamp;
	_timestamp = millis();
}

unsigned long SensorData::GetTimestamp()
{
	return _timestamp;
}

unsigned long SensorData::GetTimeSinceLastExecution()
{
	return _time_since_last_execution;
}

uint8_t SensorData::GetError()
{
	return _errno;
}
