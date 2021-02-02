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

#ifndef SENSOR_SENSORDATA_HPP
#define SENSOR_SENSORDATA_HPP

#ifdef ROS
# include "ros.h"
#endif

class SensorData {
	public:
		SensorData();
		virtual ~SensorData();
		unsigned long GetTimestamp() const;
		unsigned long GetTimeSinceLastExecution() const;
		uint8_t GetError() const;
		void EnableMonitoring();
		void DisableMonitoring();
	protected:
		void UpdateTimestamps();
		uint8_t _errno = 0;
		bool _monitoring_enabled = false;
	private:
		unsigned long _timestamp = 0;
		unsigned long _time_since_last_execution = 0;
#ifdef ROS
	public:
		SensorData(const char *topic_name, ros::Msg *msg);
		virtual void Publish();
		void PublishMsg(const ros::Msg * msg);
		virtual ros::Publisher *GetPublisher() const;
		void SetPublisher(const char *topic_name, ros::Msg *msg);
		bool IsPublishingEnabled() const;
		void EnablePublishing();
		void DisablePublishing();
	private:
		ros::Publisher *_pub = NULL;
		bool _publishing_enabled = false;
#endif
};

#endif
