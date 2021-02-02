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
#include "SensorCurrent.hpp"

// should probably check which board this is                                    
#define ADC_RESOLUTION (1023.0)

/*
 * SensorCurrent
 */

SensorCurrent::SensorCurrent(const uint8_t analogPin, const uint8_t sample_count, const unsigned long sampling_interval)
    : Sensor(sampling_interval)
    , _analogPin(analogPin)
	, _filter(sample_count)
	, _sample_count(sample_count)
{
}

SensorCurrent::~SensorCurrent()
{
}

bool SensorCurrent::Init()
{
	pinMode(_analogPin, INPUT);
    _vref = ReadReferenceVoltage();
	return (true);
}

bool SensorCurrent::Update()
{
	bool error_occured = false;

    if (!_timer.Unlock())
        return (true);

    _data._current = ReadDCCurrent();

	if (_data._monitoring_enabled) {
		if (_data._current < _data._lower_limit) {
			_data._errno = SensorCurrentData::CURRENT_CAP_LOWER;
			error_occured = true;
		} else if (_data._current > _data._upper_limit) {
			_data._errno = SensorCurrentData::CURRENT_CAP_UPPER;
			error_occured = true;
		}
	}
    return (error_occured == false);
}

SensorCurrentData &SensorCurrent::RetreiveData()
{
	return (_data);
}

void SensorCurrent::SetMonitoringParameters(const uint16_t lower_limit, const uint16_t upper_limit)
{
	_data._lower_limit = lower_limit;
	_data._upper_limit = upper_limit;
}

uint16_t SensorCurrent::GetCurrentMilliAmps() const
{
    return (_data.GetCurrentMilliAmps());
}

uint8_t SensorCurrent::GetCurrentAmps() const
{
    return (_data.GetCurrentAmps());
}

/*read DC Current Value
 * https://github.com/nxcosa/20A-CURRENT-SENSOR/blob/master/_20AcurrentSensor.ino
 */
uint16_t SensorCurrent::ReadDCCurrent()
{
	_filter.Reset();
    for (int i = 0; i < _sample_count; i++) {
        _filter.NewReading(analogRead(_analogPin));
    }
    const uint16_t median_reading = _filter.GetFilteredSignal();
	int16_t dc_current = (median_reading / ADC_RESOLUTION * _vref) / _mVperAmp / 2;
    return ((dc_current < 0) ? 0 : dc_current);
}

/* read reference voltage
 * https://github.com/nxcosa/20A-CURRENT-SENSOR/blob/master/_20AcurrentSensor.ino
 * If testing with Arduino-CI, don't attempt to use hardware defines (breaks arduino-ci's compilation of targets) 
 */
#ifndef ARDUINO_CI
long SensorCurrent::ReadReferenceVoltage()
{
    long result;
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB &= ~_BV(MUX5); // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#endif
#if defined(__AVR__)
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC))
        ;
    result = ADCL;
    result |= ADCH << 8;
    result = 1126400L / result; //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
    return result;
#elif defined(__arm__)
    return (3300); //Arduino Due
#else
    return (3300); //Guess that other un-supported architectures will be running a 3.3V!
#endif
}
#else
long SensorCurrent::ReadReferenceVoltage()
{
	return (4500); // assume 4.5V for testing
}
#endif

/*
 * SensorCurrentData
 */

uint16_t SensorCurrentData::GetCurrentMilliAmps() const
{
    return (_current * 1000);
}

uint8_t SensorCurrentData::GetCurrentAmps() const
{
    return (_current);
}

#ifdef ROS
void SensorCurrentData::Publish()
{
	// set _current_msg variables
}
#endif

