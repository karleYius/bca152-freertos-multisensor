#ifndef ALARM_LOGIC_H
#define ALARM_LOGIC_H

enum class AlarmState
{
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
};

AlarmState evaluateTemperature(float temperature);

#endif