#ifndef STEP_COUNTER_HPP
#define STEP_COUNTER_HPP

#include "vec3.hpp"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#define STEPCNT_MEASUREMENT_PERIOD_MS 25
#define STEPCNT_MEASUREMENT_BUFFER 20


#define STEPCNT_ENVELOPE_TIMEOUT_MS 500
#define STEPCNT_ENVELOPE_TIMEOUT_FALLOFF_MS 50
#define STEPCNT_ENVELOPE_REDUNDANCY 10

#define STEPCNT_ABSOLUTE_THRESHOLD_VALUE 2000.0f
#define STEPCNT_ENVELOPE_THRESHOLD_VALUE 0.6f

//#define STEPCNT_SERIAL_PLOTTING

class StepCounter {
public:
	StepCounter();
	
	void begin();
	void update(HardwareSerial & serial);
	
	void reset();
	int get_step_count();
	
private:
	Adafruit_MPU6050 mpu;
	int step_count;
	uint32_t last_measurement_millis;
	Vec3 data_buffer[STEPCNT_MEASUREMENT_BUFFER];
	int data_buff_position;
	
	float last_max[STEPCNT_ENVELOPE_REDUNDANCY];
	uint32_t last_max_millis[STEPCNT_ENVELOPE_REDUNDANCY];
	float envelope;
	
	bool last_was_step;
};

#endif
