#ifndef HEART_RATE_SENSOR_HPP
#define HEART_RATE_SENSOR_HPP

#include <stdint.h>
#include "Arduino.h"

#define HRSENSOR_OVERSAMPLE 40
#define HRSENSOR_OVERSAMPLE_TIME_US 100

#define HRSENSOR_BUFFER_SIZE 20
#define HRSENSOR_MICROS_PER_SAMPLE 10000

#define HRSENSOR_THRESHOLD 0.7f

#define HRSENSOR_ENVELOPE_TIMEOUT_MS 1500
#define HRSENSOR_ENVELOPE_TIMEOUT_FALLOFF_MS 70
#define HRSENSOR_ENVELOPE_REDUNDANCY 20

#define HRSENSOR_PULSE_TIME_COUNT 5

// #define HRSENSOR_SERIAL_PLOTTING

class HeartRateSensor {
public:
	HeartRateSensor(int pin);
	
	void update(HardwareSerial & serial);
	void reset();
	
	void get_heartrate_and_variance(float & heartrate, float & variance);
	bool get_heartbeat_signal();
	
private:
	
	int pin;
	uint64_t last_micros;
	
	float buffer[HRSENSOR_BUFFER_SIZE];
	float delta, delta_base;
	int data_length;
	int new_data_position;
	
	float last_max[HRSENSOR_ENVELOPE_REDUNDANCY];
	uint32_t last_max_millis[HRSENSOR_ENVELOPE_REDUNDANCY];
	float envelope;
	
	bool new_pulse;
	bool last_pulse_state;
	uint32_t last_pulse_millis[HRSENSOR_PULSE_TIME_COUNT];
};

#endif
