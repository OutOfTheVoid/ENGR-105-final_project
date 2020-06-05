#include "heart_rate_sensor.hpp"
#include <math.h>

HeartRateSensor::HeartRateSensor(int pin):
	pin(pin){
	reset();
}

void HeartRateSensor::reset() {
	// clear data
	data_length = 0;
	new_data_position = 0;
	last_micros = micros();
	for (int i = 0; i < HRSENSOR_ENVELOPE_REDUNDANCY; i ++) {
		last_max_millis[i] = millis();
		last_max[i] = 0.0f;
	}
	envelope = 0.0f;
	for (int i = 0; i < HRSENSOR_PULSE_TIME_COUNT; i ++) {
		last_pulse_millis[i] = 0;
	}
	last_pulse_state = false;
	new_pulse = false;
}

void HeartRateSensor::update(HardwareSerial & serial) {
	// measure time
	uint64_t t = micros();
	uint32_t t_millis = millis();
	// if we should sample again, do it!
	if (t - last_micros >= HRSENSOR_MICROS_PER_SAMPLE) {
		last_micros = t;
		// oversample a bit to clean up input noise some
		float total = 0.0f;
		for (int i = 0; i < HRSENSOR_OVERSAMPLE; i ++) {
			delayMicroseconds(HRSENSOR_OVERSAMPLE_TIME_US);
			total += analogRead(pin);
		}
		// record data in circular buffer
		buffer[new_data_position] = total;
		new_data_position = (new_data_position + 1) % HRSENSOR_BUFFER_SIZE;
		if (data_length < HRSENSOR_BUFFER_SIZE) {
			data_length ++;
		}
		
		// if we have enough data...
		if (data_length == HRSENSOR_BUFFER_SIZE) {
			// compute averages
			float average_d = 0.0f;
			for (int j = 0; j < HRSENSOR_BUFFER_SIZE - 2; j ++) {
				float immediate_d = 
					buffer[(new_data_position + 2 + j) % HRSENSOR_BUFFER_SIZE] - 
					buffer[(new_data_position + 1 + j) % HRSENSOR_BUFFER_SIZE];
				float weight = ((float) j) - (float (HRSENSOR_BUFFER_SIZE / 2)) / (float) HRSENSOR_BUFFER_SIZE;
				average_d += immediate_d * weight;
			}
			// record average and slew-filtered average
			delta = average_d;
			delta_base = delta_base * 0.8f + average_d * 0.2f;
			
			// compute envelope with a cascaded buffer
			uint32_t timeout_ms = HRSENSOR_ENVELOPE_TIMEOUT_MS;
			for (int i = 0; i < HRSENSOR_ENVELOPE_REDUNDANCY; i ++) {
				if (delta_base > last_max[i]) {
					last_max[i] = delta_base;
					last_max_millis[i] = t_millis;
				} else {
					if (t_millis - last_max_millis[i] > timeout_ms) {
						last_max[i] = 0.0f;
						for (int j = i + 1; j < HRSENSOR_ENVELOPE_REDUNDANCY; j ++) {
							if (last_max[j] > last_max[i]) {
								last_max[i] = last_max[j];
							}
						}
					}
				}
				timeout_ms -= HRSENSOR_ENVELOPE_TIMEOUT_FALLOFF_MS;
			}
			if (last_max[0] > envelope) {
				envelope = last_max[0];
			} else {
				envelope = envelope * 0.8f + last_max[0] * 0.2f;
			}
			// detect rising edge of a pulse
			bool pulse_state = delta_base > envelope * HRSENSOR_THRESHOLD;
			new_pulse = pulse_state & !last_pulse_state;
			last_pulse_state = pulse_state;
			
			// if a new pulse happened, record it's time in the shifting pulse buffer
			if (new_pulse) {
				for (int i = HRSENSOR_PULSE_TIME_COUNT - 2; i >= 0; i --) {
					last_pulse_millis[i + 1] = last_pulse_millis[i];
				}
				last_pulse_millis[0] = t_millis;
			}
			
			#ifdef HRSENSOR_SERIAL_PLOTTING
				serial.print(envelope * HRSENSOR_THRESHOLD);
				serial.print(" ");
				serial.print(new_pulse ? 100.0f : 0.0f);
				serial.print(" ");
				serial.println(delta_base);
			#else
				(void) serial;
			#endif
		}
	}
}

void HeartRateSensor::get_heartrate_and_variance(float & heartrate, float & variance) {
	// compute average time between pulses
	float average_pulse_interval = (last_pulse_millis[0] - last_pulse_millis[HRSENSOR_PULSE_TIME_COUNT - 1]) / (float) (HRSENSOR_PULSE_TIME_COUNT - 1);
	// compute variance
	float total_variance = 0.0f;
	for (int i = HRSENSOR_PULSE_TIME_COUNT - 2; i >= 0; i --) {
		float deviation = fabs((last_pulse_millis[i] - last_pulse_millis[i + 1]) - average_pulse_interval) / average_pulse_interval;
		total_variance += deviation * deviation;
	}
	variance = total_variance / (float) (HRSENSOR_PULSE_TIME_COUNT - 1);
	// compute heart rate
	heartrate = 0.06f * average_pulse_interval;
}

bool HeartRateSensor::get_heartbeat_signal() {
	return new_pulse;
}
