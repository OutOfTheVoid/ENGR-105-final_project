#include "step_counter.hpp"

StepCounter::StepCounter():
	mpu() {
	reset();
}

void StepCounter::begin() {
	mpu.begin();
}

void StepCounter::reset() {
	// reset data
	step_count = 0;
	last_measurement_millis = millis();
	data_buff_position = 0;
	for (int i = 0; i < STEPCNT_MEASUREMENT_BUFFER; i ++) {
		data_buffer[i] = Vec3(); // zero it out
	}
	for (int i = 0; i < STEPCNT_ENVELOPE_REDUNDANCY; i ++) {
		last_max_millis[i] = millis();
		last_max[i] = 0.0f;
	}
	envelope = 0.0f;
	last_was_step = false;
}

void StepCounter::update(HardwareSerial & serial) {
	// if time for a nother sample...
	uint32_t t = millis();
	if (t - last_measurement_millis > STEPCNT_MEASUREMENT_PERIOD_MS) {
		last_measurement_millis = t;
		// get the sample
		sensors_event_t a, g, temp;
		mpu.getEvent(&a, &g, &temp);
		// record it
		data_buffer[data_buff_position].x = a.acceleration.x;
		data_buffer[data_buff_position].y = a.acceleration.y;
		data_buffer[data_buff_position].z = a.acceleration.z;
		// average it
		Vec3 average;
		for (int i = 0; i < STEPCNT_MEASUREMENT_BUFFER; i ++) {
			average = average + data_buffer[i];
		}
		average = average * (1.0f / (float) STEPCNT_MEASUREMENT_BUFFER);
		// compute change from average of most recent sample
		float deviance = Vec3::dot(average, data_buffer[data_buff_position] - average);
		// compute "deviance" from the average
		deviance = deviance * deviance * deviance;
		// compute envelope
		uint32_t timeout_ms = STEPCNT_ENVELOPE_TIMEOUT_MS;
		for (int i = 0; i < STEPCNT_ENVELOPE_REDUNDANCY; i ++) {
			if (deviance > last_max[i]) {
				last_max[i] = deviance;
				last_max_millis[i] = t;
			} else {
				if (t - last_max_millis[i] > timeout_ms) {
					last_max[i] = 0.0f;
					for (int j = i + 1; j < STEPCNT_ENVELOPE_REDUNDANCY; j ++) {
						if (last_max[j] > last_max[i]) {
							last_max[i] = last_max[j];
						}
					}
				}
			}
			timeout_ms -= STEPCNT_ENVELOPE_TIMEOUT_FALLOFF_MS;
		}
		envelope = envelope * 0.9f + last_max[0] * 0.1f;
		// check if this is a new step
		bool is_step = (envelope > STEPCNT_ABSOLUTE_THRESHOLD_VALUE) && (envelope * STEPCNT_ENVELOPE_THRESHOLD_VALUE < deviance);
		if (is_step && !last_was_step) {
			// and count it!
			step_count ++;
		}
		last_was_step = is_step;
		
		// forward circular buffer write head
		data_buff_position ++;
		data_buff_position %= STEPCNT_MEASUREMENT_BUFFER;
		
		#ifdef STEPCNT_SERIAL_PLOTTING
			serial.print(deviance);
			serial.print(" ");
			serial.print(envelope * STEPCNT_ENVELOPE_THRESHOLD_VALUE);
			serial.print(" ");
			serial.print(step_count);
			serial.print("\n");
		#else
			(void) serial;
		#endif
	}
}

int StepCounter::get_step_count() {
	return step_count;
}

