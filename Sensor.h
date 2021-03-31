/*
  Sensor module.
  2021-01-12  T. Nakagawa
*/

#ifndef SENSOR_H_
#define SENSOR_H_

struct Sensor {
  float acc[3];		// [G] for x, y, z (+x: forward, +y: left, +z: down).
  float gyro[3];	// [rad/s] for roll, pitch, yaw (the rotations of right screws for x, y, z).
  float angle[3];	// [rad] for roll, pitch, yaw.
  float cycle_time;	// Cycle time [s].
  float elapsed_time;	// Elapsed time [s].
};

extern Sensor sen;

void sen_init();			// Initialize the sensor.
void sen_process();			// Process the sensor.
void sen_calibrate(bool enable);	// Enable or disable sensor calibration.

#endif
