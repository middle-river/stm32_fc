/*
  System module.
  2021-01-12  T. Nakagawa
*/

#ifndef SYSTEM_H_
#define SYSTEM_H_

struct Parameter {
  float pid[3][3];	// PID coefficients for {Roll, Pitch, Yaw}{P, I, D}
  float angle_max;	// Maximum angle in ANGLE mode [rad].
  float rotation_max;	// Maximum rotation rate in ACRO mode [rad/s].
  float rotation_gain;	// The gain of the rotation rate.
  float throttle_gain;	// The gain for the throttle.
  float stk_thres;	// Threshold for stick command input.
  float vbat_scale;	// Scaling factor of the battery voltage.
  float vbat_warn;	// Battery voltage for the warning level [V].
  float vbat_crit;	// Battery voltage for the critical level [V].
  float filter_acc;	// Filter cutoff frequency for the accelerometer [Hz].
  float filter_gyro;	// Filter cutoff frequency for the gyro [Hz].
  float filter_dterm;	// Filter cutoff frequency for the PID controller D term.
};

struct System {
  struct Parameter par;
  float vbat;		// Battery voltage [V].
  float motor[4];	// [0.0, 1.0] for RearR, FrontR, RearL, FrontL.
};

extern System sys;

void sys_init();			// Initialize the miscellaneous tasks..
void sys_process();			// Process the miscellaneous tasks.
void sys_motor(const float *motor);	// Set the motor speed.
void sys_load();			// Load the parameters.
void sys_save();			// Save the parameters.
void sys_reset();			// Reset the parameters.
void sys_calib();			// Calibrate the system.

#endif
