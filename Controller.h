/*
  Controller module.
  2021-01-12  T. Nakagawa
*/

#ifndef CONTROLLER_H_
#define CONROLLER_H_

struct Controller {
  bool nobat;		// No battery state.
  bool lowbat;		// Low battery state.
  bool armed;		// Armed state.
  bool calib;		// Calibration state.
  bool selflv;		// Self-leveling (angle) mode.
  float cycle_time;	// Cycle time [s].
  float elapsed_time;	// Elapsed time [s].
};

extern Controller ctl;

void ctl_init();	// Initialize the controller.
void ctl_process();	// Process the controller.

#endif
