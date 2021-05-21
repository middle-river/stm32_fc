/*
  Controller module.
  2021-01-12  T. Nakagawa
*/

#include "Communication.h"
#include "Config.h"
#include "Controller.h"
#include "Receiver.h"
#include "Sensor.h"
#include "System.h"
#include "Utility.h"

static constexpr float CALIBRATION_TIME = 2.0f;	// Calibration duration for sensors.
static constexpr float MOTOR_MIX[4][3] = {{-1.0f, -1.0f, -1.0f}, {-1.0f, +1.0f, +1.0f}, {+1.0f, -1.0f, +1.0f}, {+1.0f, +1.0f, -1.0f}};	// Quad X.

Controller ctl;

void ctl_init() {
}

void ctl_process() {
  const uint32_t time_now = micros();

  // Update states.
  static uint32_t calib_timer = 0;
  const int stk_v = (rec.value[3] <= sys.par.stk_thres) ? -1 : (rec.value[3] >= 1.0f - sys.par.stk_thres) ? +1 : 0;
  const int stk_h = (rec.value[2] <= sys.par.stk_thres) ? -1 : (rec.value[2] >= 1.0f - sys.par.stk_thres) ? +1 : 0;
  ctl.selflv = (rec.value[4] > 0.5f);
  if (!ctl.nobat && sys.vbat <= sys.par.vbat_crit * BAT_CELL) {
    ctl.nobat = true;
  }
  if (ctl.nobat && sys.vbat > sys.par.vbat_crit * BAT_CELL + 0.1f) {
    ctl.nobat = false;
  }
  if (!ctl.lowbat && sys.vbat <= sys.par.vbat_warn * BAT_CELL) {
    ctl.lowbat = true;
  }
  if (ctl.lowbat && sys.vbat > sys.par.vbat_warn * BAT_CELL + 0.1f) {
    ctl.lowbat = false;
  }
  if (!ctl.calib && !ctl.armed) {
    if (stk_v > 0 && stk_h < 0) {
      ctl.calib = true;
      sen_calibrate(true);
      calib_timer = millis() + (uint32_t)CALIBRATION_TIME * 1000;
    }
  }
  if (ctl.calib) {
    if (millis() >= calib_timer) {
      ctl.calib = false;
      sen_calibrate(false);
    }
  }
  if (!ctl.armed && !ctl.calib) {
    if (stk_v < 0 && stk_h > 0) {
      ctl.armed = true;
    }
  }
  if (ctl.armed) {
    if (!rec.online || (stk_v < 0 && stk_h < 0)) {
      ctl.armed = false;
      sys_motor((const float []){0.0f, 0.0f, 0.0f, 0.0f});
    }
  }

  // PID controller.
  static float integ[3];
  static float deriv[3];
  float output[3];
  const float integ_limit = sys.par.rotation_max * CTL_LOOPFREQ;
  static float lpf_cutoff = -1.0f;
  static LPF_EMA lpf_d[3];
  if (lpf_cutoff != sys.par.filter_dterm) {
    lpf_cutoff = sys.par.filter_dterm;
    for (int i = 0; i < 3; i++) {
      lpf_d[i].reset(lpf_cutoff, CTL_LOOPFREQ);
    }
  }
  for (int i = 0; i < 3; i++) {
    float setpoint;
    if (ctl.selflv && i != 2) {	// ANGLE mode.
      setpoint = ((rec.value[i] - 0.5f) / 0.5f * sys.par.angle_max - sen.angle[i]) * (sys.par.rotation_max / sys.par.angle_max);
    } else {			// ACRO mode.
      setpoint = (rec.value[i] - 0.5f) / 0.5f * sys.par.rotation_max;
    }
    const float error = setpoint - sen.gyro[i];
    integ[i] = constrain(integ[i] + error, -integ_limit, integ_limit);
    deriv[i] = lpf_d[i].filter(error - deriv[i]);
    const float p_term = sys.par.pid[i][0] * error;
    const float i_term = sys.par.pid[i][1] * integ[i] / CTL_LOOPFREQ;
    const float d_term = sys.par.pid[i][2] * deriv[i] * CTL_LOOPFREQ;
    output[i] = p_term + i_term + d_term;
    deriv[i] = error;
  }

  // Mix the results and output to the motors.
  float motor[4];
  for (int i = 0; i < 4; i++) {
    motor[i] = rec.value[3] * sys.par.throttle_gain;
    for (int j = 0; j < 3; j++) motor[i] += MOTOR_MIX[i][j] * output[j] * sys.par.rotation_gain;
  }
  if (rec.value[3] <= sys.par.stk_thres) {
    for (int i = 0; i < 4; i++) motor[i] = 0.0f;
    for (int i = 0; i < 3; i++) integ[i] = deriv[i] = 0.0f;
  }
  for (int i = 0; i < 4; i++) motor[i] = constrain(motor[i], 0.0f, 1.0f);
  if (ctl.armed) sys_motor(motor);

  static uint32_t time_old;
  ctl.elapsed_time = (micros() - time_now) * 1.0e-6f;
  ctl.cycle_time = (time_now - time_old) * 1.0e-6f;
  time_old = time_now;
}
