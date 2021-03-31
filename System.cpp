/*
  System module.
  2021-01-12  T. Nakagawa
*/

#include <EEPROM.h>
#include <analog.h>
#include <pins_arduino.h>
#include "Config.h"
#include "Controller.h"
#include "Receiver.h"
#include "System.h"
#include "Utility.h"

static PWM pwm[4];

System sys;

void sys_init() {
  pinMode(PIN_LED_FR, OUTPUT);
  pinMode(PIN_LED_BK, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pwm[i].begin(digitalPinToPinName(PIN_ESC[i]), ESC_FREQ, 502, RESOLUTION_10B_COMPARE_FORMAT);
  }
  sys_load();

  if (ESC_CALIBRATE) {
    sys_motor((const float []){1.0f, 1.0f, 1.0f, 1.0f});
    digitalWrite(PIN_LED_FR, HIGH);
    digitalWrite(PIN_LED_BK, LOW);
    delay(5000);
    sys_motor((const float []){0.05f, 0.05f, 0.05f, 0.05f});
    digitalWrite(PIN_LED_FR, LOW);
    digitalWrite(PIN_LED_BK, HIGH);
    while (true) ;
  }
}

void sys_process() {
  // Check the battery voltage.
  const uint16_t adc = adc_read_value(digitalPinToPinName(PIN_BAT), 12);
  const float vbat = adc * sys.par.vbat_scale * BAT_VREF / 4096.0f;
  sys.vbat = 0.9f * sys.vbat + 0.1f * vbat;

  // Update the LED flasher.
  static int led_cnt = 8;
  uint8_t led_pat;
  if (!rec.online) {
    led_pat = LED_OFFLINE;
  } else if (ctl.calib) {
    led_pat = LED_CALIB;
  } else if (!ctl.armed) {
    led_pat = LED_DISARMED;
  } else if (ctl.nobat) {
    led_pat = LED_NOBAT;
  } else if (ctl.lowbat) {
    led_pat = LED_LOWBAT;
  } else {
    led_pat = LED_ARMED;
  }
  const int led = ((led_pat << led_cnt++) & 0x80) != 0 ? HIGH : LOW;
  if (led_cnt >= 8) led_cnt = 0;
  digitalWrite(PIN_LED_FR, led);
  digitalWrite(PIN_LED_BK, led);
}

void sys_motor(const float *motor) {
  for (int i = 0; i < 4; i++) {
    const float value = constrain(motor[i], 0.0f, 1.0f);
    const int width = 502 + (int)(502 * value);	// Map 0.0-1.0 to 502-1004 (1.0-2.0ms for the clock 490Hz).
    pwm[i].write(width);
    sys.motor[i] = value;
  }
}

void sys_load() {
  EEPROM.get(ROM_PAR, sys.par);
}

void sys_save() {
  EEPROM.put(ROM_PAR, sys.par);
}

void sys_reset() {
  sys.par.pid[0][0] = 0.1f;
  sys.par.pid[0][1] = 0.01f;
  sys.par.pid[0][2] = 0.001f;
  sys.par.pid[1][0] = 0.1f;
  sys.par.pid[1][1] = 0.01f;
  sys.par.pid[1][2] = 0.001f;
  sys.par.pid[2][0] = 0.1f;
  sys.par.pid[2][1] = 0.01f;
  sys.par.pid[2][2] = 0.0f;
  sys.par.angle_max = 50.0f * DEG2RAD;
  sys.par.rotation_max = 180.0f * DEG2RAD;
  sys.par.rotation_gain = 1.0f;
  sys.par.throttle_gain = 1.0f;
  sys.par.stk_thres = 0.05f;
  sys.par.vbat_scale = BAT_SCALE;
  sys.par.vbat_warn = 3.3f;
  sys.par.vbat_crit = 3.1f;
  sys.par.filter_sensor = 100.0f;
  sys.par.filter_dterm = 10.0f;
}
