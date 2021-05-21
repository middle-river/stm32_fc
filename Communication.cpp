/*
  Communication module.
  2021-01-12  T. Nakagawa
*/

#include <utility>
#include "Communication.h"
#include "Config.h"
#include "Controller.h"
#include "Receiver.h"
#include "Sensor.h"
#include "System.h"
#include "Utility.h"

static constexpr uint8_t MSP_IDENT           = 100;
static constexpr uint8_t MSP_STATUS          = 101;
static constexpr uint8_t MSP_RAW_IMU         = 102;
static constexpr uint8_t MSP_MOTOR           = 104;
static constexpr uint8_t MSP_RC              = 105;
static constexpr uint8_t MSP_ATTITUDE        = 108;
static constexpr uint8_t MSP_ANALOG          = 110;
static constexpr uint8_t MSP_PID             = 112;
static constexpr uint8_t MSP_BOX             = 113;
static constexpr uint8_t MSP_MISC            = 114;
static constexpr uint8_t MSP_MOTOR_PINS      = 115;
static constexpr uint8_t MSP_BOXNAMES        = 116;
static constexpr uint8_t MSP_BOXIDS          = 119;
static constexpr uint8_t MSP_SET_PID         = 202;
static constexpr uint8_t MSP_SET_MISC        = 207;
static constexpr uint8_t MSP_RESET_CONF      = 208;
static constexpr uint8_t MSP_BIND            = 241;
static constexpr uint8_t MSP_EEPROM_WRITE    = 250;
static constexpr uint8_t MSP_DEBUG           = 254;

static void respond(uint8_t cmd, const uint8_t *data, uint8_t size, bool error=false) {
  Serial.write('$');
  Serial.write('M');
  Serial.write(error ? '!' : '>');
  uint8_t csum = 0;
  Serial.write(size);
  csum ^= size;
  Serial.write(cmd);
  csum ^= cmd;
  while (size--) {
    Serial.write(*data);
    csum ^= *data++;
  }
  Serial.write(csum);
}

static void execute(uint8_t cmd, uint8_t *data) {
  switch(cmd) {
  case MSP_IDENT:
    {
      const struct __attribute__((packed)) {
        uint8_t version;
        uint8_t multitype;
        uint8_t msp_version;
        uint32_t capability;
      } ident = {240, 3, 0, ((7 << 28) | 1)};
      respond(cmd, (const uint8_t *)&ident, sizeof(ident));
    }
    break;
  case MSP_STATUS:
    {
      const struct __attribute__((packed)) {
        uint16_t cycle_time;
        uint16_t i2c_errors_count;
        uint16_t sensor;
        uint32_t flag;
        uint8_t current_set;
      } status = {(uint16_t)(ctl.cycle_time * 1.0e6f), (uint16_t)(sen.cycle_time * 1.0e6f), 0x01, (((uint32_t)ctl.selflv << 1) | ((uint32_t)ctl.armed << 0)), 0};
      respond(cmd, (const uint8_t *)&status, sizeof(status));
    }
    break;
  case MSP_RAW_IMU:
    {
      struct __attribute__((packed)) {
        int16_t acc_smooth[3];
        int16_t gyro_data[3];
        int16_t mag_adc[3];
      } imu = {};
      for (int i = 0; i < 3; i++) {
        imu.acc_smooth[i] = (int16_t)(sen.acc[i] * 512.0f);
        imu.gyro_data[i] = (int16_t)(sen.gyro[i] * RAD2DEG * 4.1f);
      }
      respond(cmd, (const uint8_t *)&imu, sizeof(imu));
    }
    break;
  case MSP_MOTOR:
    {
      int16_t motor[8] = {0, 0, 0, 0, 1500, 1500, 1500, 1500};
      for (int i = 0; i < 4; i++) {
        motor[i] = 1000 + (int)(sys.motor[i] * 1000.0f);	// Map 0.0-1.0 to 1000-2000.
      }
      respond(cmd, (const uint8_t *)&motor, sizeof(motor));
    }
    break;
  case MSP_RC:
    {
      int16_t rc_data[8] = {};
      for (int i = 0; i < 8; i++) {
        rc_data[i] = 1000 + (int)(rec.value[i] * 1000.0f);	// Map 0.0-1.0 to 1000-2000.
      }
      respond(cmd, (const uint8_t *)&rc_data, sizeof(rc_data));
    }
    break;
  case MSP_ATTITUDE:
    {
      struct __attribute__((packed)) {
        int16_t angle[2];
        int16_t heading;
      } att = {};
      att.angle[0] = (int)(sen.angle[0] * RAD2DEG * 10.0f);
      att.angle[1] = (int)(sen.angle[1] * RAD2DEG * -10.0f);
      att.heading = (int)(sen.angle[2] * RAD2DEG);
      respond(cmd, (const uint8_t *)&att, sizeof(att));
    }
    break;
  case MSP_ANALOG:
    {
      struct __attribute__((packed)) {
        uint8_t vbat;
        uint16_t int_power_meter_sum;
        uint16_t rssi;
        uint16_t amperage;
      } analog = {};
      analog.vbat = (int)(sys.vbat * 10.0f);
      respond(cmd, (const uint8_t *)&analog, sizeof(analog));
    }
    break;
  case MSP_PID:
    {
      uint8_t pid[10][3] = {};
      for (int i = 0; i < 3; i++) {
        pid[i][0] = (uint8_t)(sys.par.pid[i][0] * 100.0f);
        pid[i][1] = (uint8_t)(sys.par.pid[i][1] * 1000.0f);
        pid[i][2] = (uint8_t)(sys.par.pid[i][2] * 10000.0f);
      }
      pid[5][0] = (uint8_t)(sys.par.filter_acc * 1.0f);
      pid[6][0] = (uint8_t)(sys.par.filter_gyro * 1.0f);
      pid[7][0] = (uint8_t)(sys.par.filter_dterm * 1.0f);
      pid[5][1] = (uint8_t)(sys.par.angle_max * RAD2DEG * 0.1f + 0.5f);
      pid[6][1] = (uint8_t)(sys.par.rotation_max * RAD2DEG * 0.1f + 0.5f);
      pid[5][2] = (uint8_t)(sys.par.rotation_gain * 10.0f);
      pid[6][2] = (uint8_t)(sys.par.throttle_gain * 10.0f);
      respond(cmd, (const uint8_t *)*pid, sizeof(pid));
    }
    break;
  case MSP_BOX:
    {
      const uint16_t activate[2] = {0, 0};
      respond(cmd, (const uint8_t *)activate, sizeof(activate));
    }
    break;
  case MSP_MISC:
    {
      struct __attribute__((packed)) {
        uint16_t int_power_trigger;
        uint16_t min_throttle;
        uint16_t max_throttle;
        uint16_t min_command;
        uint16_t failsafe_throttle;
        uint16_t arm;
        uint32_t lifetime;
        uint16_t mag_declination;
        uint8_t vbatscale;
        uint8_t vbatlevel_warn1;
        uint8_t vbatlevel_warn2;
        uint8_t vbatlevel_crit;
      } misc = {};
      misc.min_command = (int)(sys.par.stk_thres * 1000.0f);
      misc.vbatscale = (int)(sys.par.vbat_scale * 70.0f);
      misc.vbatlevel_warn2 = (int)(sys.par.vbat_warn * 10.0f);
      misc.vbatlevel_crit = (int)(sys.par.vbat_crit * 10.0f);
      respond(cmd, (const uint8_t *)&misc, sizeof(misc));
    }
    break;
  case MSP_MOTOR_PINS:
    {
      const uint8_t pwm_pin[8] = {0, 1, 2, 3, 9, 9, 9, 9};
      respond(cmd, pwm_pin, sizeof(pwm_pin));
    }
    break;
  case MSP_BOXNAMES:
    {
      const char boxnames[] = "ARM;ANGLE;";
      respond(cmd, (const uint8_t *)boxnames, strlen(boxnames));
    }
    break;
  case MSP_BOXIDS:
    {
      const uint8_t boxids[2] = {0, 1};
      respond(cmd, boxids, sizeof(boxids));
    }
    break;
  case MSP_SET_PID:
    {
      const uint8_t (*pid)[3] = (uint8_t (*)[3])data;
      for (int i = 0; i < 3; i++) {
        sys.par.pid[i][0] = (float)pid[i][0] / 100.0f;
        sys.par.pid[i][1] = (float)pid[i][1] / 1000.0f;
        sys.par.pid[i][2] = (float)pid[i][2] / 10000.0f;
      }
      sys.par.filter_acc    = pid[5][0] * 1.0f;
      sys.par.filter_gyro   = pid[6][0] * 1.0f;
      sys.par.filter_dterm  = pid[7][0] * 1.0f;
      sys.par.angle_max     = pid[5][1] * DEG2RAD * 10.0f;
      sys.par.rotation_max  = pid[6][1] * DEG2RAD * 10.0f;
      sys.par.rotation_gain = pid[5][2] * 0.1f;
      sys.par.throttle_gain = pid[6][2] * 0.1f;
      respond(cmd, NULL, 0);
    }
    break;
  case MSP_SET_MISC:
    {
      const struct __attribute__((packed)) Misc {
        uint16_t int_power_trigger;
        uint16_t min_throttle;
        uint16_t max_throttle;
        uint16_t min_command;
        uint16_t failsafe_throttle;
        uint16_t arm;
        uint32_t lifetime;
        uint16_t mag_declination;
        uint8_t vbatscale;
        uint8_t vbatlevel_warn1;
        uint8_t vbatlevel_warn2;
        uint8_t vbatlevel_crit;
      } *misc = (const struct Misc *)data;
      sys.par.stk_thres = misc->min_command / 1000.0f;
      sys.par.vbat_scale = misc->vbatscale / 70.0f;
      sys.par.vbat_warn = misc->vbatlevel_warn2 / 10.0f;
      sys.par.vbat_crit = misc->vbatlevel_crit / 10.0f;
      respond(cmd, NULL, 0);
    }
    break;
  case MSP_RESET_CONF:
    {
      if (!ctl.armed) sys_reset();
      respond(cmd, NULL, 0);
    }
    break;
  case MSP_BIND:
    {
      rec_rebind();
      respond(cmd, NULL, 0);
    }
    break;
  case MSP_EEPROM_WRITE:
    {
      sys_save();
      respond(cmd, NULL, 0);
    }
    break;
  case MSP_DEBUG:
    {
      int16_t debug[4] = {};
      debug[0] = (int16_t)(ctl.elapsed_time * 1.0e6f);
      debug[1] = (int16_t)(sen.elapsed_time * 1.0e6f);
      debug[2] = (int16_t)rec.count;
      debug[3] = (int16_t)0;
      respond(cmd, (const uint8_t *)debug, sizeof(debug));
    }
    break;
  default:
    {
      respond(cmd, NULL, 0, true);
    }
    break;
  }
}

void com_init() {
  Serial.begin(COM_BAUDRATE);
}

void com_process() {
  static int buf_ptr = 0;
  static uint8_t buf[64];
  static uint8_t buf_csum = 0;

  if (Serial.availableForWrite() < 50) return;
  if (Serial.available() <= 0) return;

  // Protocol: '$' 'M' ('<'|'>') <size> <command> <data>* <check sum>
  const uint8_t c = (uint8_t)Serial.read();
  buf[buf_ptr++] = c;
  if ((buf_ptr == 1 && c != '$') ||
      (buf_ptr == 2 && c != 'M') ||
      (buf_ptr == 3 && c != '<') ||
      buf_ptr >= 64) {
    buf_ptr = 0;
  }
  if (buf_ptr == 4) buf_csum = 0;
  if (buf_ptr >= 4) buf_csum ^= c;
  if (buf_ptr >= 6 && buf_ptr == buf[3] + 6) {
    if (buf_csum == 0) execute(buf[4], buf + 5);
    buf_ptr = 0;
  }
}
