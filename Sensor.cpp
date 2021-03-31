/*
  Sensor module for MPU-6000.
  2021-01-12  T. Nakagawa
*/

#include <EEPROM.h>
#include <SPI.h>
#include "Config.h"
#include "Sensor.h"
#include "System.h"
#include "Utility.h"

struct Offset {
  float acc[3];
  float gyro[3];
};

static constexpr float COMP_KP = 0.5f;	// Proportional controller parameter for the complementary filter.

static SPIClass spi(PIN_IMU_MOSI, PIN_IMU_MISO, PIN_IMU_SCK);
static Offset offset;				// Sensor offset.
static Offset calib;				// Accumulators for calibration.
static int calib_num = 0;			// Number of samples for calibration.
static bool calib_enable = false;		// Enable calibration.
static float rot[4] = {1.0f, 0.0f, 0.0f, 0.0f};	// Quaternions of estimated orientation.

static inline uint16_t bswap_16(uint16_t x) {
  return ((x << 8) | (x >> 8));
}

static inline void read_register(uint8_t reg, uint8_t *data, uint8_t len) {
  digitalWrite(PIN_IMU_CS, LOW);
  spi.transfer(0x80 | reg);
  spi.transfer(data, len);
  digitalWrite(PIN_IMU_CS, HIGH);
}

static inline void write_register(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_IMU_CS, LOW);
  spi.transfer(reg);
  spi.transfer(val);
  digitalWrite(PIN_IMU_CS, HIGH);
}

Sensor sen;

void sen_init() {
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  spi.begin();
  EEPROM.get(ROM_SEN, offset);

  // Initialize MPU-6000.
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  write_register(0x6b, 0x08);	// Power Management 1 (device reset)
  delay(100);
  write_register(0x68, 0x07);	// Signal Path Reset (gyro/accel/temp reset)
  delay(100);
  write_register(0x6b, 0x03);	// Power Management 1 (gyro Z clock source)
  write_register(0x1b, 0x18);	// Gyroscope Cnofiguration (2000deg/sec full scale range)
  write_register(0x1c, 0x18);	// Accelerometer Configuration (16G full scale range)
  spi.endTransaction();
  spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
}

void sen_process() {
  const uint32_t time_now = micros();

  // Read the sensor values.
  int16_t data[7];
  read_register(0x3b, (uint8_t *)data, 14);
  for (int i = 0; i < 7; i++) data[i] = bswap_16(data[i]);
  for (int i = 0; i < 3; i++) {
    float acc = data[i] / 2048.0f;	// 2048 LSB/G
    if (IMU_REV[i]) acc = -acc;
    sen.acc[IMU_MAP[i]] = acc;
    float gyro = data[4 + i] / 16.4f * DEG2RAD;	// 16.4 LSB/deg/s
    if (IMU_REV[i]) gyro = -gyro;
    sen.gyro[IMU_MAP[i]] = gyro;
  }
  if (calib_enable) {
    if (calib_num == 0) {
      for (int i = 0; i < 3; i++) {
        calib.acc[i] = 0.0f;
        calib.gyro[i] = 0.0f;
      }
    }
    for (int i = 0; i < 3; i++) {
      calib.acc[i] += sen.acc[i];
      calib.gyro[i] += sen.gyro[i];
    }
    calib_num++;
  } else if (calib_num > 0) {
    for (int i = 0; i < 3; i++) {
      offset.acc[i] = calib.acc[i] / calib_num;
      offset.gyro[i] = calib.gyro[i] / calib_num;
    }
    offset.acc[2] += 1.0;	// Gravity.
    EEPROM.put(ROM_SEN, offset);
    rot[0] = 1.0f; rot[1] = rot[2] = rot[3] = 0.0f;
    calib_num = 0;
  }

  // Apply LPF to the raw sensor values after ofsetting the bias.
  static float lpf_cutoff = -1.0f;
  static LPF lpf_acc[3], lpf_gyro[3];
  if (lpf_cutoff != sys.par.filter_sensor) {
    lpf_cutoff = sys.par.filter_sensor;
    for (int i = 0; i < 3; i++) {
      lpf_acc[i].reset(lpf_cutoff, SEN_SAMPFREQ);
      lpf_gyro[i].reset(lpf_cutoff, SEN_SAMPFREQ);
    }
  }
  for (int i = 0; i < 3; i++) {
    sen.acc[i] = lpf_acc[i].filter(sen.acc[i] - offset.acc[i]);
    sen.gyro[i] = lpf_gyro[i].filter(sen.gyro[i] - offset.gyro[i]);
  }
  if (sen.acc[0] == 0.0f && sen.acc[1] == 0.0f && sen.acc[2] == 0.0f) sen.acc[2] = -1.0f;	// Avoid invalid sensor values.

  // Estimate the attitude with the complementary filter.
  const float zacc = 1.0f / sqrtf(sen.acc[0] * sen.acc[0] + sen.acc[1] * sen.acc[1] + sen.acc[2] * sen.acc[2]);
  const float acc[3] = {-sen.acc[0] * zacc, -sen.acc[1] * zacc, -sen.acc[2] * zacc};	// Normalized measured gravity.
  const float grav[3] = {
    2.0f * (rot[1] * rot[3] - rot[0] * rot[2]),
    2.0f * (rot[2] * rot[3] + rot[0] * rot[1]),
    rot[0] * rot[0] - rot[1] * rot[1] - rot[2] * rot[2] + rot[3] * rot[3]
  };	// Estimated gravity vector.
  const float err[3] = {
    acc[1] * grav[2] - acc[2] * grav[1],
    acc[2] * grav[0] - acc[0] * grav[2],
    acc[0] * grav[1] - acc[1] * grav[0]
  };	// Error vector.
  const float gyro[3] = {
    sen.gyro[0] + COMP_KP * err[0],
    sen.gyro[1] + COMP_KP * err[1],
    sen.gyro[2] + COMP_KP * err[2]
  };	// Corrected rotation rate.
  const float dlt[4] = {
    0.5f * (-rot[1] * gyro[0] - rot[2] * gyro[1] - rot[3] * gyro[2]),
    0.5f * ( rot[0] * gyro[0] - rot[3] * gyro[1] + rot[2] * gyro[2]),
    0.5f * ( rot[3] * gyro[0] + rot[0] * gyro[1] - rot[1] * gyro[2]),
    0.5f * (-rot[2] * gyro[0] + rot[1] * gyro[1] + rot[0] * gyro[2])
  };	// Rate of orientation change.
  for (int i = 0; i < 4; i++) rot[i] += dlt[i] * (1.0f / SEN_SAMPFREQ);
  const float zrot = 1.0f / sqrtf(rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2] + rot[3] * rot[3]);
  for (int i = 0; i < 4; i++) rot[i] *= zrot;

  // Convert quaternion to angles.
  sen.angle[0] = atan2f(2.0f * (rot[2] * rot[3] + rot[0] * rot[1]), rot[0] * rot[0] - rot[1] * rot[1] - rot[2] * rot[2] + rot[3] * rot[3]);
  sen.angle[1] = asinf(2.0f * (rot[0] * rot[2] - rot[1] * rot[3]));
  sen.angle[2] = atan2f(2.0f * (rot[1] * rot[2] + rot[0] * rot[3]), rot[0] * rot[0] + rot[1] * rot[1] - rot[2] * rot[2] - rot[3] * rot[3]);

  static uint32_t time_old;
  sen.elapsed_time = (micros() - time_now) * 1.0e-6f;
  sen.cycle_time = (time_now - time_old) * 1.0e-6f;
  time_old = time_now;
}

void sen_calibrate(bool enable) {
  calib_enable = enable;
}
