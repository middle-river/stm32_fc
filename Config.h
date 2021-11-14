/*
  Global configuration.
  2021-01-12  T. Nakagawa
*/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>
#include "Utility.h"

constexpr int PIN_BAT      = PA0;
constexpr int PIN_ESC[4]   = {PB6, PB7, PB8, PB9};
constexpr int PIN_IMU_CS   = PB12;
constexpr int PIN_IMU_MISO = PB14;
constexpr int PIN_IMU_MOSI = PB15;
constexpr int PIN_IMU_SCK  = PB13;
constexpr int PIN_LED_BK   = PB4;
constexpr int PIN_LED_FR   = PB3;
constexpr int PIN_TRX_CE   = PA3;
constexpr int PIN_TRX_CS   = PA4;
constexpr int PIN_TRX_MISO = PA6;
constexpr int PIN_TRX_MOSI = PA7;
constexpr int PIN_TRX_SCK  = PA5;

constexpr int ROM_REC = 0x0000;		// 32 bytes.
constexpr int ROM_SEN = ROM_REC + 32;	// 32 bytes.
constexpr int ROM_PAR = ROM_SEN + 32;	// 96 bytes.

constexpr uint8_t LED_OFFLINE  = 0b10000000;
constexpr uint8_t LED_CALIB    = 0b11011010;
constexpr uint8_t LED_ACRO     = 0b11001000;
constexpr uint8_t LED_ANGLE    = 0b11001100;
constexpr uint8_t LED_NOBAT    = 0b10101010;
constexpr uint8_t LED_LOWBAT   = 0b11111100;
constexpr uint8_t LED_ARMED    = 0b11111111;

constexpr int ESC_FREQ = 3922;		// ESC protocol (PWM: 490, Oneshot125: 3922, Oneshot42: 11766).

constexpr int BAT_CELL = 1;					// Number of LiPo cells.
constexpr float BAT_VREF = 2.85f;				// ADC Vref voltage [V].
constexpr float BAT_SCALE = (39000.0f + 22000.0f) / 22000.0f;	// ADC scale.

constexpr int IMU_MAP[3] = {1, 0, 2};			// IMU orientation mappings.
constexpr bool IMU_REV[3] = {false, false, true};	// IMU orientation directions.
constexpr float IMU_ROT = 0.0f * DEG2RAD;		// IMU rotation.

constexpr int COM_BAUDRATE = 115200;	// Serial communication baudrate [bps].
constexpr int CTL_LOOPFREQ = 4000;	// PID loop frequency (max. 8000) [Hz].
constexpr int SEN_SAMPFREQ = 4000;	// Sensor update loop frequency (max. 8000) [Hz].

#endif
