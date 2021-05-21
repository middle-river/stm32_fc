/*
  Receiver module for the HiSky protocol.
  2021-01-12  T. Nakagawa
*/

#include <EEPROM.h>
#include <SPI.h>
#include "Config.h"
#include "Receiver.h"

class NRF24L01 {
private:
  SPIClass *spi_;
  const uint8_t ce_pin_;
  const uint8_t cs_pin_;

  uint8_t readStatus() {
    digitalWrite(cs_pin_, LOW);
    const uint8_t result = spi_->transfer(0xff);
    digitalWrite(cs_pin_, HIGH);
    return result;
  }

  uint8_t readRegister(uint8_t reg) {
    digitalWrite(cs_pin_, LOW);
    spi_->transfer(reg);
    const uint8_t result = spi_->transfer(0xff);
    digitalWrite(cs_pin_, HIGH);
    return result;
  }

  void readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
    digitalWrite(cs_pin_, LOW);
    spi_->transfer(reg);
    spi_->transfer(data, len);
    digitalWrite(cs_pin_, HIGH);
  }

  void writeRegister(uint8_t reg, uint8_t val) {
    digitalWrite(cs_pin_, LOW);
    spi_->transfer(0x20 | reg);
    spi_->transfer(val);
    digitalWrite(cs_pin_, HIGH);
  }

  void writeRegister(uint8_t reg, const uint8_t *data, uint8_t len) {
    digitalWrite(cs_pin_, LOW);
    spi_->transfer(0x20 | reg);
    do spi_->transfer(*data++); while (--len);
    digitalWrite(cs_pin_, HIGH);
  }

public:
  enum CRCType { CT_NONE = 0x03, CT_1BYTE = 0x0b, CT_2BYTE = 0x0f, };
  enum DataRate { DR_1MBPS = 0x00, DR_2MBPS = 0x08, DR_250KBPS = 0x20, };

  NRF24L01(SPIClass *spi, uint8_t ce_pin, uint8_t cs_pin) : spi_(spi), ce_pin_(ce_pin), cs_pin_(cs_pin) {
    pinMode(ce_pin_, OUTPUT);
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(ce_pin_, LOW);
    digitalWrite(cs_pin_, HIGH);
    spi_->begin();
    spi_->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    delayMicroseconds(100000UL);  // Power on reset.
    writeRegister(0x00, 0x00);  // PWR_UP=0.
    writeRegister(0x00, 0x0b);  // PWR_UP=1 and PRIM_RX=1.
    delayMicroseconds(5000UL);  // Tpd2stby.
    writeRegister(0x01, 0x00);  // Disable auto acknowledgment.
    writeRegister(0x04, 0x00);  // Disable auto retransmit.
    writeRegister(0x1c, 0x00);  // Disable dynamic payload length.
    writeRegister(0x1d, 0x00);  // Disable dynamic payload length and Payload with ACK.
  }

  void enable() {
    digitalWrite(ce_pin_, HIGH);  // CE=1.
    delayMicroseconds(130UL);  // RX Settling.
  }

  void disable() {
    digitalWrite(ce_pin_, LOW);  // CE=0.
  }

  bool empty() {
    return (bool)(readRegister(0x17) & 0x01);
  }

  void setCRCType(CRCType crc) {
    writeRegister(0x00, crc);
  }

  void setDataRate(DataRate dr) {
    writeRegister(0x06, dr);
  }

  void setChannel(uint8_t ch) {
    writeRegister(0x05, ch);
  }

  void setPipes(uint8_t pipes) {
    writeRegister(0x02, (uint8_t)(1 << pipes) - 1);  // Enable data pipe 0~(pipes - 1).
  }

  void setAddressWidth(uint8_t aw) {
    writeRegister(0x03, aw - 2);
  }

  void setAddress(uint8_t pipe, const uint8_t *adrs) {
    if (pipe < 2) writeRegister(0x0a + pipe, adrs, readRegister(0x03) + 2); else writeRegister(0x0a + pipe, adrs, 1);
  }

  void setPayloadSize(uint8_t size) {
    for (uint8_t i = 0; i < 6; i++) writeRegister(0x11 + i, size);
  }

  void read(uint8_t *data) {
    readRegister(0x61, data, readRegister(0x11));  // readRegister() works also for R_RX_PAYLOAD.
    writeRegister(0x07, 0xff);  // Clear IRQ bits.
  }

  void flush() {
    writeRegister(0xe2, 0xff);  // writeRegister() works also for FLUSH_RX.
  }
};

struct BindInfo {
  uint8_t address[5];
  uint8_t channels[20];
};

static constexpr int BIND_CHANNEL = 81;
static constexpr uint8_t BIND_ADDRESS[] = "\x12\x23\x23\x45\x78";
static constexpr int PACKET_SIZE = 10;
static constexpr int CHAN_MAP[8] = {0, 1, 3, 2, 5, 6, 4, 7};
static constexpr bool CHAN_REV[8] = {false, false, true, true, false, false, false, false};
static constexpr float ONLINE_TIMEOUT = 1.0;	// No signals for this period means offline [s].
static constexpr float HOP_PERIOD = 0.0095f;    // Maximum frequency hopping period [s].

static SPIClass spi(PIN_TRX_MOSI, PIN_TRX_MISO, PIN_TRX_SCK);
static NRF24L01 radio(&spi, PIN_TRX_CE, PIN_TRX_CS);
static BindInfo bind_info;
static uint8_t hop;
static uint8_t state;  // 0: Start, 1-4: Binding, 5: Bound, 6: Receiving.
static bool rebind = false;

static void switch_channel() {
  radio.disable();
  radio.setChannel(bind_info.channels[hop]);
  radio.enable();
}

Receiver rec;

void rec_init() {
  radio.setCRCType(NRF24L01::CT_2BYTE);
  radio.setDataRate(NRF24L01::DR_1MBPS);
  radio.setPipes(1);
  radio.setAddressWidth(5);
  radio.setPayloadSize(PACKET_SIZE);

  // Load the binding information.
  EEPROM.get(ROM_REC, bind_info);
  if (bind_info.channels[0] == 0xff) state = 0; else state = 5; 
}

void rec_process() {
  static uint16_t adrs_sum;
  static uint32_t count;
  static int wait;
  static uint32_t hop_timer;
  static uint32_t onl_timer;
  static uint32_t stat_timer;

  uint32_t now = micros();

  if (now >= stat_timer) {
    rec.count = count;
    count = 0;
    stat_timer = now + 1000000;
  }

  if (rec.online && now >= onl_timer) {
    rec.online = false;
    for (int i = 0; i < 8; i++) rec.value[i] = 0.0f;
  }

  if (rebind) {
    rebind = false;
    rec_reset();
    state = 0;
  }

  if (state == 0) {
    radio.disable();
    radio.setAddress(0, BIND_ADDRESS);
    radio.setChannel(BIND_CHANNEL);
    radio.enable();
    radio.flush();
    adrs_sum = 0;
    state++;
    return;
  } else if (state == 5) {
    wait = 0;
    hop = 0;
    radio.disable();
    radio.setAddress(0, bind_info.address);
    switch_channel();
    hop_timer = now + (uint32_t)(HOP_PERIOD * 1.0e6f);
    radio.flush();
    state++;
    return;
  }

  if (radio.empty()) {
    if (state == 6 && now >= hop_timer) {
      wait = min(max(1, 2 * wait), 21);
      hop = (hop + wait) % 20;
      switch_channel();
      hop_timer = now + wait * (uint32_t)(HOP_PERIOD * 1.0e6f);
    }

    return;
  }

  uint8_t buf[PACKET_SIZE];
  radio.read(buf);

  if (state == 1) {
    if (buf[0] == 0xff && buf[1] == 0xaa && buf[2] == 0x55) {
      for (int i = 0; i < 5; i++) adrs_sum += (bind_info.address[i] = buf[3 + i]);
      state++;
    }
  } else if (state <= 4) {
    if (buf[0] == (adrs_sum & 0xff) && buf[1] == (adrs_sum >> 8) && buf[2] == state - 2) {
      for (int i = 0; i < ((state != 4) ? 7 : 6); i++) bind_info.channels[7 * (state - 2) + i] = buf[3 + i];
      if (state == 4) {
        EEPROM.put(ROM_REC, bind_info);
      }
      state++;
    }
  } else {
    count++;
    wait = 0;
    hop = (hop + 1) % 20;
    switch_channel();
    hop_timer = now + (uint32_t)(HOP_PERIOD * 1.0e6f);

    uint16_t high = *(uint16_t *)(buf + 8);
    for (int8_t i = 0; i < 8; i++) {
      uint16_t val = (((high & 0x0003) << 8) | buf[i]);
      high >>= 2;
      if (i == 6 && val >= 3) val = 1000;
      val = constrain(val, 0, 1000);
      if (CHAN_REV[i]) val = 1000 - val;
      rec.value[CHAN_MAP[i]] = val / 1000.0f;
    }
    rec.online = true;
    onl_timer = now + (uint32_t)(ONLINE_TIMEOUT * 1.0e6f);
  }
}

void rec_rebind() {
  rebind = true;
}

void rec_reset() {
  bind_info.channels[0] = 0xff;
  EEPROM.put(ROM_REC, bind_info);
}
