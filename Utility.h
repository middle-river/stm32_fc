/*
  Utility module.
  2021-01-12  T. Nakagawa
*/

#ifndef UTILITY_H_
#define UTILITY_H_

constexpr float DEG2RAD = 2.0f * (float)M_PI / 360.0f;
constexpr float RAD2DEG = 360.0f / (2.0f * (float)M_PI);

// Low pass filter by exponential moving average.
class LPF_EMA {
private:
  float alpha_;
  float state_;

public:
  LPF_EMA() {
  }

 void reset(float cutoff_freq, float sampling_freq) {
    const float c = cosf(2.0f * (float)M_PI * cutoff_freq / sampling_freq);
    alpha_ = c - 1.0f + sqrtf(c * c - 4.0f * c + 3.0f);
    state_ = 0.0f;
  }

  float filter(float value) {
    state_ = (1.0f - alpha_) * state_ + alpha_ * value;
    return state_;
  }
};

// Low pass filter by the 2-stage biquad filter.
class LPF_BQ2 {
private:
  static constexpr int STAGE = 2;
  float a_[3];
  float b_[3];
  float state_[STAGE][2];

public:
  LPF_BQ2() {
  }

  void reset(float cutoff_freq, float sampling_freq) {
    const float q = 1.0f / sqrtf(2.0f);
    const float omega = 2.0f * (float)M_PI * cutoff_freq / sampling_freq;
    const float alpha = sinf(omega) / (2.0f * q);
    a_[0] = 1.0f + alpha;
    a_[1] = -2.0f * cosf(omega);
    a_[2] = 1.0f - alpha;
    b_[0] = (1.0f - cosf(omega)) / 2.0f;
    b_[1] = 1.0f - cosf(omega);
    b_[2] = (1.0f - cosf(omega)) / 2.0f;
    const float norm = a_[0];
    for (int i = 0; i < 3; i++) {
      a_[i] /= norm;
      b_[i] /= norm;
    }
    for (int i = 0; i < STAGE; i++) state_[i][0] = state_[i][1] = 0.0f;
  }

  float filter(float value) {
    float acc, res = value;
    for (int i = 0; i < STAGE; i++) {
      acc = res - a_[1] * state_[i][0] - a_[2] * state_[i][1];
      res = b_[0] * acc + b_[1] * state_[i][0] + b_[2] * state_[i][1];
      state_[i][1] = state_[i][0]; state_[i][0] = acc;
    }
    return res;
  }
};

// Pulse width modulation.
class PWM {
private:
  TimerCompareFormat_t resolution_;
  HardwareTimer *ht_;
  uint32_t channel_;

public:
  PWM() {
  }

  void begin(PinName pin, uint32_t pwm_freq, uint32_t value, TimerCompareFormat_t resolution) {
    TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    const uint32_t index = get_timer_index(instance);
    if (HardwareTimer_Handle[index] == NULL) {
      ht_ = new HardwareTimer(instance);
    } else {
      ht_ = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
    }
    channel_ = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    resolution_ = resolution;
    ht_->setMode(channel_, TIMER_OUTPUT_COMPARE_PWM1, pin);
    ht_->setOverflow(pwm_freq, HERTZ_FORMAT);
    ht_->setCaptureCompare(channel_, value, resolution_);
    ht_->resume();
  }

  void write(uint32_t value) {
    ht_->setCaptureCompare(channel_, value, resolution_);
  }
};

// Interval timer.
inline void timer(TIM_TypeDef *instance, uint32_t frequency, uint32_t priority, void (*callback)()) {
  HardwareTimer *tim = new HardwareTimer(instance);
  tim->setMode(1, TIMER_OUTPUT_COMPARE, NC);
  tim->setOverflow(frequency, HERTZ_FORMAT);
  tim->attachInterrupt(callback);
  tim->setInterruptPriority(priority, 0);
  tim->resume();
}

#endif
