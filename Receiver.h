/*
  Receiver module.
  2021-01-12  T. Nakagawa
*/

#ifndef RECEIVER_H_
#define RECEIVER_H_

struct Receiver {
  float value[8];	// [0.0, 1.0] for ROLL, PITCH, YAW, THROT, AUX1, AUX2, AUX3, AUX4.
  bool online;
  uint32_t count;
};

extern Receiver rec;

void rec_init();	// Initialize the receiver.
void rec_process();	// Process the receiver.
void rec_rebind();	// Request rebinding.
void rec_reset();	// Reset binding.

#endif
