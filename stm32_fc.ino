#include "Communication.h"
#include "Config.h"
#include "Controller.h"
#include "Receiver.h"
#include "Sensor.h"
#include "System.h"
#include "Utility.h"

void setup() {
  com_init();
  ctl_init();
  rec_init();
  sen_init();
  sys_init();

  timer( TIM2, COM_BAUDRATE / 9, 4, com_process);
  timer( TIM3,     CTL_LOOPFREQ, 2, ctl_process);
  timer(TIM15,            10000, 3, rec_process);
  timer(TIM16,     SEN_SAMPFREQ, 2, sen_process);
  timer(TIM17,                8, 3, sys_process);
}

void loop() {
}
