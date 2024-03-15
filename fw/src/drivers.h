#ifndef DRIVERS_H
#define DRIVERS_H

#ifndef DEF_UNITTEST
  #include "driver_ac.h"
  #include "driver_adc.h"
  #include "driver_clk.h"
  #include "driver_can.h"
  #include "driver_dac.h"
  #include "driver_dma.h"
  #include "driver_dsu.h"
  #include "driver_flash.h"
  #include "driver_gpio.h"
  #include "driver_i2c.h"
  #include "driver_pwm.h"
  #include "driver_spi.h"
  #include "driver_temp.h"
  #include "driver_timer.h"
  #include "driver_uart.h"
  #include "driver_vref.h"
  #include "driver_wdt.h"
#else
  #include "can_structs.h"
  #include "driver_flash.h"
#endif

#endif
