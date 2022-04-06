#pragma once

#include "drivers/AccelerationSensor.h"

#define SC7A20_resolution 12 // Output data resolution, options: 8, 10, 12 bit
#define SC7A20_sample_rate CTRL_REG1_ODR_200HZ // 200 Hz Output data rate
#define SC7A20_click_treshold 48 // 48 * FS / 128 = 750 mg where FS is ±2 g
#define SC7A20_click_time_limit 21 // 21 * 1 / ODR = 60 ms where ODR is 200 Hz
#define SC7A20_click_latency 6 // 6 * 1 / ODR = 30 ms where ODR is 200 Hz
#define SC7A20_click_window 56 // 56 * 1 / ODR = 280 ms where ODR is 200 Hz

namespace Pinetime {
  namespace Drivers {
    class SC7A20: public AccelerationSensor {
    public:
      SC7A20(TwiMaster& twiMaster, uint8_t twiAddress);
      SC7A20(const SC7A20&) = delete;
      SC7A20& operator=(const SC7A20&) = delete;
      SC7A20(SC7A20&&) = delete;
      SC7A20& operator=(SC7A20&&) = delete;

      void Init();
      AccelerationValues Process();
      void SetMotion(MotionEvents event);
      MotionEvents GetMotionInfo();
    };
  }
}
