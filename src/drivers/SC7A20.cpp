#include "drivers/SC7A20.h"
#include "drivers/SC7A20_registers.h"
#include <libraries/delay/nrf_delay.h>
#include <task.h>

using namespace Pinetime::Drivers;

SC7A20::SC7A20(TwiMaster& twiMaster, uint8_t twiAddress) : 
  AccelerationSensor(twiMaster, twiAddress) { }

void SC7A20::Init() {
  // Reset internal memory
  uint8_t data = CTRL_REG5_BOOT;
  Write(CTRL_REG5, &data, 1);
  vTaskDelay(5);
  data = 0;
  Write(CTRL_REG5, &data, 1);

  // Read Chip ID
  Read(WHO_AM_I, &data, 1);
  if(data == 17) {
    deviceType = AccelerationDeviceTypes::SC7A20;
  } else {
    deviceType = AccelerationDeviceTypes::Unknown;
    return;
  }

  // Configure resolution to be +-2g
  data = CTRL_REG4_FS_2G;
  Write(CTRL_REG4, &data, 1);

  // Enable high pass filter for click detection
  data = CTRL_REG2_HPCLICK;
  Write(CTRL_REG2, &data, 1);

  // Enable click event on interrupt function 1 and pin 1
  data = CTRL_REG3_I1_CLICK;
  Write(CTRL_REG3, &data, 1);
  // Set interrupt to active low
  data = CTRL_REG6_H_LACTIVE;
  Write(CTRL_REG6, &data, 1);

  // Configure click treshold, latch interrupt and timings

  data = SC7A20_click_treshold | CLICK_THS_LIR_CLICK;
  Write(CLICK_THS, &data, 1);
  data = SC7A20_click_time_limit;
  Write(TIME_LIMIT, &data, 1);
  data = SC7A20_click_latency;
  Write(TIME_LATENCY, &data, 1);
  data = SC7A20_click_window;
  Write(TIME_WINDOW, &data, 1);

  // Enable block update, configure resolution mode
  data = CTRL_REG4_BDU;
  #if SC7A20_resolution == 12
    data |= CTRL_REG4_HR;
  #endif
  Write(CTRL_REG4, &data, 1);

  // Use FIFO for batch data
  data = CTRL_REG5_FIFO_EN;
  Write(CTRL_REG5, &data, 1);
  data = FIFO_CTRL_REG_FIFO;
  Write(FIFO_CTRL_REG, &data, 1);

  // Enable all axis, configure low power mode if needed
  data = SC7A20_sample_rate | CTRL_REG1_X_EN | CTRL_REG1_Y_EN | CTRL_REG1_Z_EN;
  #if SC7A20_resolution == 8
    data |= CTRL_REG1_LP_EN;
  #endif
  Write(CTRL_REG1, &data, 1);

  isInitialized = true;
}

AccelerationValues SC7A20::Process() {
  if (!isInitialized) return {};
 
  // Read FIFO size, should be about 20 (200 Hz ODR / 10 Hz main loop)
  uint8_t length = 0;
  Read(FIFO_SRC_REG, &length, 1);
  length &= FIFO_SRC_REG_FSS_MASK;

  // Read FIFO samples one by one (full read does not work)
  for(uint8_t i=0; i<length; i++) {
    // Set the most significant bit of the sub-address field for block read
    Read(0x80 | OUT_X_L, (uint8_t*)&fifo[i], sizeof(int16_t) * 3);
    // Shift because value is left-justified
    for(uint8_t j=0; j<3; j++) fifo[i][j] >>= (16 - SC7A20_resolution);
    // X and Y axis are swapped because of the way the sensor is mounted in the P8
    int16_t swap = fifo[i][0];
    fifo[i][0] = fifo[i][1];
    fifo[i][1] = swap;
  }

  // Restart FIFO
  uint8_t data = FIFO_CTRL_REG_BYPASS;
  Write(FIFO_CTRL_REG, &data, 1);
  data = FIFO_CTRL_REG_FIFO;
  Write(FIFO_CTRL_REG, &data, 1);

  // Compute averages of FIFO
  int16_t avgs[3] = { 0 }; 
  // 2g range in n bits
  for(uint8_t i=0; i<length; i++) for(uint8_t j=0; j<3; j++) {
    avgs[j] += ((fifo[i][j] * 2000) / (1 << (SC7A20_resolution - 1)));
  }
  for(uint8_t j=0; j<3; j++) avgs[j] /= length;

  // Step counting is not implemented
  return { 0, avgs[0], avgs[1], avgs[2], (int16_t*)fifo, length };
}
