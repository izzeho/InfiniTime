#include "drivers/Bma421.h"
#include "drivers/TwiMaster.h"
#include <drivers/Bma421_C/bma423.h>
#include <libraries/delay/nrf_delay.h>

using namespace Pinetime::Drivers;
namespace {
  int8_t user_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void* intf_ptr) {
    auto bma421 = static_cast<Bma421*>(intf_ptr);
    bma421->Read(reg_addr, reg_data, length);
    return 0;
  }

  int8_t user_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length, void* intf_ptr) {
    auto bma421 = static_cast<Bma421*>(intf_ptr);
    bma421->Write(reg_addr, reg_data, length);
    return 0;
  }

  void user_delay(uint32_t period_us, void* intf_ptr) {
    nrf_delay_us(period_us);
  }
}

Bma421::Bma421(TwiMaster& twiMaster, uint8_t twiAddress) : 
  AccelerationSensor(twiMaster, twiAddress) {
  bma.intf = BMA4_I2C_INTF;
  bma.bus_read = user_i2c_read;
  bma.bus_write = user_i2c_write;
  bma.variant = BMA42X_VARIANT;
  bma.intf_ptr = this;
  bma.delay_us = user_delay;
  bma.read_write_len = 16;
  bma.resolution = 12;
}

void Bma421::Init() {
  if (!isResetOk) return; // Call SoftReset (and reset TWI device) first!

  // Initialize interface
  auto ret = bma423_init(&bma);
  if (ret != BMA4_OK) return;

  // Identify chip by ID. The driver code has been modified to handle BMA421 as BMA423 
  switch(bma.chip_id) {
    case BMA423_CHIP_ID: deviceType = AccelerationDeviceTypes::BMA421; break;
    case BMA425_CHIP_ID: deviceType = AccelerationDeviceTypes::BMA425; break;
    default: deviceType = AccelerationDeviceTypes::Unknown; break;
  }

  // Load proprietary firmware blob required for step counting engine
  ret = bma423_write_config_file(&bma);
  if (ret != BMA4_OK) return;

  // Enable step counter and accelerometer, disable step detector
  ret = bma423_feature_enable(BMA423_STEP_CNTR | BMA423_SINGLE_TAP | BMA423_DOUBLE_TAP, 1, &bma);
  if (ret != BMA4_OK) return;
  ret = bma423_step_detector_enable(0, &bma);
  if (ret != BMA4_OK) return;
  ret = bma4_set_accel_enable(1, &bma);
  if (ret != BMA4_OK) return;

  // Configure tap sensitivity
  if(bma423_single_tap_set_sensitivity(0, &bma) != BMA4_OK) return;
  if(bma423_double_tap_set_sensitivity(0, &bma) != BMA4_OK) return;

  // Configure and enable interrupt INT1
  if(bma4_set_interrupt_mode(BMA4_LATCH_MODE, &bma) != BMA4_OK) return;
  struct bma4_int_pin_config int_config;
  int_config.edge_ctrl = BMA4_LEVEL_TRIGGER;
  int_config.lvl = BMA4_ACTIVE_LOW;
  int_config.od = BMA4_OPEN_DRAIN;
  int_config.output_en = BMA4_OUTPUT_ENABLE;
  int_config.input_en = BMA4_INPUT_DISABLE;
  if(bma4_set_int_pin_config(&int_config, BMA4_INTR1_MAP, &bma) != BMA4_OK) return;
  SetMotion(MotionEvents::None);

  // Configure FIFO
  ret = bma4_set_fifo_config(BMA4_FIFO_ACCEL, 1, &bma);
  if (ret != BMA4_OK) return;
  fifo_frame.data = (uint8_t*)&fifo;
  fifo_frame.length = sizeof(fifo);
  fifo_frame.fifo_data_enable = BMA4_FIFO_A_ENABLE;
  fifo_frame.fifo_header_enable = 0;

  // Configure accelerometer
  struct bma4_accel_config accel_conf;
  accel_conf.odr = BMA4_OUTPUT_DATA_RATE_200HZ;
  accel_conf.range = BMA4_ACCEL_RANGE_2G;
  accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
  accel_conf.perf_mode = BMA4_CONTINUOUS_MODE;
  ret = bma4_set_accel_config(&accel_conf, &bma);
  if (ret != BMA4_OK) return;

  isInitialized = true;
}

AccelerationValues Bma421::Process() {
  if (!isInitialized) return {};

  // Dump entire FIFO into buffer
  bma4_read_fifo_data(&fifo_frame, &bma);
  // Decode FIFO frames in-place sequentially
  uint16_t length = 32; // Should be about 20 (200 Hz ODR / 10 Hz main loop)
  bma4_extract_accel((bma4_accel*)fifo, &length, &fifo_frame, &bma);

  // Read step counter engine
  uint32_t steps = 0;
  bma423_step_counter_output(&steps, &bma);

  // Read temperature sensor
  int32_t temperature = 0;
  bma4_get_temperature(&temperature, BMA4_DEG, &bma);
  temperature = temperature / 1000;

  // Read sport activity mode
  uint8_t activity = 0;
  bma423_activity_output(&activity, &bma);

  // Compute averages of FIFO
  int16_t avgs[3] = { 0 }; 
  for(uint8_t i=0; i<length; i++) {
    // X and Y axis are swapped because of the way the sensor is mounted in the PineTime
    int16_t swap = fifo[i][0];
    fifo[i][0] = fifo[i][1];
    fifo[i][1] = swap;
    for(uint8_t j=0; j<3; j++) avgs[j] += fifo[i][j];
  }
  for(uint8_t j=0; j<3; j++) avgs[j] /= length;

  return {steps, avgs[0], avgs[1], avgs[2], (int16_t*)fifo, length};
}

void Bma421::ResetStepCounter() {
  bma423_reset_step_counter(&bma);
}

void Bma421::SoftReset() {
  auto ret = bma4_soft_reset(&bma);
  if (ret == BMA4_OK) {
    isResetOk = true;
    nrf_delay_ms(1);
  }
}

void Bma421::SetMotion(MotionEvents event) {
  #if defined(DRIVER_WAKE_ACC)
    uint8_t map = 0;
    if(event == MotionEvents::SingleTap) {
      map = BMA423_SINGLE_TAP_INT;
    } else if(event == MotionEvents::DoubleTap) {
      map = BMA423_DOUBLE_TAP_INT;
    }

    bma423_map_interrupt(BMA4_INTR1_MAP, map, 1, &bma);
  #endif
}

MotionEvents Bma421::GetMotionInfo() {
  #if defined(DRIVER_WAKE_ACC)
    uint16_t int_status = 0;
    if (bma423_read_int_status(&int_status, &bma) == BMA4_OK) {
      if (int_status & BMA423_SINGLE_TAP_INT) {
        return MotionEvents::SingleTap;
      } else if (int_status & BMA423_DOUBLE_TAP_INT) {
        return MotionEvents::DoubleTap;
      }
    }
  #endif

  return MotionEvents::None;
}
