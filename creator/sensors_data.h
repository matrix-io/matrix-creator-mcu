/*
 * Copyright 2016 <Admobilize>
 * All rights reserved.
 */

#ifndef CPP_CREATOR_MCU_DATA_H_
#define CPP_CREATOR_MCU_DATA_H_

#include "chtypes.h"

const int16_t mem_offset_uv = 0x0;
const int16_t mem_offset_press = 0x10;
const int16_t mem_offset_humidity = 0x20;
const int16_t mem_offset_imu = 0x30;
const int16_t mem_offset_calib = 0x50;
const int16_t mem_offset_control = 0x60;
const int16_t mem_offset_mcu = 0x90;

const int32_t OFFSET_WRITE_ENABLE = 0x25352535;
const int32_t OFFSET_READ_ENABLE = 0x25352534;

struct UVData {
  int UV;
};

struct PressureData {
  float altitude;
  float pressure;
  float temperature;
};

struct HumidityData {
  float humidity;
  float temperature;
};

struct IMUData {
  float yaw;
  float pitch;
  float roll;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
};

struct IMUControl {
  int32_t mag_offset_wr_flag;
};

struct IMUCalibrationData {
  int32_t mag_offset_x;
  int32_t mag_offset_y;
  int32_t mag_offset_z;
};

struct MCUData {
  uint32_t ID;
  uint32_t version;
};


#endif  // CPP_DRIVER_PRESSURE_DATA_H_
