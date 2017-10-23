/*
 * Copyright 2016 <Admobilize>
 * All rights reserved.
 */

#ifndef CPP_CREATOR_MCU_DATA_H_
#define CPP_CREATOR_MCU_DATA_H_

#include "chtypes.h"

const int16_t mem_offset_env = 0x00;
const int16_t mem_offset_imu = 0x30;
const int16_t mem_offset_mcu = 0x90;

struct EnvData {
  int UV;
  int altitude;
  int pressure;
  int temperature_mpl;
  int humidity;
  int temperature_hts;
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
  float mag_offset_x;
  float mag_offset_y;
  float mag_offset_z;
};

struct MCUData {
  uint32_t ID;
  uint32_t version;
};


#endif  // CPP_DRIVER_PRESSURE_DATA_H_
