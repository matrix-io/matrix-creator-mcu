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
  int accel_x;
  int accel_y;
  int accel_z;
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int mag_x;
  int mag_y;
  int mag_z;
  int mag_offset_x;
  int mag_offset_y;
  int mag_offset_z;
  float yaw;
  float pitch;
  float roll;
};

struct MCUData {
  uint32_t ID;
  uint32_t version;
};

#endif  // CPP_DRIVER_PRESSURE_DATA_H_
