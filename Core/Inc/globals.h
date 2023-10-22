//
// Created by yashk on 10/22/2023.
//

#ifndef VCU_FIRMWARE_2024_GLOBALS_H
#define VCU_FIRMWARE_2024_GLOBALS_H

#include <cstdint>

static uint32_t global_shutdown = 0;
typedef struct {
  int BrakePressed;
  int BrakeBroken;
  int MotorON;
  int MotorPressed;
  int BSPDShutdown;
} BSPD;

#endif //VCU_FIRMWARE_2024_GLOBALS_H
