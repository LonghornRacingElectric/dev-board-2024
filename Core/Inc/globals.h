//
// Created by yashk on 10/22/2023.
//

#ifndef VCU_FIRMWARE_2024_GLOBALS_H
#define VCU_FIRMWARE_2024_GLOBALS_H

#include <cstdint>
#include "stm32h7xx_hal.h"

static uint32_t global_shutdown = 0;
#define UART_BUF_SIZE 32
#define SPI_BUF_SIZE 16
#define CAN_DATA_SIZE 8

#define INV_TEMP1_DATA 0x0A0 //Stores inverter module temperature
#define INV_TEMP3_DATA 0x0A2 //Stores motor temperature
#define INV_MOTOR_POSITIONS 0x0A5 //Stores motor position
#define INV_CURRENT 0x0A6 //Stores inverter current
#define INV_VOLTAGE 0x0A7 //Stores inverter voltage
#define INV_STATE_CODES 0x0AA //Stores inverter state codes
#define INV_FAULT_CODES 0x0AB //Stores inverter fault codes
#define INV_TORQUE_TIMER 0x0AC //Stores inverter torque results
#define INV_HIGH_SPEED_MSG 0x0B0 //Stores inverter high speed message
#define VCU_INV_COMMAND 0x0C0 //Stores inverter command
#define VCU_INV_PARAMETER_RW 0x0C1 //Sets inverter parameter r/w
#define INV_VCU_RESPONSE_RW 0x0C2 //Responds back success of parameter r/w

#define VCU_REQUEST_DATA_ID 0x100
#define HVC_RESPONSE_ID 0x110
#define HVC_IMU2_RESPONSE_ID 0x111
#define INV_RESPONSE_ID 0x120
#define PDU_RESPONSE_ID 0x130
#define PDU_IMU3_RESPONSE_ID 0x131
#define WHS1_RESPONSE_ID 0x140
#define WHS2_RESPONSE_ID 0x150
#define WHS3_RESPONSE_ID 0x160
#define WHS4_RESPONSE_ID 0x170

typedef struct bspd {
  int BrakePressed;
  int BrakeBroken;
  int MotorON;
  int MotorPressed;
  int BSPDShutdown;
} BSPD;

int Critical_Error_Handler(uint32_t fault_type);
int Noncritical_Error_Handler(uint32_t fault_type);
void init_TX(FDCAN_TxHeaderTypeDef*, uint32_t);

typedef struct MotorInfo {
  int16_t motorAngle;
  int16_t motorVelocity;
  int16_t resolverAngle;
} motor_info_t;

typedef struct InverterInfo {
  int16_t busCurrent;
  int16_t phaseACurrent;
  int16_t phaseBCurrent;
  int16_t phaseCCurrent;

  int16_t busVoltage;
  int16_t BCVoltage;
  int16_t ABVoltage;
  int16_t outputVoltage;

  int16_t inverterFrequency;

  int16_t torqueFeedback;
  int16_t torqueCommand;

} inverter_info_t;

static motor_info_t motorInfo;
static inverter_info_t inverterInfo;

#endif //VCU_FIRMWARE_2024_GLOBALS_H
