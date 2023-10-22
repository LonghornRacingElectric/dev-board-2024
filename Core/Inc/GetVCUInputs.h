//
// Created by yashk on 10/16/2023.
//

#ifndef VCU_FIRMWARE_2024_GETVCUINPUTS_H
#define VCU_FIRMWARE_2024_GETVCUINPUTS_H

#include "firmware_faults.h"
#include "VcuModel.h"
#include "library.h"
#include "stm32h7xx_hal.h"

#define CAN_DATA_SIZE 8
#define ADC_BUF_SIZE 5
#define APPS1_CHANNEL 0
#define APPS2_CHANNEL 1
#define BSE1_CHANNEL 2
#define BSE2_CHANNEL 3
#define STEER_CHANNEL 4

#define UART_BUF_SIZE 32
#define SPI_BUF_SIZE 16


uint32_t Get_VCU_Inputs(VcuInput* , VcuParameters*, ADC_HandleTypeDef* , FDCAN_HandleTypeDef* , SPI_HandleTypeDef* , UART_HandleTypeDef* );
void init_TX(uint32_t);
int update_HVC(VcuInput* );
int update_INV(VcuInput* );
int update_PDU(VcuInput* );
int update_WHS(VcuInput* );
int update_IMU(VcuInput* );
int Critical_Error_Handler(uint32_t );
int Noncritical_Error_Handler(uint32_t );



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

#endif //VCU_FIRMWARE_2024_GETVCUINPUTS_H
