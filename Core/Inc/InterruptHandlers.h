//
// Created by yashk on 10/24/2023.
//

#ifndef VCU_FIRMWARE_2024_INTERRUPTHANDLERS_H
#define VCU_FIRMWARE_2024_INTERRUPTHANDLERS_H

#include "stm32h7xx_hal.h"
#include "globals.h"

static FDCAN_TxHeaderTypeDef Tx0Header;
static FDCAN_RxHeaderTypeDef Rx0Header;
static FDCAN_TxHeaderTypeDef Tx1Header;
static FDCAN_RxHeaderTypeDef Rx1Header;

static uint8_t Tx0Data[CAN_DATA_SIZE] = {0};
static uint8_t Rx0Data[CAN_DATA_SIZE] = {0};
static uint8_t Rx1Data[CAN_DATA_SIZE] = {0};
static uint8_t Tx1Data[CAN_DATA_SIZE] = {0};

static uint8_t HVCData[CAN_DATA_SIZE] = {0};
static int8_t INVTemp1Data[CAN_DATA_SIZE] = {0};
static int8_t INVTemp3Data[CAN_DATA_SIZE] = {0};
static uint8_t INVStateData[CAN_DATA_SIZE] = {0};
static uint8_t INVFaultData[CAN_DATA_SIZE] = {0};
static uint8_t INVParamsData[CAN_DATA_SIZE] = {0};
static uint8_t PDUData[CAN_DATA_SIZE] = {0};
static uint8_t WHSData[CAN_DATA_SIZE] = {0};
static uint8_t IMU2Data[CAN_DATA_SIZE] = {0};
static uint8_t IMU3Data[CAN_DATA_SIZE] = {0};

#endif //VCU_FIRMWARE_2024_INTERRUPTHANDLERS_H
