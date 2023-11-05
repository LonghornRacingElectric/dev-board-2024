//
// Created by yashk on 10/29/2023.
//

#include "imu_internal.h"
#include <cstdio>

static uint8_t IMU_TX_CMDS[32] = {0};
static uint8_t IMU_RX_CMDS[32] = {0};

unsigned int Get_IMU_Response(SPI_HandleTypeDef* hspi, VcuInput* input) {
  // TODO: Put in IMU response parsing here using sscanf
  // After using sscanf, add this to input field values

  return 0;
}

unsigned int Send_IMU_Request(SPI_HandleTypeDef* hspi) {
  //TODO: Put in IMU commands here using whatever
  if(HAL_SPI_Transmit_DMA(hspi, (uint8_t*)IMU_TX_CMDS, sizeof(IMU_TX_CMDS)) != HAL_OK){
    return 1;
  }
  /* Note: this will call an interrupt, but the DMA interrupt does not exist.
  The data will automatically be allocated to IMU_RX_CMDS */
  if(HAL_SPI_Receive_DMA(hspi, (uint8_t*)IMU_RX_CMDS, sizeof(IMU_RX_CMDS)) != HAL_OK){
    return 1;
  }
  return 0;
}


