//
// Created by yashk on 10/29/2023.
//

#include "gps.h"
#include <cstdio>

static char GPS_TX_CMDS[32] = "GPS commands";
static char GPS_RX_CMDS[32] = "GPS response";
// ADD more response buffers if needed

unsigned int Get_GPS_Response(UART_HandleTypeDef* huart, VcuInput* input){
  // TODO: Put in GPS response parsing here using sscanf
  // After using sscanf, add this to input field values
  return 0;
}

unsigned int Send_GPS_Request(UART_HandleTypeDef* huart){
  //TODO: Put in GPS commands here using sprintf
  sprintf(GPS_TX_CMDS, "GPS commands");
  if(HAL_UART_Transmit_IT(huart, (uint8_t*)GPS_TX_CMDS, sizeof(GPS_TX_CMDS)) != HAL_OK){
    return 1;
  }
  return 0;
}
