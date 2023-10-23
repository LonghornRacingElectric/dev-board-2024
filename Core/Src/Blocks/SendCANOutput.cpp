//
// Created by yashk on 10/16/2023.
//

#include "SendCANOutput.h"

FDCAN_TxHeaderTypeDef CAN_OUT_TxHeader;

#define VCU_INV_COMMAND 0x0C0 //Stores inverter command
#define VCU_INV_PARAMETER_RW 0x0C1 //Sets inverter parameter r/w
#define VCU_PDU_BRAKING 0x0F0 //Stores PDU braking data

int8_t CAN_OUT_TxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint32_t Send_CAN_Output(VcuOutput* output, VcuParameters* params, BSPD* bspd, FDCAN_HandleTypeDef* hfdcan) {

  //Send out Torque Command
  auto torqueCommand = (int16_t) output->inverterTorqueRequest;
  CAN_OUT_TxData[0] = (int8_t) (torqueCommand >> 8);
  CAN_OUT_TxData[1] = (int8_t) (torqueCommand & 0x0FF);
  CAN_OUT_TxData[4] = 1;
  CAN_OUT_TxData[5] = 0x01; //Maybe change this to depend on global_shutdown and inverterReady
  //TODO: Add torque limit later

  init_TX(&CAN_OUT_TxHeader, VCU_INV_COMMAND);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(CAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(TORQUEREQUEST_DATA_FAULT);
  }

  CAN_OUT_TxData[0] = output->r2dBuzzer;
  CAN_OUT_TxData[1] = output->prndlState;
  CAN_OUT_TxData[2] = output->enableDragReduction;
  CAN_OUT_TxData[3] = bspd->BrakePressed;
  CAN_OUT_TxData[4] = bspd->BrakeBroken;
  CAN_OUT_TxData[5] = bspd->MotorON;
  CAN_OUT_TxData[6] = bspd->MotorPressed;
  CAN_OUT_TxData[7] = 0xFF;

  init_TX(&CAN_OUT_TxHeader, VCU_PDU_BRAKING);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(CAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(PDU_DATA_FAULT);
  }
  return 0;
}