//
// Created by yashk on 10/24/2023.
//

#include "InterruptHandlers.h"
#include "globals.h"
#include "firmware_faults.h"
#include "VcuModel.h"
#include "library.h"
#include <algorithm>

using namespace std;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  // Checks if the interrupt is from the correct FIFO
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET){
    return;
  }
  // Gets the message from the CAN buffer, and stores it in Rx0Header and Rx0Data, returns an error if it fails
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rx0Header, Rx0Data) != HAL_OK)
  {
    /* Reception Error */
    Critical_Error_Handler(GENERIC_CAN_DATA_FAULT);
    return;
  }
  // Checks if interrupts are still enabled
  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Critical_Error_Handler(VCU_DATA_FAULT);
    return;
  }
  // Run the code that corresponds to the CAN ID
  switch (Rx0Header.Identifier){
    case VCU_REQUEST_DATA_ID:
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      break;
    case HVC_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), HVCData);
      break;

    case INV_TEMP1_DATA:
      copy(begin(Rx0Data), end(Rx0Data), INVTemp1Data);
      break;
    case INV_TEMP3_DATA:
      copy(begin(Rx0Data), end(Rx0Data), INVTemp3Data);
      break;
    case INV_MOTOR_POSITIONS:
      motorInfo.motorAngle = ((Rx0Data[1] << 8) + Rx0Data[0]) * 10;
      motorInfo.motorVelocity = ((Rx0Data[3] << 8) + Rx0Data[2]);
      inverterInfo.inverterFrequency = ((Rx0Data[5] << 8) + Rx0Data[4]) * 10;
      motorInfo.resolverAngle = ((Rx0Data[7] << 8) + Rx0Data[6]) * 10;
      break;
    case INV_CURRENT:
      inverterInfo.phaseACurrent = ((Rx0Data[1] << 8) + Rx0Data[0]) * 10;
      inverterInfo.phaseBCurrent = ((Rx0Data[3] << 8) + Rx0Data[2]) * 10;
      inverterInfo.phaseCCurrent = ((Rx0Data[5] << 8) + Rx0Data[4]) * 10;
      inverterInfo.busCurrent = ((Rx0Data[7] << 8) + Rx0Data[6]) * 10;
      break;
    case INV_VOLTAGE:
      inverterInfo.busVoltage = ((Rx0Data[1] << 8) + Rx0Data[0]) * 10;
      inverterInfo.outputVoltage = ((Rx0Data[3] << 8) + Rx0Data[2]) * 10;
      inverterInfo.ABVoltage = ((Rx0Data[5] << 8) + Rx0Data[4]) * 10;
      inverterInfo.BCVoltage = ((Rx0Data[7] << 8) + Rx0Data[6]) * 10;
      break;
    case INV_STATE_CODES:
      copy(begin(Rx0Data), end(Rx0Data), INVStateData);
      break;
    case INV_FAULT_CODES:
      copy(begin(Rx0Data), end(Rx0Data), INVFaultData);
      break;
    case INV_TORQUE_TIMER:
      inverterInfo.torqueCommand = ((Rx0Data[1] << 8) + Rx0Data[0]) * 10;
      inverterInfo.torqueFeedback = ((Rx0Data[3] << 8) + Rx0Data[2]) * 10;
      break;
    case INV_HIGH_SPEED_MSG:
      inverterInfo.torqueCommand = ((Rx0Data[0] << 8) + Rx0Data[1]) * 10;
      inverterInfo.torqueFeedback = ((Rx0Data[2] << 8) + Rx0Data[3]) * 10;
      motorInfo.motorVelocity = (Rx0Data[4] << 8) + Rx0Data[5];
      inverterInfo.busVoltage = ((Rx0Data[6] << 8) + Rx0Data[7]) * 10;
      break;
    case INV_VCU_RESPONSE_RW:
      copy(begin(Rx0Data), end(Rx0Data), INVParamsData);
      break;

    case PDU_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), PDUData);
      break;
    case WHS1_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), WHSData);
      break;
    case WHS2_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), WHSData + 4);
      break;
    case WHS3_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), WHSData + 8);
      break;
    case WHS4_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), WHSData + 12);
      break;
    case HVC_IMU2_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), IMU2Data);
      break;
    case PDU_IMU3_RESPONSE_ID:
      copy(begin(Rx0Data), end(Rx0Data), IMU3Data);
      break;
    default:
      break;
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET){
    return;
  }
  // Gets the message from the CAN buffer, and stores it in Rx1Header and Rx1Data, returns an error if it fails
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Rx1Header, Rx1Data) != HAL_OK)
  {
    /* Reception Error */
    Critical_Error_Handler(GENERIC_CAN_DATA_FAULT);
    return;
  }
  // Checks if interrupts are still enabled
  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Critical_Error_Handler(VCU_DATA_FAULT);
    return;
  }
  // Run the code that corresponds to the CAN ID
  switch (Rx1Header.Identifier){
    case HVC_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), HVCData);
      break;

    case INV_TEMP1_DATA:
      copy(begin(Rx1Data), end(Rx1Data), INVTemp1Data);
      break;
    case INV_TEMP3_DATA:
      copy(begin(Rx1Data), end(Rx1Data), INVTemp3Data);
      break;
    case INV_MOTOR_POSITIONS:
      motorInfo.motorAngle = ((Rx1Data[0] << 8) + Rx1Data[1]) * 10;
      motorInfo.motorVelocity = ((Rx1Data[2] << 8) + Rx1Data[3]);
      inverterInfo.inverterFrequency = ((Rx1Data[4] << 8) + Rx1Data[5]) * 10;
      motorInfo.resolverAngle = ((Rx1Data[6] << 8) + Rx1Data[7]) * 10;
      break;
    case INV_CURRENT:
      inverterInfo.phaseACurrent = ((Rx1Data[0] << 8) + Rx1Data[1]) * 10;
      inverterInfo.phaseBCurrent = ((Rx1Data[2] << 8) + Rx1Data[3]) * 10;
      inverterInfo.phaseCCurrent = ((Rx1Data[4] << 8) + Rx1Data[5]) * 10;
      inverterInfo.busCurrent = ((Rx1Data[6] << 8) + Rx1Data[7]) * 10;
      break;
    case INV_VOLTAGE:
      inverterInfo.busVoltage = ((Rx1Data[0] << 8) + Rx1Data[1]) * 10;
      inverterInfo.outputVoltage = ((Rx1Data[2] << 8) + Rx1Data[3]) * 10;
      inverterInfo.ABVoltage = ((Rx1Data[4] << 8) + Rx1Data[5]) * 10;
      inverterInfo.BCVoltage = ((Rx1Data[6] << 8) + Rx1Data[7]) * 10;
      break;
    case INV_STATE_CODES:
      copy(begin(Rx1Data), end(Rx1Data), INVStateData);
      break;
    case INV_FAULT_CODES:
      copy(begin(Rx1Data), end(Rx1Data), INVFaultData);
      break;
    case INV_TORQUE_TIMER:
      inverterInfo.torqueCommand = ((Rx1Data[0] << 8) + Rx1Data[1]) * 10;
      inverterInfo.torqueFeedback = ((Rx1Data[2] << 8) + Rx1Data[3]) * 10;
      break;
    case INV_HIGH_SPEED_MSG:
      inverterInfo.torqueCommand = ((Rx1Data[0] << 8) + Rx1Data[1]) * 10;
      inverterInfo.torqueFeedback = ((Rx1Data[2] << 8) + Rx1Data[3]) * 10;
      motorInfo.motorVelocity = (Rx1Data[4] << 8) + Rx1Data[5];
      inverterInfo.busVoltage = ((Rx1Data[6] << 8) + Rx1Data[7]) * 10;
      break;
    case INV_VCU_RESPONSE_RW:
      copy(begin(Rx1Data), end(Rx1Data), INVParamsData);
      break;

    case PDU_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), PDUData);
      break;
    case WHS1_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), WHSData);
      break;
    case WHS2_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), WHSData + 4);
      break;
    case WHS3_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), WHSData + 8);
      break;
    case WHS4_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), WHSData + 12);
      break;
    case HVC_IMU2_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), IMU2Data);
      break;
    case PDU_IMU3_RESPONSE_ID:
      copy(begin(Rx1Data), end(Rx1Data), IMU3Data);
      break;
    default:
      break;
  }

}
