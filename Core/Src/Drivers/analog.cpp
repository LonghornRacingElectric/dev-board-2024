//
// Created by yashk on 10/29/2023.
//

#include "analog.h"

unsigned int Init_Analog(ADC_HandleTypeDef* hadc){

  if(HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK){
    return 1;
  }
  // Start polling ADC Data
  if(HAL_ADC_Start_DMA(hadc, (uint32_t*)adcData, 5) != HAL_OK){
    return 1;
  }

  return 0;
}

unsigned int Get_Analog(ADC_HandleTypeDef* hadc, VcuInput* input, VcuParameters* params){

  input->apps1 = (float) (adcData[APPS1_CHANNEL] * 3.3 / 65536.0);
  input->apps2 = (float) (adcData[APPS2_CHANNEL] * 3.3 / 65536.0);
  input->bse1 = (float) (adcData[BSE1_CHANNEL] * 3.3 / 65536.0);
  input->bse2 = (float) (adcData[BSE2_CHANNEL] * 3.3 / 65536.0);
  input->steeringWheelPotVoltage = (float) (adcData[STEER_CHANNEL] * 3.3 / 65536.0);

  return 0;
}
