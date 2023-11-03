//
// Created by yashk on 10/29/2023.
//

#include "analog.h"


static uint16_t adc1Dma[4];

unsigned int Init_Analog(ADC_HandleTypeDef *hadc1) {

    if (HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return 1;
    }
    if (HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *) adc1Dma, 4) != HAL_OK) {
        return 1;
    }

    return 0;
}

unsigned int Get_Analog(VcuInput *input, VcuParameters *params) {

  input->apps1 = (float) (adc1Dma[3]) * 3.3f / 65536.0f;
  input->apps2 = (float) (adc1Dma[0]) * 3.3f / 65536.0f;
  input->bse1 = (float) (adc1Dma[2]) * 3.3f / 65536.0f;
  input->bse2 = (float) (adc1Dma[1]) * 3.3f / 65536.0f;
//  input->steeringWheelPotVoltage = (float) (adcData[STEER_CHANNEL] * 3.3 / 65536.0);

    return 0;
}
