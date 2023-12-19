#include "analog.h"

int adc_calibrate(ADC_HandleTypeDef* hadc){
    return HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

int adc_start(ADC_HandleTypeDef* hadc){
    int error = adc_calibrate(hadc);
    if(error != 0) return error;
    return HAL_ADCEx_MultiModeStart_DMA(hadc, reinterpret_cast<uint32_t *>(adcValues), NUM_SENSORS);
}

int adc_stop(ADC_HandleTypeDef* hadc){
    return HAL_ADCEx_MultiModeStop_DMA(hadc);
}

static float analog_convert(uint16_t adc_value){
    return ((float) adc_value) * 3.3f / 65536.0f;
}

void adc_get(AnalogVoltages* analogVoltages) {
    analogVoltages->apps1 = analog_convert(adcValues[APPS1]);
    analogVoltages->apps2 = analog_convert(adcValues[APPS2]);
    analogVoltages->bse1 = analog_convert(adcValues[BSE1]);
    analogVoltages->bse2 = analog_convert(adcValues[BSE2]);
    analogVoltages->steer = analog_convert(adcValues[STEER]);
//    analogVoltages->sus1 = analog_convert(adcValues[SUS1]);
//    analogVoltages->sus2 = analog_convert(adcValues[SUS2]);
}

void adc_get_bspd(BSPD* bspd_state){
    bspd_state->brake_pressed = HAL_GPIO_ReadPin(BSPD_BrakePressed_GPIO_Port, BSPD_BrakePressed_Pin) == GPIO_PIN_SET;
    bspd_state->motor_on = HAL_GPIO_ReadPin(BSPD_MotorOn_GPIO_Port, BSPD_MotorOn_Pin) == GPIO_PIN_SET;
    bspd_state->shutdown = HAL_GPIO_ReadPin(BSPD_Shutdown_GPIO_Port, BSPD_Shutdown_Pin) == GPIO_PIN_SET;
    bspd_state->brake_failure = HAL_GPIO_ReadPin(BSPD_BrakeFailure_GPIO_Port, BSPD_BrakeFailure_Pin) == GPIO_PIN_SET;
    bspd_state->motor_failure = HAL_GPIO_ReadPin(BSPD_MotorFailure_GPIO_Port, BSPD_MotorFailure_Pin) == GPIO_PIN_SET;
}
