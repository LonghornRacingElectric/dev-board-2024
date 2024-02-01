///**
// * @file adbms.cpp
// * @author Pranit Arya
// * @brief Communication functions for ADBMS IC (LTC6813)
// * @version 0.1
// * @date 2023-10-29
// *
// * Created by Pranit Arya 10/29/23
// *
// */
//
//#include <cstring>
//#include "adbms.h"
//#include "spi.h"
//
//static void adbms_setCs(bool on) {
//  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, (GPIO_PinState) on);
//}
//
//void adbms_init() {
//  initPec();
//
//  adbms_setCs(false);
//  adbms_setCs(true);
//  HAL_Delay(1);
//  adbms_setCs(false);
//  adbms_setCs(true);
//  HAL_Delay(1);
//  adbms_setCs(false);
//  adbms_setCs(true);
//  HAL_Delay(1);
//}
//
//
//// Interrupt callbacks
////static LTC6813_Error_t bms_error = LTC6813_ERROR;
////void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
////  bms_error = LTC6813_OK;
////}
////
////void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
////  bms_error = LTC6813_SPI_ERROR;
////}
//
//// voltages_buf must be >= 18*NUM_BMS_ICS = 180
//LTC6813_Error_t ltc6813_voltage_read(uint16_t *voltages_buf) {
////    uint16_t data_buf[18*NUM_BMS_ICS];
//  LTC6813_Error_t status = LTC6813_OK;
//  LTC6813_Error_t err = LTC6813_OK;
//
//  for(int i = 0; i < NUM_BMS_ICS; i++) {
//    status = adbms_readCommand(CMD_RDCVA, ((uint8_t *) voltages_buf) + i);
//    if(status != LTC6813_OK)
//      err = status;
//    status = adbms_readCommand(CMD_RDCVB, ((uint8_t *) voltages_buf) + i + 6);
//    if(status != LTC6813_OK)
//      err = status;
//    status = adbms_readCommand(CMD_RDCVC, ((uint8_t *) voltages_buf) + i + 12);
//    if(status != LTC6813_OK)
//      err = status;
//    status = adbms_readCommand(CMD_RDCVD, ((uint8_t *) voltages_buf) + i + 18);
//    if(status != LTC6813_OK)
//      err = status;
//    status = adbms_readCommand(CMD_RDCVE, ((uint8_t *) voltages_buf) + i + 24);
//    if(status != LTC6813_OK)
//      err = status;
//    status = adbms_readCommand(CMD_RDCVF, ((uint8_t *) voltages_buf) + i + 30);
//    if(status != LTC6813_OK)
//      err = status;
//  }
//
//  return err;
//}
//
//LTC6813_Error_t adbms_writeCommand(uint16_t command, uint8_t *data, uint8_t len) {
//  // TODO implement properly
////  uint16_t cmd_buf = command; // PEC15 requires pointer to command
////  uint16_t cmd_crc = pec15(cmd_buf, 2);
////  uint16_t data_crc = pec15( data, len);
////
////  HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_RESET);
////
////  // Send command
////  if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_buf, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
////    return LTC6813_SPI_ERROR;
////  }
////  if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_crc, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
////    return LTC6813_SPI_ERROR;
////  }
////
////  // TODO: Handle shifting (comm with multiple BMS ICs)
////  if(HAL_SPI_Transmit(&hspi1, data, len, LTC_SPI_TIMEOUT) != HAL_OK) {
////    return LTC6813_SPI_ERROR;
////  }
////  if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &data_crc, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
////    return LTC6813_SPI_ERROR;
////  }
////
////  HAL_GPIO_WritePin(SPI_CS_BMS_GPIO_Port, SPI_CS_BMS_Pin, GPIO_PIN_SET);
//
//  return LTC6813_OK;
//}
//
//// data_buf must be >= data size (6 bytes per ADC IC?)
//LTC6813_Error_t adbms_readCommand(uint16_t command, uint8_t *data) {
//  uint8_t buff[8];
//  buff[0] = (command >> 8) & 0xFF; // CMD0
//  buff[1] = command & 0xFF; // CMD1
//  uint16_t command_crc = calculatePec(buff, 2);
//  buff[2] = (command_crc >> 8) & 0xFF; // PEC0
//  buff[3] = command_crc & 0xFF; // PEC1
//
//  adbms_setCs(false);
//  adbms_setCs(true);
//  adbms_setCs(false);
//  adbms_setCs(true);
//  adbms_setCs(false);
//  HAL_SPI_Receive(&hspi1, buff, 1, LTC_SPI_TIMEOUT);
//  adbms_setCs(true);
//  adbms_setCs(false);
//
//  uint32_t err = HAL_SPI_Transmit(&hspi1, buff, 4, LTC_SPI_TIMEOUT);
//
//  for(int i = 0; i < NUM_BMS_ICS; i++) {
//    memset(buff, 0xFF, 8);
//    err |= HAL_SPI_Receive(&hspi1, buff, 8, LTC_SPI_TIMEOUT);
//    volatile uint16_t data_crc = calculatePec(buff, 6);
//    memcpy(data, buff, 6); // TODO temp
//    // TODO verify CRC
//    // TODO put in data_buf
////    if(data_crc != received_data_crc) {
////      output_status = LTC6813_INVALID_DATA;
////    }
//  }
//
//  adbms_setCs(true);
//
//  if(err != HAL_OK) {
//    return LTC6813_SPI_ERROR;
//  }
//
//  return LTC6813_OK;
//}
//
//LTC6813_Error_t adbms_simpleCommand(uint16_t command) {
//  uint8_t buff[8];
//  buff[0] = (command >> 8) & 0xFF;
//  buff[1] = command & 0xFF;
//  uint16_t command_crc = calculatePec(buff, 2);
//  buff[2] = (command_crc >> 8) & 0xFF;
//  buff[3] = command_crc & 0xFF;
//
//  adbms_setCs(false);
//  adbms_setCs(true);
//  adbms_setCs(false);
//  adbms_setCs(true);
//  adbms_setCs(false);
//
//  uint32_t err = HAL_SPI_Transmit(&hspi1, buff, 4, LTC_SPI_TIMEOUT);
//
//  adbms_setCs(true);
//
//  if(err != HAL_OK) {
//    return LTC6813_SPI_ERROR;
//  }
//
//  return LTC6813_OK;
//}
//
///************************************
//Copyright 2012 Analog Devices, Inc. (ADI)
//Permission to freely use, copy, modify, and distribute this software for any purpose with or
//without fee is hereby granted, provided that the above copyright notice and this permission
//notice appear in all copies: THIS SOFTWARE IS PROVIDED “AS IS” AND ADI DISCLAIMS ALL WARRANTIES
//INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ADI BE LIABLE FOR
//ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY
//USE OF SAME, INCLUDING ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
//OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//***************************************/
//
//void initPec()
//{
//  uint16_t remainder;
//  for (int i = 0; i < 256; i++)
//  {
//    remainder = i << 7;
//    for (int bit = 8; bit > 0; --bit)
//    {
//      if (remainder & 0x4000)
//      {
//        remainder = (remainder << 1);
//        remainder = (remainder ^ CRC15_POLY);
//      }
//      else
//      {
//        remainder = (remainder << 1);
//      }
//    }
//    pecTable[i] = remainder & 0xFFFF;
//  }
//}
//
//uint16_t calculatePec(uint8_t *data , int len)
//{
//  uint16_t remainder, address;
//  remainder = 16; //PEC seed
//  for (int i = 0; i < len; i++)
//  {
//    address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
//    remainder = (remainder << 8 ) ^ pecTable[address];
//  }
//  return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
//}
//
/////************************************************
//// * ADI-given PEC15 Code
////*************************************************/
////
/////************************************
////Copyright 2012 Analog Devices, Inc. (ADI)
////Permission to freely use, copy, modify, and distribute this software for any purpose with or
////without fee is hereby granted, provided that the above copyright notice and this permission
////notice appear in all copies: THIS SOFTWARE IS PROVIDED “AS IS” AND ADI DISCLAIMS ALL WARRANTIES
////INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ADI BE LIABLE FOR
////ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY
////USE OF SAME, INCLUDING ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
////OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
////***************************************/
////
/////**
//// * @brief Generate pec15 CRC for LTC6813
//// *
//// * @param data
//// * @param len
//// * @return uint16_t
//// */
////static uint16_t pec15 (uint8_t *data , int len) {
////  uint16_t remainder, address;
////  remainder = 16; // PEC seed
////  for (int i = 0; i < len; i++) {
////    address = ((remainder >> 7) ^ data[i]) & 0xff; //calculate PEC table address
////    remainder = (remainder << 8 ) ^ pec15Table[address];
////  }
////  return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
////}