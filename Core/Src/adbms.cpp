/**
 * @file adbms.cpp
 * @author Pranit Arya
 * @brief Communication functions for ADBMS IC (LTC6813)
 * @version 0.1
 * @date 2023-10-29
 *
 * Created by Pranit Arya 10/29/23
 */

#include <cstring>
#include "adbms.h"
#include "spi.h"
#include "ltc6813.h"

static uint16_t pec15 (char *data , int len);

// Interrupt callbacks
static LTC6813_Error_t bms_error = LTC6813_ERROR;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    bms_error = LTC6813_OK;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    bms_error = LTC6813_SPI_ERROR;
}

extern SPI_HandleTypeDef hspi1; // BMS SPI

// voltages_buf must be >= 18*NUM_BMS_ICS = 180
LTC6813_Error_t ltc6813_voltage_read(uint16_t *voltages_buf) {
//    uint16_t data_buf[18*NUM_BMS_ICS];
    LTC6813_Error_t status = LTC6813_OK;
    LTC6813_Error_t err = LTC6813_OK;

    for(int i = 0; i < NUM_BMS_ICS; i++) {
        status = ltc6813_cmd_read(CMD_RDCVA, ((uint8_t *)voltages_buf)+i);
        if(status != LTC6813_OK)
            err = status;
        status = ltc6813_cmd_read(CMD_RDCVB, ((uint8_t *)voltages_buf)+i+6);
        if(status != LTC6813_OK)
            err = status;
        status = ltc6813_cmd_read(CMD_RDCVC, ((uint8_t *)voltages_buf)+i+12);
        if(status != LTC6813_OK)
            err = status;
        status = ltc6813_cmd_read(CMD_RDCVD, ((uint8_t *)voltages_buf)+i+18);
        if(status != LTC6813_OK)
            err = status;
        status = ltc6813_cmd_read(CMD_RDCVE, ((uint8_t *)voltages_buf)+i+24);
        if(status != LTC6813_OK)
            err = status;
        status = ltc6813_cmd_read(CMD_RDCVF, ((uint8_t *)voltages_buf)+i+30);
        if(status != LTC6813_OK)
            err = status;
    }

    return err;
}

LTC6813_Error_t ltc6813_cmd_write(LTC6813_Command_t command, uint8_t *data, uint8_t len) {
    LTC6813_Command_t cmd_buf = command; // PEC15 requires pointer to command
    uint16_t cmd_crc = pec15((char *) cmd_buf, 2);
    uint16_t data_crc = pec15((char *) data, len);

    // Send command
    if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_buf, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }
    if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_crc, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }

    // TODO: Handle shifting (comm with multiple BMS ICs)
    if(HAL_SPI_Transmit(&hspi1, data, len, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }
    if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &data_crc, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }

    return LTC6813_OK;
}

LTC6813_Error_t ltc6813_start_adc_conv(uint8_t cells) {
    ltc6813_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, cells);

    return LTC6813_OK;
}

cell_asic ltcChip[NUM_BMS_ICS];
LTC6813_Error_t ltc6813_cmd_voltage_read(uint8_t reg_num, uint16_t *data_buf) {
    ltc6813_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);
    uint8_t error = ltc6813_rdcv(reg_num + 1, NUM_BMS_ICS, ltcChip);

    // TODO: Check implementation
    // TODO: Figure out how to only edit voltages for conversions that were done this cycle (or if that's even necessary)
    for(int i = 0; i < NUM_BMS_ICS; i++) {
        for(int o = 0; o < 18; o++) {
            data_buf[i * o] = ltcChip[i].cells.c_codes[o];
        }
    }

    return ((error) ? LTC6813_ERROR : LTC6813_OK);
}

// ltc6813_cmd_read(CMD_RDCs[cmd_ID], rawData);
// In: Command_id, * to data array
// Out: Dump raw data into array

// data_buf must be >= data size (6 bytes per ADC IC?)
LTC6813_Error_t ltc6813_cmd_read(LTC6813_Command_t command, uint8_t *data_buf) {
    LTC6813_Command_t cmd_buf = command; // PEC15 requires pointer to command
    LTC6813_Error_t output_status = LTC6813_OK;
    uint16_t received_data_crc;
    uint16_t cmd_crc = pec15((char *) cmd_buf, 2);

    // Send command
    if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_buf, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }
    if(HAL_SPI_Transmit(&hspi1, (uint8_t *) &cmd_crc, 2, LTC_SPI_TIMEOUT) != HAL_OK) {
        return LTC6813_SPI_ERROR;
    }

    // Get result from daisy chain, check CRC
    for(int i = 0; i < NUM_BMS_ICS; i++) {
        if(HAL_SPI_Receive(&hspi1, data_buf+(i*6), 6, LTC_SPI_TIMEOUT) != HAL_OK) {
            return LTC6813_SPI_ERROR;
        }
        if(HAL_SPI_Receive(&hspi1, (uint8_t *)(&received_data_crc), 2, LTC_SPI_TIMEOUT) != HAL_OK) { // PEC15
            return LTC6813_SPI_ERROR;
        }
        uint16_t data_crc = pec15((char *)data_buf, 5);
        if(data_crc != received_data_crc) {
            output_status = LTC6813_INVALID_DATA;
        }
    }

    return output_status;

}

/************************************************
 * ADI-given PEC15 Code
*************************************************/

/************************************
Copyright 2012 Analog Devices, Inc. (ADI)
Permission to freely use, copy, modify, and distribute this software for any purpose with or
without fee is hereby granted, provided that the above copyright notice and this permission
notice appear in all copies: THIS SOFTWARE IS PROVIDED “AS IS” AND ADI DISCLAIMS ALL WARRANTIES
INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ADI BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY
USE OF SAME, INCLUDING ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
***************************************/

/**
 * @brief Generate pec15 CRC for LTC6813
 *
 * @param data
 * @param len
 * @return uint16_t
 */
static uint16_t pec15 (char *data , int len) {
    uint16_t remainder, address;
    remainder = 16; // PEC seed
    for (int i = 0; i < len; i++) {
        address = ((remainder >> 7) ^ data[i]) & 0xff; //calculate PEC table address
        remainder = (remainder << 8 ) ^ pec15Table[address];
    }
    return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

// *******************************************************************************************************************