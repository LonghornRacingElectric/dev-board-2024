/*!
  ltc681x hardware library
@verbatim
  This library contains all of the hardware dependant functions used by the bms 
  code
@endverbatim
REVISION HISTORY
$Revision: 2000 $
$Date: 2017-4

Copyright (c) 2017, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2017 Linear Technology Corp. (LTC)
*/
#include <stdint.h>
#include "bms_hardware.h"
#include "main.h"
#include "spi.h"

void cs_low() {
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

void cs_high() {
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void delay_u(uint16_t micro) {
  int len = micro * 10;
  for(volatile int i = 0; i < len; i++);
}

void delay_m(uint16_t milli) {
  HAL_Delay(milli);
}

/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
) {
  uint32_t error = HAL_SPI_Transmit(&hspi1, data, len, SPI_TIMEOUT);
  if(error) {
    Error_Handler();
  }
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
) {
  uint32_t error = HAL_SPI_Transmit(&hspi1, tx_Data, tx_len, SPI_TIMEOUT);
  if(error) {
    Error_Handler();
  }

  error = HAL_SPI_Receive(&hspi1, rx_data, rx_len, SPI_TIMEOUT);
  if(error) {
    Error_Handler();
  }
}


uint8_t spi_read_byte() {
  uint8_t data = 0xFF;

  uint32_t error = HAL_SPI_Receive(&hspi1, &data, 1, SPI_TIMEOUT);
  if(error) {
    Error_Handler();
  }

  return (data);
}

void spi_write_byte(uint8_t data) {
  uint32_t error = HAL_SPI_Transmit(&hspi1, &data, 1, SPI_TIMEOUT);
  if(error) {
    Error_Handler();
  }
}
