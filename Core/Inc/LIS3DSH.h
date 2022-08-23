/*
 * LIS3DSH.h
 *
 * Copyright (c) 2022 8E1XX.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/
 *
 */

#ifndef INC_LIS3DSH_H_
#define INC_LIS3DSH_H_

#include "stm32f4xx_hal.h"

#define SPI_BUFFER_SIZE 	7
#define OUT_T	 			0x0C
#define INFO1	 			0x0D
#define INFO2	 			0x0E
#define WHO_AM_I 			0x0F
#define CTRL_REG4			0x20
#define CTRL_REG1			0x21
#define CTRL_REG2			0x22
#define CTRL_REG3			0x23
#define CTRL_REG5			0x24
#define CTRL_REG6			0x25
#define OUT_X_L  			0x28
#define OUT_X_H  			0x29
#define OUT_Y_L  			0x2A
#define OUT_Y_H  			0x2B
#define OUT_Z_L  			0x2C
#define OUT_Z_H  			0x2D
#define FIFO_CTRL 			0x2E
#define FIFO_SRC			0x2F




class LIS3DSH {

 /* Attributes */
 private:
	uint8_t accDataTx [SPI_BUFFER_SIZE] = {OUT_X_L | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	volatile uint8_t accDataRx [SPI_BUFFER_SIZE];
	int16_t xRaw;
	int16_t yRaw;
	int16_t zRaw;
	float offsetX;
	float offsetY;
	float offsetZ;

 public:
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csAccPinBank;
	uint16_t 		   csAccPin;

 /* Method */
 private:


 public:
	/* Initialization */
	void init(SPI_HandleTypeDef *spiHandler, GPIO_TypeDef *csPinBank, uint16_t csPin);

	/* Read/Write data - polling method */
	void writeToAddressPol(uint8_t address, uint8_t data);
	uint8_t readFromAddressPol(uint8_t address);
	void readDataRawPol();

	/* Read/Write data - DMA Method */
	uint8_t readDataDMA();
	void readDataDMAComplete();

	/* Getter and setter */
	int16_t getXRaw();
	int16_t getYRaw();
	int16_t getZRaw();
};



#endif /* INC_LIS3DSH_H_ */
