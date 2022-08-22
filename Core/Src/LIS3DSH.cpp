/*
 * LIS3DSH.cpp
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
 */

#include "LIS3DSH.h"

void LIS3DSH::init(SPI_HandleTypeDef *spiHandler,
				   GPIO_TypeDef *csPinBank, uint16_t csPin) {

	spiHandle = spiHandler;
	csAccPinBank = csPinBank;
	csAccPin = csPin;
	x = 0;
	y = 0;
	z = 0;

	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_SET);
	HAL_Delay(100);

	LIS3DSH::writeToAddressPol(CTRL_REG4, 0b01101111); // ODR3 ODR2 ODR1 ODR0 BDU Zen Yen Xen
	LIS3DSH::writeToAddressPol(CTRL_REG3, 0b11101000); // DR_EN IEA IEL INT2_EN INT1_EN VFILT - STRT
  //accelWriteToAddressPol(CTRL_REG5, 0b00000000); // BW2 BW1 FSCALE2 FSCALE1 FSCALE0 ST2 ST1 SIM
	LIS3DSH::writeToAddressPol(CTRL_REG6, 0b10010000); // BOOT FIFO_EN WTM_EN ADD_INC P1_EMPTY P1_WTM P1_OVERRUN P2_BOOT

}

void LIS3DSH::writeToAddressPol(uint8_t address, uint8_t data) {
  	uint8_t SPIDataTx [2] = {address, data};
  	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_RESET);
  	HAL_SPI_Transmit(spiHandle, SPIDataTx, 2, HAL_MAX_DELAY);
  	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_SET);
}

uint8_t LIS3DSH::readFromAddressPol(uint8_t address) {
	/*
	 * Create buffer with 2 index
	 * index 0 -> address data that need to be OR-ed with R/W control MSB bit
	 * index 1 -> dummy data, because receiving data need to be done simultaneously with transmitting data
	 */
	uint8_t SPIDataTx [2] = {(uint8_t) (address | 0x80), 0x00};
	uint8_t SPIDataRx [2];
	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spiHandle, SPIDataTx, SPIDataRx, 2, 5000);
	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_SET);
	return SPIDataRx [1];
}

void LIS3DSH::readDataRawPol() {
	uint8_t SPIDataTx [7] = {OUT_X_L | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t SPIDataRx [7];
	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spiHandle, SPIDataTx, SPIDataRx, 7, 5000);
	HAL_GPIO_WritePin(csAccPinBank, csAccPin, GPIO_PIN_SET);

	xRaw = SPIDataRx [1] | (SPIDataRx [2] << 8);
	yRaw = SPIDataRx [3] | (SPIDataRx [4] << 8);
	zRaw = SPIDataRx [5] | (SPIDataRx [6] << 8);
}

int16_t LIS3DSH::getXRaw() {
	return xRaw;
}

int16_t LIS3DSH::getYRaw() {
	return yRaw;
}

int16_t LIS3DSH::getZRaw() {
	return zRaw;
}
