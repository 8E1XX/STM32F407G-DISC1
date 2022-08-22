/*
 * LIS3DSH.h
 *
 *  Created on: Aug 19, 2022
 *      Author: 8E1XX
 */

#ifndef INC_LIS3DSH_H_
#define INC_LIS3DSH_H_

#include "stm32f4xx_hal.h"

#define OUT_T	 	0x0C
#define INFO1	 	0x0D
#define INFO2	 	0x0E
#define WHO_AM_I 	0x0F
#define CTRL_REG4	0x20
#define CTRL_REG1	0x21
#define CTRL_REG2	0x22
#define CTRL_REG3	0x23
#define CTRL_REG5	0x24
#define CTRL_REG6	0x25
#define OUT_X_L  	0x28
#define OUT_X_H  	0x29
#define OUT_Y_L  	0x2A
#define OUT_Y_H  	0x2B
#define OUT_Z_L  	0x2C
#define OUT_Z_H  	0x2D
#define FIFO_CTRL 	0x2E
#define FIFO_SRC	0x2F

class LIS3DSH {

 /* Attributes */
 private:
	int16_t xRaw;
	int16_t yRaw;
	int16_t zRaw;
	float x;
	float y;
	float z;

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

	/* Read/Write data - interrupt and DMA Method */
	void readDataRawDMA();

	/* Getter and setter */
	int16_t getXRaw();
	int16_t getYRaw();
	int16_t getZRaw();

	void setXValue (float x);
	void setYValue (float y);
	void setZValue (float z);
};



#endif /* INC_LIS3DSH_H_ */
