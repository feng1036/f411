#ifndef __I2CDEV_H
#define __I2CDEV_H
#include <stdint.h>
#include <stdbool.h>
#include "i2c_drv.h"


#define I2CDEV_NO_MEM_ADDR  0xFF

typedef I2cDrv    I2C_Dev;
#define I2C1_DEV  &sensorsBus


bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);


bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data);

int i2cdevInit(I2C_Dev *dev);


bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data);


bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data);

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data);


bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);


bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data);


bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t data);

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data);


bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

#endif //__I2CDEV_H
