/**
 * @file arduino-i2c.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

extern int i2c_write(unsigned char bus, unsigned char devAddr, unsigned char reg, unsigned char data);
extern int i2c_writeBuffer(unsigned char bus, unsigned char devAddr, unsigned char reg, unsigned char * data, unsigned char count);
extern int i2c_writeRaw(unsigned char bus, unsigned char devAddr, unsigned char * data, unsigned char count);

extern int i2c_readByte(unsigned char bus, unsigned char devAddr, unsigned char reg, unsigned char * ptrDest);
extern int i2c_read(unsigned char bus, unsigned char devAddr, unsigned char reg, unsigned char * data, unsigned char count);
extern int i2c_readRaw(unsigned char bus, unsigned char devAddr, unsigned char * data, unsigned char count);