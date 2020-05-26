/**
 * \file ads111x.h
 * \brief ADS111x 16bit Analog to digital converter (2 or 4 channels)
 *  I2C default address: 0x48 when ADR connected to GND
 *                       0x49 when ADR connected to VDD
 *                       0x4A when ADR connected to SDA
 *                       0x4B when ADR connected to SCL
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 30.04.2019
 *
 * Library to setup and drive the 16 bit ADC converter ADS111x 
 * 
 */

#ifndef ADS111X_H
#define ADS111X_H

/**
 * \struct ads111x [ads111x.h] Configuration structure definition
 */

typedef struct ads111x{
    char deviceName[25];                        // Device Name of IC
    unsigned char deviceAddress;                // Bus device address
} device_ads111x;

/**
 * \brief ADS111x driver initialization
 * \param pointer on the configuration structure
 * \return code error
 */
extern int ads111x_init(device_ads111x *ads111xconfig);        // ADS1215 driver initialization

/**
 * \brief ADS111x read convertion data register
 * \param pointer on the configuration structure
 * \return code error
 */
extern int ads111x_getVoltage_mv(device_ads111x *ads111xconfig, unsigned char channel);

extern int actuator_getVoltage(device_ads111x *ads111xconfig, unsigned char channel);

#endif /* ADS111X_H */