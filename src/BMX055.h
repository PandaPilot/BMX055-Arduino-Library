#pragma once

#include <stdio.h>
#include <cmath>

#include "pico/stdlib.h"  // General Pico SDK functions
#include "hardware/i2c.h" // I2C communication functions

// Define I2C addresses for BMX055 sensor components
// #define BMX055_ACCL_ADDR 0x18 // SDO1-3v3 0x19 // SDO1-GND address: 0x18
// #define BMX055_GYRO_ADDR 0x68 // SDO2-3v3 0x69 // SDO2-GND address: 0x68
// #define BMX055_MAGN_ADDR 0x10 // CSB3-3v3 0x13 // CSB3-GND address: 0x10

#define ACCL_FILTER_ADDR 0x10

class BMX055
{
public:
    BMX055(i2c_inst_t *port, uint8_t accl_addr, uint8_t gyro_addr, uint8_t magn_addr);
    uint8_t beginAcc(char range);
    uint8_t beginGyro(char resolution);
    uint8_t beginMagn(void);
    void getAcceleration(float *x, float *y, float *z, float *accTotal);
    void getAcceleration(float *array);
    void getAcceleration(volatile float *array); // For working with volatile when doing multicore
    void popAccel(float *array);
    float getAccelerationX();
    float getAccelerationY();
    float getAccelerationZ();
    float getAccelerationTotal();
    void getMagnet(int *x, int *y, int *z);
    void getMagnet(volatile float *array);
    void getRotation(int *x, int *y, int *z);
    void getRotation(volatile float *array);

private:
    i2c_inst_t *i2c_port;
    uint8_t _data[6];
    char _range;
    float accRange;
    char _resolution;
    float gyroRes;
    uint8_t accl_addr_; 
    uint8_t gyro_addr_; 
    uint8_t magn_addr_;
};
