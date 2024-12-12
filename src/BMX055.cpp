#include "BMX055.h"
#include <cstdint> // Include fixed-width integer types (e.g., uint8_t, uint16_t)

BMX055::BMX055(i2c_inst_t *port, uint8_t accl_addr, uint8_t gyro_addr, uint8_t magn_addr)
{
    // Constructor - Typically you would initialize I2C here if needed
    i2c_port = port;
    accl_addr_ = accl_addr;
    gyro_addr_ = gyro_addr;
    magn_addr_ = magn_addr;
}

uint8_t BMX055::beginAcc(char range)
{
    _range = range; // Save the range internally

    switch (range)
    {
    case 0x03:
        accRange = (2.0 / 2048.0);
        break;
    case 0x05:
        accRange = (4.0 / 2048.0);
        break;
    case 0x08:
        accRange = (8.0 / 2048.0);
        break;
    case 0x0C:
        accRange = (16.0 / 2048.0);
        break;
    }

    // Start I2C communication and set up the accelerometer
    uint8_t data[] = {0x0F, range};
    uint8_t check = 0;

    i2c_write_blocking(i2c_port, accl_addr_, data, 2, false);

    data[0] = ACCL_FILTER_ADDR;
    data[1] = 0x08; // Bandwidth = 7.81 Hz
    i2c_write_blocking(i2c_port, accl_addr_, data, 2, false);

    data[0] = 0x11;
    data[1] = 0x00; // Normal mode, Sleep duration = 0.5ms
    i2c_write_blocking(i2c_port, accl_addr_, data, 2, false);

    // data[0] = 0x36; // slow offset compensation
    // data[1] = 0x00;
    // i2c_write_blocking(i2c_port, accl_addr_, data, 2, false);
    // Reading accelerometer NVM offsets
    data[0] = 0x38; // x-axis
    i2c_write_blocking(i2c_port, accl_addr_, &data[0], 1, true);
    i2c_read_blocking(i2c_port, accl_addr_, &data[1], 1, false);
    printf("Acc offset x: %d ", (int8_t) data[1]);

    data[0] = 0x39; // y-axis
    i2c_write_blocking(i2c_port, accl_addr_, &data[0], 1, true);
    i2c_read_blocking(i2c_port, accl_addr_, &data[1], 1, false);
    printf("y: %d ", (int8_t) data[1]);

    data[0] = 0x3A; // z-axis
    data[1] = 0xED; // z-axis -0.15g
    i2c_write_blocking(i2c_port, accl_addr_, data, 2, false);
    i2c_write_blocking(i2c_port, accl_addr_, &data[0], 1, true);
    i2c_read_blocking(i2c_port, accl_addr_, &data[1], 1, false);
    printf("z: %d \n", (int8_t) data[1]);

    return 0; // Indicate success (or you could return an error code)
}

uint8_t BMX055::beginGyro(char resolution)
{
    _resolution = resolution;

    switch (resolution)
    {
    case 0x00:
        gyroRes = 61.0 / 1000.0;
        break;
    case 0x01:
        gyroRes = 30.5 / 1000.0;
        break;
    case 0x02:
        gyroRes = 15.3 / 1000.0;
        break;
    case 0x03:
        gyroRes = 7.6 / 1000.0;
        break;
    case 0x04:
        gyroRes = 3.8 / 1000.0;
        break;
    }
    uint8_t data[] = {0x0F, resolution}; // Range register, Full scale = +/- 125 degree/s
    uint8_t check = 0x01;
    int loop = 0;
    printf("Gyro set resolution: 0x%02X ", data[1]);

    while ((check != data[1]) && (loop < 500))
    {
        i2c_write_blocking(i2c_port, gyro_addr_, data, 2, false);
        sleep_ms(10);
        i2c_write_blocking(i2c_port, gyro_addr_, &data[0], 1, true);
        i2c_read_blocking(i2c_port, gyro_addr_, &check, 1, false);
        sleep_ms(10);
        loop++;
        // printf("Gyro act resolution: 0x%02X\n", check);
    }

    if (check != data[1])
    {
        printf("Failed, loop %d\n", loop);
    }
    else
    {
        printf("Success, loop %d\n", loop);
    }
    loop = 0;

    data[0] = 0x10;
    data[1] = 0x07; // Bandwidth = 100 Hz
    i2c_write_blocking(i2c_port, gyro_addr_, data, 2, false);

    data[0] = 0x11;
    data[1] = 0x00; // Normal mode, Sleep duration = 2ms
    i2c_write_blocking(i2c_port, gyro_addr_, data, 2, false);

    data[0] = 0x31; // Slow Offset Cancellation
    data[1] = 0xCF; // Enable for all axes, 1.0 deg/s, duration = 80ms
    i2c_write_blocking(i2c_port, gyro_addr_, &data[0], 1, true);
    i2c_read_blocking(i2c_port, gyro_addr_, &check, 1, false);
    printf("Gyro slow cancellation pre : 0x%02X\n", check);

    i2c_write_blocking(i2c_port, gyro_addr_, data, 2, false);

    i2c_write_blocking(i2c_port, gyro_addr_, &data[0], 1, true);
    i2c_read_blocking(i2c_port, gyro_addr_, &check, 1, false);
    printf("Gyro slow cancellation post: 0x%02X\n", check);
    
    uint8_t gyro_offset[6] = {0x36, 0b10000000, 0b00000000, 0b11111111, 0b11111111, 0b00001111};
    i2c_write_blocking(i2c_port, gyro_addr_, gyro_offset, 6, false);
    // gx: 000000001011
    // gy: 111111110001
    // gz: 111111110001

    return 0;
}

uint8_t BMX055::beginMagn()
{
    uint8_t data[] = {0x4B, 0x83}; // Soft reset
    i2c_write_blocking(i2c_port, magn_addr_, data, 2, false);

    data[0] = 0x4C;
    data[1] = 0x00; // Normal Mode, ODR = 10 Hz
    i2c_write_blocking(i2c_port, magn_addr_, data, 2, false);

    data[0] = 0x4E;
    data[1] = 0x84; // X, Y, Z-Axis enabled
    i2c_write_blocking(i2c_port, magn_addr_, data, 2, false);

    data[0] = 0x51;
    data[1] = 0x04; // No. of Repetitions for X-Y Axis = 9
    i2c_write_blocking(i2c_port, magn_addr_, data, 2, false);

    data[0] = 0x52;
    data[1] = 0x0F; // No. of Repetitions for Z-Axis = 15
    i2c_write_blocking(i2c_port, magn_addr_, data, 2, false);

    sleep_ms(300); // Delay 300ms to allow the sensor to stabilize

    return 0;
}

void BMX055::popAccel(float *array)
{
    uint8_t reg_addr = 0x02; // Start address for acceleration data
    uint8_t data[6];         // Buffer to store the acceleration data

    // Read 6 bytes of data from the accelerometer registers
    int result = i2c_write_blocking(i2c_port, accl_addr_, &reg_addr, 1, true);
    if (result < 0)
    {
        // Handle I2C write error
        printf("I2C Accel write failed\n");
        return;
    }

    result = i2c_read_blocking(i2c_port, accl_addr_, data, 6, false);
    if (result < 0)
    {
        // Handle I2C read error
        printf("I2C Accel read failed\n");
        return;
    }

    // Convert the data to 12-bit values for X, Y, Z
    int xAccl = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;
    if (xAccl > 2047)
        xAccl -= 4096;
    array[0] = xAccl * accRange;

    int yAccl = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
    if (yAccl > 2047)
        yAccl -= 4096;
    array[1] = yAccl * accRange;

    int zAccl = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;
    if (zAccl > 2047)
        zAccl -= 4096;
    array[2] = zAccl * accRange;
    // Calculate total acceleration
    array[3] = 9.81 * std::sqrt((array[0]) * (array[0]) + (array[1]) * (array[1]) + (array[2]) * (array[2]));
}

void BMX055::getAcceleration(float *x, float *y, float *z, float *accTotal)
{
    float array_[4];
    popAccel(array_);
    *x = array_[0];
    *y = array_[1];
    *z = array_[2];
    *accTotal = array_[3];
}

void BMX055::getAcceleration(float *array)
{
    float array_[4];
    popAccel(array_);
    array[0] = array_[0];
    array[1] = array_[1];
    array[2] = array_[2];
    array[3] = array_[3];
}

void BMX055::getAcceleration(volatile float *array)
{
    float array_[4];
    popAccel(array_);
    array[0] = array_[0];
    array[1] = array_[1];
    array[2] = array_[2];
    array[3] = array_[3];
}

float BMX055::getAccelerationX()
{
    float x, y, z, total;
    getAcceleration(&x, &y, &z, &total);
    return x;
}

float BMX055::getAccelerationY()
{
    float x, y, z, total;
    getAcceleration(&x, &y, &z, &total);
    return y;
}

float BMX055::getAccelerationZ()
{
    float x, y, z, total;
    getAcceleration(&x, &y, &z, &total);
    return z;
}

float BMX055::getAccelerationTotal()
{
    float x, y, z, total;
    getAcceleration(&x, &y, &z, &total);
    return total;
}

void BMX055::getMagnet(int *x, int *y, int *z)
{
    uint8_t reg = 0x42;
    int result = i2c_write_blocking(i2c_port, magn_addr_, &reg, 1, true);
    if (result < 0)
    {
        printf("I2C Mag write failed\n");
        return;
    }
    result = i2c_read_blocking(i2c_port, magn_addr_, (uint8_t *)_data, 6, false);
    if (result < 0)
    {
        printf("I2C Mag read failed\n");
        return;
    }

    int xMag = ((_data[1] << 8) | (_data[0] & 0xF8)) >> 3;
    if (xMag > 4095)
        xMag -= 8192;
    *x = xMag;

    int yMag = ((_data[3] << 8) | (_data[2] & 0xF8)) >> 3;
    if (yMag > 4095)
        yMag -= 8192;
    *y = yMag;

    int zMag = ((_data[5] << 8) | (_data[4] & 0xFE)) >> 1;
    if (zMag > 16383)
        zMag -= 32768;
    *z = zMag;
}

void BMX055::getMagnet(volatile float *array)
{
    uint8_t reg = 0x42;
    i2c_write_blocking(i2c_port, magn_addr_, &reg, 1, true);
    i2c_read_blocking(i2c_port, magn_addr_, (uint8_t *)_data, 6, false);
    int result = i2c_write_blocking(i2c_port, magn_addr_, &reg, 1, true);
    if (result < 0)
    {
        printf("I2C Mag write failed\n");
        return;
    }
    result = i2c_read_blocking(i2c_port, magn_addr_, (uint8_t *)_data, 6, false);
    if (result < 0)
    {
        printf("I2C Mag read failed\n");
        return;
    }

    int xMag = ((_data[1] << 8) | (_data[0] & 0xF8)) >> 3;
    if (xMag > 4095)
        xMag -= 8192;
    array[0] = xMag;

    int yMag = ((_data[3] << 8) | (_data[2] & 0xF8)) >> 3;
    if (yMag > 4095)
        yMag -= 8192;
    array[1] = yMag;

    int zMag = ((_data[5] << 8) | (_data[4] & 0xFE)) >> 1;
    if (zMag > 16383)
        zMag -= 32768;
    array[2] = zMag;
}

void BMX055::getRotation(int *x, int *y, int *z)
{
    uint8_t reg = 0x02;
    int result = i2c_write_blocking(i2c_port, gyro_addr_, &reg, 1, true);
    if (result < 0)
    {
        printf("I2C Gyro write failed\n");
        return;
    }
    result = i2c_read_blocking(i2c_port, gyro_addr_, (uint8_t *)_data, 6, false);
    if (result < 0)
    {
        printf("I2C Gyro read failed\n");
        return;
    }

    int xGyro = (_data[1] << 8) | _data[0];
    if (xGyro > 32767)
        xGyro -= 65536;
    *x = xGyro;

    int yGyro = (_data[3] << 8) | _data[2];
    if (yGyro > 32767)
        yGyro -= 65536;
    *y = yGyro;

    int zGyro = (_data[5] << 8) | _data[4];
    if (zGyro > 32767)
        zGyro -= 65536;
    *z = zGyro;
}

void BMX055::getRotation(volatile float *array)
{
    uint8_t reg = 0x02;
    int result = i2c_write_blocking(i2c_port, gyro_addr_, &reg, 1, true);
    if (result < 0)
    {
        printf("I2C Gyro write failed\n");
        return;
    }
    result = i2c_read_blocking(i2c_port, gyro_addr_, (uint8_t *)_data, 6, false);
    if (result < 0)
    {
        printf("I2C Gyro read failed\n");
        return;
    }

    int xGyro = (_data[1] << 8) | _data[0];
    if (xGyro > 32767)
        xGyro -= 65536;
    array[0] = xGyro * gyroRes;

    int yGyro = (_data[3] << 8) | _data[2];
    if (yGyro > 32767)
        yGyro -= 65536;
    array[1] = yGyro * gyroRes;

    int zGyro = (_data[5] << 8) | _data[4];
    if (zGyro > 32767)
        zGyro -= 65536;
    array[2] = zGyro * gyroRes;
}
