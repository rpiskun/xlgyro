#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief I2C slave address of LSM9DS1 accelerometer and gyroscope
 * @def LSM9DS1_XLGYRO_ADDRESS
 * @note SlaveAddress[6:1] = 0b110101
 *       SlaveAddress[0] = SDO_A/G pin = 1
 */
#define LSM9DS1_XLGYRO_ADDRESS          (0x6B)

/**
 * @brief I2C slave address of LSM9DS1 magnetometer
 * @def LSM9DS1_MAGNETOMETR_ADDRESS
 * @note SlaveAddress[6:2] = 0b00111
 *       SlaveAddress[1] = SDO_M pin = 1
 *       SlaveAddress[0] = 0
 */
#define LSM9DS1_MAGNETOMETR_ADDRESS     (0x1E)

/**
 * @brief Defines read I2C operation
 * @def I2C_OPERATION_READ
 */
#define I2C_OPERATION_READ              (0x01)

/**
 * @brief Defines write I2C operation
 * @def I2C_OPERATION_WRITE
 */
#define I2C_OPERATION_WRITE             (0x00)

/**
 * @brief
 * @def LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE
 */
#define LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE    (1)

/**
 * @brief
 * @def LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE
 */
#define LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE   (0)

#define ACT_THS                         (0x04)
#define ACT_DUR                         (0x05)
#define INT_GEN_CFG_XL                  (0x06)
#define INT_GEN_THS_X_XL                (0x07)
#define INT_GEN_THS_Y_XL                (0x08)
#define INT_GEN_THS_Z_XL                (0x09)
#define INT_GEN_DUR_XL                  (0x0A)
#define REFERENCE_G                     (0x0B)
#define INT1_CTRL                       (0x0C)
#define INT2_CTRL                       (0x0D)
#define WHO_AM_I                        (0x0F)
#define CTRL_REG1_G                     (0x10)
#define CTRL_REG2_G                     (0x11)
#define CTRL_REG3_G                     (0x12)
#define ORIENT_CFG_G                    (0x13)
#define INT_GEN_SRC_G                   (0x14)
#define INT_GEN_SRC_G                   (0x14)
#define OUT_TEMP_L                      (0x15)
#define OUT_TEMP_H                      (0x16)
#define STATUS_REG_G                    (0x17)
#define OUT_X_G                         (0x18)
#define OUT_Y_G                         (0x1A)
#define OUT_Z_G                         (0x1C)
#define CTRL_REG4                       (0x1E)
#define CTRL_REG5_XL                    (0x1F)
#define CTRL_REG6_XL                    (0x20)
#define CTRL_REG7_XL                    (0x21)
#define CTRL_REG8                       (0x22)
#define CTRL_REG9                       (0x23)
#define CTRL_REG10                      (0x24)
#define INT_GEN_SRC_XL                  (0x26)
#define STATUS_REG_XL                   (0x27)
#define OUT_X_XL                        (0x28)
#define OUT_Y_XL                        (0x2A)
#define OUT_Z_XL                        (0x2C)
#define FIFO_CTRL                       (0x2E)
#define FIFO_SRC                        (0x2F)
#define INT_GEN_CFG_G                   (0x30)
#define INT_GEN_THS_X_G                 (0x31)
#define INT_GEN_THS_Y_G                 (0x33)
#define INT_GEN_THS_Z_G                 (0x35)
#define INT_GEN_DUR_G                   (0x37)

#define XL_G_WHO_AM_I_RESPONSE          (0x68)

/**
 * @brief
 *
 */
typedef enum LSM9DS1_OPERATION_STATUS_ENUM
{
    E_LSM9DS1_SUCCESS = 0,
    E_LSM9DS1_FAIL
} LSM9DS1_OPERATION_STATUS_E;

typedef enum
{
    E_X_AXIS = 0,
    E_Y_AXIS,
    E_Z_AXIS,
    E_AXIS_COUNT
} AXISES;

/**
 * @brief
 *
 */
typedef enum LINEAR_ACCELERATION_RANGE_ENUM
{
    E_LINEAR_ACCELERATION_RANGE_MIN = 0,
    E_LINEAR_ACCELERATION_RANGE_2 = E_LINEAR_ACCELERATION_RANGE_MIN,
    E_LINEAR_ACCELERATION_RANGE_4,
    E_LINEAR_ACCELERATION_RANGE_8,
    E_LINEAR_ACCELERATION_RANGE_16,
    E_LINEAR_ACCELERATION_RANGE_MAX = E_LINEAR_ACCELERATION_RANGE_16
} LINEAR_ACCELERATION_RANGE_E;

/**
 * @brief
 *
 */
typedef enum ANGULAR_RATE_RANGE_ENUM
{
    E_ANGULAR_RATE_RANGE_MIN = 0,
    E_ANGULAR_RATE_RANGE_245 = E_ANGULAR_RATE_RANGE_MIN,
    E_ANGULAR_RATE_RANGE_500,
    E_ANGULAR_RATE_RANGE_2000,
    E_ANGULAR_RATE_RANGE_MAX = E_ANGULAR_RATE_RANGE_2000
} ANGULAR_RATE_RANGE_E;

typedef enum MAGNETIC_RANGE_ENUM
{
    E_MAGNETIC_RANGE_4 = 0,
    E_MAGNETIC_RANGE_8,
    E_MAGNETIC_RANGE_12,
    E_MAGNETIC_RANGE_16,
} MAGNETIC_RANGE_E;

/**
 * @brief
 *
 */
typedef enum OPERATING_MODE_ENUM
{
    E_OPERATING_MODE_POWERDOWN = 0,
    E_OPERATING_MODE_XL_ONLY,
    E_OPERATING_MODE_XL_GYRO
} OPERATING_MODE_E;

/**
 * @brief
 *
 */
typedef enum XL_POWER_MODE_ENUM
{
    E_XL_POWER_MODE_POWERDOWN   = 0x00,
    E_XL_POWER_MODE_10HZ        = 0x01,
    E_XL_POWER_MODE_50HZ        = 0x02,
    E_XL_POWER_MODE_119HZ       = 0x03,
    E_XL_POWER_MODE_238HZ       = 0x04,
    E_XL_POWER_MODE_476HZ       = 0x05,
    E_XL_POWER_MODE_952HZ       = 0x06,
    E_XL_POWER_MODE_NA          = 0x07
} XL_POWER_MODE_E;

/**
 * @brief
 *
 */
typedef enum GYRO_POWER_MODE_ENUM
{
    E_GYRO_POWER_MODE_POWERDOWN = 0x00,
    E_GYRO_POWER_MODE_14_9HZ    = 0x01,
    E_GYRO_POWER_MODE_59_5HZ    = 0x02,
    E_GYRO_POWER_MODE_119HZ     = 0x03,
    E_GYRO_POWER_MODE_238HZ     = 0x04,
    E_GYRO_POWER_MODE_476HZ     = 0x05,
    E_GYRO_POWER_MODE_952HZ     = 0x06,
    E_GYRO_POWER_MODE_NA        = 0x07
} GYRO_POWER_MODE_E;

/**
 * @brief
 *
 */
typedef enum FIFO_MODE_ENUM
{
    E_FIFO_MODE_BYPASS                  = 0x00,
    E_FIFO_MODE_FIFO                    = 0x01,
    E_FIFO_MODE_CONTINUOUS_TO_FIFO      = 0x03,
    E_FIFO_MODE_BYPASS_TO_CONTINUOUS    = 0x04,
    E_FIFO_MODE_CONTINUOUS              = 0x06
} FIFO_MODE_E;

/**
 * @brief
 *
 */
typedef union LSM9DS1_ADDRESS_STRUCT
{
    struct __attribute__((packed, aligned(1)))
    {
        /** R/W operation */
        uint8_t operationRW     : 1;
        /** Slave device address. Not shifted */
        uint8_t deviceAddr      : 7;
    } fields;
    uint16_t value;
} LSM9DS1_ADDRESS_S;

/**
 * @brief Describes I2C request
 * @struct LSM9DS1_REQUEST_S
 */
typedef struct __attribute__((packed, aligned(1))) LSM9DS1_REQUEST_STRUCT
{
    union
    {
        struct
        {
            /** Register address - 7 LSB bits */
            uint8_t registerAddr    : 7;
            /** Address auto increment flag - MSB bit */
            uint8_t autoIncrement   : 1;
        } fields;
        uint8_t value;
    } subAddress;
    /** Pointer to payload */
    uint8_t payload[0];
} LSM9DS1_REQUEST_S;

/**
 * @brief
 *
 */
typedef struct LSM9DS1_CONFIG_STRUCT
{
    LINEAR_ACCELERATION_RANGE_E linearAccelerationRate;
    ANGULAR_RATE_RANGE_E angularRate;
    MAGNETIC_RANGE_E magneticRange;
    OPERATING_MODE_E operatingMode;
    XL_POWER_MODE_E xlPowerMode;
    GYRO_POWER_MODE_E gyroPowerMode;
    FIFO_MODE_E fifoMode;
} LSM9DS1_CONFIG_S;

typedef union CTRL_REG6_XL_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t bw_xl       : 2;
        uint8_t bw_scal_odr : 1;
        uint8_t fs_xl       : 2;
        uint8_t odr_xl      : 3;
    } bitmap;
    uint8_t value;
} CTRL_REG6_XL_U;

typedef union CTRL_REG1_G_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t bw_g    : 2;
        uint8_t reserver: 1;
        uint8_t fs_g    : 2;
        uint8_t odr_g   : 3;
    } bitmap;
    uint8_t value;
} CTRL_REG1_G_U;

typedef union FIFO_CTRL_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t fth   : 5;
        uint8_t fmode : 3;
    } bitmap;
    uint8_t value;
} FIFO_CTRL_U;

typedef union FIFO_SRC_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t fss   : 6;
        uint8_t ovrn  : 1;
        uint8_t fth   : 1;
    } bitmap;
    uint8_t value;
} FIFO_SRC_U;

typedef union CTRL_REG9_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t stop_on_fth     : 1;
        uint8_t fifo_en         : 1;
        uint8_t i2c_disable     : 1;
        uint8_t drdy_mask       : 1;
        uint8_t fifo_temp_en    : 1;
        uint8_t reserved1       : 1;
        uint8_t sleep_g         : 1;
        uint8_t reserved2       : 1;
    } bitmap;
    uint8_t value;
} CTRL_REG9_U;

typedef union STATUS_REG_UNION
{
    struct __attribute__((packed, aligned(1)))
    {
        uint8_t xl_da       : 1;
        uint8_t g_da        : 1;
        uint8_t tda         : 1;
        uint8_t boot_status : 1;
        uint8_t inact       : 1;
        uint8_t ig_g        : 1;
        uint8_t ig_xl       : 1;
    } bitmap;
    uint8_t value;
} STATUS_REG_U;

typedef struct RAW_DATA_STRUCT
{
    int16_t rawData[E_AXIS_COUNT];
} RAW_DATA_S;

/**
 * @brief
 *
 * @param pI2cHandle
 * @param pConfig
 * @return LSM9DS1_OPERATION_STATUS_E
 */
LSM9DS1_OPERATION_STATUS_E LSM9DS1_Init(
    I2C_HandleTypeDef *pI2cHandle,
    const LSM9DS1_CONFIG_S *pConfig);
LSM9DS1_OPERATION_STATUS_E LSM9DS1_AccelReadRawData(RAW_DATA_S *pRawData);
LSM9DS1_OPERATION_STATUS_E LSM9DS1_GyroReadRawData(RAW_DATA_S *pRawData);
LSM9DS1_OPERATION_STATUS_E LSM9DS1_Calibrate();
LSM9DS1_OPERATION_STATUS_E LSM9DS1_IsAccelDataReady(bool *isReady);
LSM9DS1_OPERATION_STATUS_E LSM9DS1_IsGyroDataReady(bool *isReady);
uint8_t LSM9DS1_GetFifoSamples();
uint32_t LSM9DS1_PollDataBlocking();
LSM9DS1_OPERATION_STATUS_E LSM9DS1_AccelRawDataAveraged(RAW_DATA_S *pAccelAveraged);
LSM9DS1_OPERATION_STATUS_E LSM9DS1_GyroRawDataAveraged(RAW_DATA_S *pAccelAveraged);

#endif /* LSM9DS1_H */
