#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "lsm9ds1.h"

#define LSM9DS1_I2C_WRITE_BUFFER_SIZE       (32)
#define LSM9DS1_I2C_READ_BUFFER_SIZE        (32)

static I2C_HandleTypeDef *pLsm9ds1I2cHandle = NULL;
static double accelResolution = 0;
static double gyroResolution = 0;

static const double linearAccelerationSensitivities[] =
{
    [E_LINEAR_ACCELERATION_RANGE_2]  = 0.000061,
    [E_LINEAR_ACCELERATION_RANGE_4]  = 0.000122,
    [E_LINEAR_ACCELERATION_RANGE_8]  = 0.000244,
    [E_LINEAR_ACCELERATION_RANGE_16] = 0.000732
};

static const double angularRateSensitivities[] =
{
    [E_ANGULAR_RATE_RANGE_245]  = 0.00875,
    [E_ANGULAR_RATE_RANGE_500]  = 0.0175,
    [E_ANGULAR_RATE_RANGE_2000] = 0.07
};

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setOperatingMode(const LSM9DS1_CONFIG_S *pConfig);

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoMode(const LSM9DS1_CONFIG_S *pConfig);

/**
 * @brief
 *
 * @param pI2cHandle
 * @param pConfig
 * @return LSM9DS1_OPERATION_STATUS_E
 */
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setOperatingMode(
    const LSM9DS1_CONFIG_S *pConfig)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    do
    {
        if (pConfig->operatingMode == E_OPERATING_MODE_XL_ONLY)
        {
            pRequest->subAddress.fields.registerAddr = CTRL_REG6_XL;
            pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
            ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }

            CTRL_REG6_XL_U *pCtrlReg6xl = (CTRL_REG6_XL_U*)&pRequest->payload[0];
            pCtrlReg6xl->bitmap.odr_xl = pConfig->xlPowerMode;
            ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }
        }
        else if (pConfig->operatingMode == E_OPERATING_MODE_XL_GYRO)
        {
            pRequest->subAddress.fields.registerAddr = CTRL_REG1_G;
            pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
            ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }

            CTRL_REG1_G_U *pCtrlReg1g = (CTRL_REG1_G_U*)&pRequest->payload[0];
            pCtrlReg1g->bitmap.odr_g = pConfig->gyroPowerMode;
            ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }
        }
        else
        {
            break;
        }

    } while (0);

    return ret;
}

/**
 * @brief
 *
 * @param pConfig
 * @return LSM9DS1_OPERATION_STATUS_E
 */
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoMode(
    const LSM9DS1_CONFIG_S *pConfig)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;
    FIFO_CTRL_U *pFifoCtrl;

    do
    {
        pRequest->subAddress.fields.registerAddr = FIFO_CTRL;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        pFifoCtrl = (FIFO_CTRL_U*)&pRequest->payload[0];

        switch (pConfig->fifoMode)
        {
            case E_FIFO_MODE_BYPASS:
                /* no break here */
            case E_FIFO_MODE_FIFO:
                /* no break here */
            case E_FIFO_MODE_CONTINUOUS_TO_FIFO:
                /* no break here */
            case E_FIFO_MODE_BYPASS_TO_CONTINUOUS:
                /* no break here */
            case E_FIFO_MODE_CONTINUOUS:
            {
                pFifoCtrl->bitmap.fmode = pConfig->fifoMode;
                ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
                if (E_LSM9DS1_SUCCESS != ret)
                {
                    break;
                }
            }

            default:
            {
                break;
            }
        }
    } while (0);

    return ret;
}

/**
 * @brief
 *
 * @param pI2cHandle
 * @param pConfig
 * @return LSM9DS1_OPERATION_STATUS_E
 */
LSM9DS1_OPERATION_STATUS_E LSM9DS1_Init(
    I2C_HandleTypeDef *pI2cHandle,
    const LSM9DS1_CONFIG_S *pConfig)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;

    do
    {
        if (pI2cHandle == NULL || pConfig == NULL)
        {
            break;
        }

        pLsm9ds1I2cHandle = pI2cHandle;

        ret = LSM9DS1_setOperatingMode(pConfig);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        ret = LSM9DS1_setFifoMode(pConfig);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        if ( pConfig->linearAccelerationRate >= E_LINEAR_ACCELERATION_RANGE_MIN &&
             pConfig->linearAccelerationRate <= E_LINEAR_ACCELERATION_RANGE_MAX )
        {
            accelResolution = linearAccelerationSensitivities[pConfig->linearAccelerationRate];
        }

        if ( pConfig->angularRate >= E_ANGULAR_RATE_RANGE_MIN &&
             pConfig->angularRate <= E_ANGULAR_RATE_RANGE_MAX )
        {
            gyroResolution = angularRateSensitivities[pConfig->angularRate];
        }
    } while (0);

    return ret;
}

/**
 * @brief
 *
 * @param deviceAddress
 * @param pRequest
 * @param requestLen
 * @return LSM9DS1_OPERATION_STATUS_E
 */
LSM9DS1_OPERATION_STATUS_E LSM9DS1_ReadBytes(
    uint8_t deviceAddress,
    LSM9DS1_REQUEST_S *pRequest,
    uint32_t requestLen)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    HAL_StatusTypeDef halStatus = HAL_ERROR;
    LSM9DS1_ADDRESS_S address = { .fields.deviceAddr = deviceAddress };

    do
    {
        if (pLsm9ds1I2cHandle == NULL)
        {
            break;
        }

        if (NULL == pRequest)
        {
            break;
        }

        address.fields.operationRW = I2C_OPERATION_WRITE;

        halStatus = HAL_I2C_Master_Transmit(
                            pLsm9ds1I2cHandle,
                            address.value,
                            &pRequest->subAddress.value,
                            1,
                            LSM9DS1_DEFAULT_TRANSACTION_TIMEOUT);

        if (HAL_OK != halStatus)
        {
            break;
        }

        address.fields.operationRW = I2C_OPERATION_READ;

        halStatus = HAL_I2C_Master_Receive(
                            pLsm9ds1I2cHandle,
                            address.value,
                            pRequest->payload,
                            requestLen,
                            LSM9DS1_DEFAULT_TRANSACTION_TIMEOUT);

        if (HAL_OK != halStatus)
        {
            break;
        }

        ret = E_LSM9DS1_SUCCESS;

    } while (0);

    return ret;
}

/**
 * @brief
 *
 * @param deviceAddress
 * @param pRequest
 * @param requestLen
 * @return LSM9DS1_OPERATION_STATUS_E
 */
LSM9DS1_OPERATION_STATUS_E LSM9DS1_WriteBytes(
    uint8_t deviceAddress,
    LSM9DS1_REQUEST_S *pRequest,
    uint32_t requestLen)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    HAL_StatusTypeDef halStatus = HAL_ERROR;
    LSM9DS1_ADDRESS_S address = { .fields.deviceAddr = deviceAddress };

    do
    {
        if (pLsm9ds1I2cHandle == NULL)
        {
            break;
        }

        if (NULL == pRequest)
        {
            break;
        }

        address.fields.operationRW = I2C_OPERATION_WRITE;

        halStatus = HAL_I2C_Master_Transmit(
                            pLsm9ds1I2cHandle,
                            address.value,
                            (uint8_t*)pRequest,
                            (requestLen + 1),
                            LSM9DS1_DEFAULT_TRANSACTION_TIMEOUT);

        if (HAL_OK != halStatus)
        {
            break;
        }

        ret = E_LSM9DS1_SUCCESS;

    } while (0);

    return ret;
}

/**
 * @brief
 *
 * @param pI2cHandle
 * @param pValue
 * @return LSM9DS1_OPERATION_STATUS_E
 */
LSM9DS1_OPERATION_STATUS_E LSM9DS1_ReadXlGyroValues(
    XLGYRO_VALUSE_S *pValue)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_READ_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_READ_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    uint16_t rawValue[3] = { 0 };

    do
    {
        // pRequest->subAddress.fields.registerAddr = FIFO_SRC;
        pRequest->subAddress.fields.registerAddr = STATUS_REG_XL;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;

        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        STATUS_REG_U statusReg = {
            .value = pRequest->payload[0]
        };

        // FIFO_SRC_U fifoSrc = {
        //     .value = pRequest->payload[0]
        // };

        if (statusReg.bitmap.g_da || statusReg.bitmap.xl_da)
        // if (fifoSrc.bitmap.fss > 0)
        {
            pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE;

            // for (uint32_t i = 0; i < fifoSrc.bitmap.fss; ++i)
            {
                if (statusReg.bitmap.xl_da)
                {
                    pRequest->subAddress.fields.registerAddr = OUT_X_XL;
                    pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE;
                    ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 6);
                    if (E_LSM9DS1_SUCCESS != ret)
                    {
                        break;
                    }

                    rawValue[E_X_AXIS] = (pRequest->payload[1] << 8 | pRequest->payload[0]);
                    rawValue[E_Y_AXIS] = (pRequest->payload[3] << 8 | pRequest->payload[2]);
                    rawValue[E_Z_AXIS] = (pRequest->payload[5] << 8 | pRequest->payload[4]);

                    pValue->xlX = rawValue[E_X_AXIS] * accelResolution;
                    pValue->xlY = rawValue[E_Y_AXIS] * accelResolution;
                    pValue->xlZ = rawValue[E_Z_AXIS] * accelResolution;
                }

                if (statusReg.bitmap.g_da)
                {
                    pRequest->subAddress.fields.registerAddr = OUT_X_G;
                    pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE;
                    ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 6);
                    if (E_LSM9DS1_SUCCESS != ret)
                    {
                        break;
                    }

                    rawValue[E_X_AXIS] = (pRequest->payload[1] << 8 | pRequest->payload[0]);
                    rawValue[E_Y_AXIS] = (pRequest->payload[3] << 8 | pRequest->payload[2]);
                    rawValue[E_Z_AXIS] = (pRequest->payload[5] << 8 | pRequest->payload[4]);

                    pValue->gX = rawValue[E_X_AXIS] * accelResolution;
                    pValue->gY = rawValue[E_Y_AXIS] * accelResolution;
                    pValue->gZ = rawValue[E_Z_AXIS] * accelResolution;
                }
            }

            ret = E_LSM9DS1_SUCCESS;
        }
    } while (0);

    return ret;
}
