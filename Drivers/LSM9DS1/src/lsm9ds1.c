#include <stddef.h>
#include <string.h>
#include "lsm9ds1.h"

#define LSM9DS1_DEFAULT_TRANSACTION_TIMEOUT     (20)
#define LSM9DS1_I2C_WRITE_BUFFER_SIZE           (32)
#define LSM9DS1_I2C_READ_BUFFER_SIZE            (32)
#define LSM9DS1_CALIBRATION_FIFO_THD            (0x1F)
#define LSM9DS1_NO_FIFO_THD                     (0)
#define LSM9DS1_SLIDING_WINDOW_SIZE             (64)

typedef struct RAW_SLIDING_WINDOW_STRUCT
{
    RAW_DATA_S window[LSM9DS1_SLIDING_WINDOW_SIZE];
    int32_t windowSum[E_AXIS_COUNT];
    bool inited;
} RAW_SLIDING_WINDOW_S;

static I2C_HandleTypeDef *pLsm9ds1I2cHandle = NULL;
static float accelResolution = 0;
static float gyroResolution = 0;
static bool xlGyroCalibrated = false;

static int16_t xlBiasRaw[E_AXIS_COUNT] = { 0 };
static float xlBias[E_AXIS_COUNT] = { 0 };
static RAW_SLIDING_WINDOW_S xlSlidingWindow = { 0 };

static int16_t gBiasRaw[E_AXIS_COUNT] = { 0 };
static float gBias[E_AXIS_COUNT] = { 0 };
static RAW_SLIDING_WINDOW_S gSlidingWindow = { 0 };

static const float linearAccelerationSensitivities[] =
{
    [E_LINEAR_ACCELERATION_RANGE_2]  = 0.000061,
    [E_LINEAR_ACCELERATION_RANGE_4]  = 0.000122,
    [E_LINEAR_ACCELERATION_RANGE_8]  = 0.000244,
    [E_LINEAR_ACCELERATION_RANGE_16] = 0.000732
};

static const float angularRateSensitivities[] =
{
    [E_ANGULAR_RATE_RANGE_245]  = 0.00875,
    [E_ANGULAR_RATE_RANGE_500]  = 0.0175,
    [E_ANGULAR_RATE_RANGE_2000] = 0.07
};

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_ReadBytes(
    uint8_t deviceAddress,
    LSM9DS1_REQUEST_S *pRequest,
    uint32_t requestLen);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_WriteBytes(
    uint8_t deviceAddress,
    LSM9DS1_REQUEST_S *pRequest,
    uint32_t requestLen);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setOperatingMode(const LSM9DS1_CONFIG_S *pConfig);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoMode(const FIFO_MODE_E fifoMode);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_enableFifo(const bool enable);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_stopOnFifoThd(const bool stop);
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoThd(const uint8_t thd);
static void LSM9DS1_slidingWindowPush(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pValue);

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_enableFifo(const bool enable)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    do
    {
        /* Enable FIFO */
        pRequest->subAddress.fields.registerAddr = CTRL_REG9;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        CTRL_REG9_U *pCtrlReg9 = (CTRL_REG9_U*)&pRequest->payload[0];
        pCtrlReg9->bitmap.fifo_en = (enable == true) ? 1 : 0;
        ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }
    } while (0);

    return ret;
}

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_stopOnFifoThd(const bool stop)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    do
    {
        /* Enable FIFO */
        pRequest->subAddress.fields.registerAddr = CTRL_REG9;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        CTRL_REG9_U *pCtrlReg9 = (CTRL_REG9_U*)&pRequest->payload[0];
        pCtrlReg9->bitmap.stop_on_fth = (stop == true) ? 1 : 0;
        ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }
    } while (0);

    return ret;
}

static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoThd(const uint8_t thd)
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
        pFifoCtrl->bitmap.fth = (thd & 0x1F);
        ret = LSM9DS1_WriteBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }
    } while (0);

    return ret;
}

uint8_t LSM9DS1_GetFifoSamples()
{
    uint8_t ret = 0;
    LSM9DS1_OPERATION_STATUS_E status = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;
    FIFO_SRC_U *pFifoSrc;

    do
    {
        pRequest->subAddress.fields.registerAddr = FIFO_SRC;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        status = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != status)
        {
            break;
        }

        pFifoSrc = (FIFO_SRC_U*)&pRequest->payload[0];
        ret = pFifoSrc->bitmap.fss;
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
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_setFifoMode(const FIFO_MODE_E fifoMode)
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

        switch (fifoMode)
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
                pFifoCtrl->bitmap.fmode = fifoMode;
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

        ret = LSM9DS1_setFifoMode(pConfig->fifoMode);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        ret = LSM9DS1_enableFifo(true);
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

        memset(&xlSlidingWindow, 0, sizeof(RAW_SLIDING_WINDOW_S));
        memset(&gSlidingWindow, 0, sizeof(RAW_SLIDING_WINDOW_S));
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
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_ReadBytes(
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
static LSM9DS1_OPERATION_STATUS_E LSM9DS1_WriteBytes(
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

static void LSM9DS1_slidingWindowPush(RAW_SLIDING_WINDOW_S *pWindow, RAW_DATA_S *pValue)
{
    pWindow->windowSum[E_X_AXIS] = 0;
    pWindow->windowSum[E_Y_AXIS] = 0;
    pWindow->windowSum[E_Z_AXIS] = 0;

    if (pWindow->inited != true)
    {
        /* If sliding window is not inited - fill it with frist given value */
        for (uint32_t i = 0; i < LSM9DS1_SLIDING_WINDOW_SIZE; ++i)
        {
            memcpy(&pWindow->window[i], pValue, sizeof(RAW_DATA_S));

            pWindow->windowSum[E_X_AXIS] += pWindow->window[i].rawData[E_X_AXIS];
            pWindow->windowSum[E_Y_AXIS] += pWindow->window[i].rawData[E_Y_AXIS];
            pWindow->windowSum[E_Z_AXIS] += pWindow->window[i].rawData[E_Z_AXIS];
        }

        pWindow->inited = true;
    }
    else
    {
        for (uint32_t i = (LSM9DS1_SLIDING_WINDOW_SIZE - 1); i > 0; --i)
        {
            pWindow->window[i] = pWindow->window[i - 1];
            pWindow->windowSum[E_X_AXIS] += pWindow->window[i].rawData[E_X_AXIS];
            pWindow->windowSum[E_Y_AXIS] += pWindow->window[i].rawData[E_Y_AXIS];
            pWindow->windowSum[E_Z_AXIS] += pWindow->window[i].rawData[E_Z_AXIS];
        }

        memcpy(&pWindow->window[0], pValue, sizeof(RAW_DATA_S));

        pWindow->windowSum[E_X_AXIS] += pWindow->window[0].rawData[E_X_AXIS];
        pWindow->windowSum[E_Y_AXIS] += pWindow->window[0].rawData[E_Y_AXIS];
        pWindow->windowSum[E_Z_AXIS] += pWindow->window[0].rawData[E_Z_AXIS];
    }
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_AccelReadRawData(RAW_DATA_S *pRawData)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    do
    {
        if (pRawData == NULL)
        {
            break;
        }

        pRequest->subAddress.fields.registerAddr = OUT_X_XL;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 6);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        pRawData->rawData[E_X_AXIS] = (pRequest->payload[1] << 8 | pRequest->payload[0]);
        pRawData->rawData[E_Y_AXIS] = (pRequest->payload[3] << 8 | pRequest->payload[2]);
        pRawData->rawData[E_Z_AXIS] = (pRequest->payload[5] << 8 | pRequest->payload[4]);

        if (xlGyroCalibrated == true)
        {
            pRawData->rawData[E_X_AXIS] -= xlBiasRaw[E_X_AXIS];
            pRawData->rawData[E_Y_AXIS] -= xlBiasRaw[E_Y_AXIS];
            pRawData->rawData[E_Z_AXIS] -= xlBiasRaw[E_Z_AXIS];
        }
    } while (0);

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_GyroReadRawData(RAW_DATA_S *pRawData)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;

    do
    {
        if (pRawData == NULL)
        {
            break;
        }

        pRequest->subAddress.fields.registerAddr = OUT_X_G;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_ENABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 6);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        pRawData->rawData[E_X_AXIS] = (pRequest->payload[1] << 8 | pRequest->payload[0]);
        pRawData->rawData[E_Y_AXIS] = (pRequest->payload[3] << 8 | pRequest->payload[2]);
        pRawData->rawData[E_Z_AXIS] = (pRequest->payload[5] << 8 | pRequest->payload[4]);

        if (xlGyroCalibrated == true)
        {
            pRawData->rawData[E_X_AXIS] -= gBiasRaw[E_X_AXIS];
            pRawData->rawData[E_Y_AXIS] -= gBiasRaw[E_Y_AXIS];
            pRawData->rawData[E_Z_AXIS] -= gBiasRaw[E_Z_AXIS];
        }
    } while (0);

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_Calibrate()
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t samples = 0;
    int32_t xlBiasRawSum[E_AXIS_COUNT] = { 0, 0, 0 };
    int32_t gBiasRawSum[E_AXIS_COUNT] = { 0, 0, 0 };
    RAW_DATA_S xlRawData;
    RAW_DATA_S gRawData;

    memset(&xlRawData, 0, sizeof(RAW_DATA_S));
    memset(&gRawData, 0, sizeof(RAW_DATA_S));

    do
    {
        ret = LSM9DS1_setFifoThd(LSM9DS1_CALIBRATION_FIFO_THD);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        ret = LSM9DS1_stopOnFifoThd(true);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        while (samples < LSM9DS1_CALIBRATION_FIFO_THD)
        {
            samples = LSM9DS1_GetFifoSamples();
        }

        xlGyroCalibrated = false;

        for (uint8_t i = 0; i < samples; ++i)
        {
            ret = LSM9DS1_AccelReadRawData(&xlRawData);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }

            ret = LSM9DS1_GyroReadRawData(&gRawData);
            if (E_LSM9DS1_SUCCESS != ret)
            {
                break;
            }

            xlBiasRawSum[E_X_AXIS] += xlRawData.rawData[E_X_AXIS];
            xlBiasRawSum[E_Y_AXIS] += xlRawData.rawData[E_Y_AXIS];
            xlBiasRawSum[E_Z_AXIS] += xlRawData.rawData[E_Z_AXIS];

            gBiasRawSum[E_X_AXIS] += gRawData.rawData[E_X_AXIS];
            gBiasRawSum[E_Y_AXIS] += gRawData.rawData[E_Y_AXIS];
            // gBiasRawSum[E_Z_AXIS] += gRawData.rawData[E_Z_AXIS] - (int16_t)(1./accelResolution); // Assumes sensor facing up!;
            gBiasRawSum[E_Z_AXIS] += gRawData.rawData[E_Z_AXIS];
        }

        for (uint8_t axis = 0; axis < E_AXIS_COUNT; ++axis)
        {
            xlBiasRaw[axis] = xlBiasRawSum[axis] / samples;
            xlBias[axis] = xlBiasRaw[axis] * accelResolution;

            gBiasRaw[axis] = gBiasRawSum[axis] / samples;
            gBias[axis] = gBiasRaw[axis] * gyroResolution;
        }

        xlGyroCalibrated = true;

        ret = LSM9DS1_stopOnFifoThd(false);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        ret = LSM9DS1_setFifoThd(LSM9DS1_NO_FIFO_THD);
    } while(0);

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_IsAccelDataReady(bool *isReady)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;
    STATUS_REG_U *pStatusReg;

    do
    {
        if (isReady == NULL)
        {
            break;
        }

        pRequest->subAddress.fields.registerAddr = STATUS_REG_XL;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        pStatusReg = (STATUS_REG_U*)&pRequest->payload[0];
        if (pStatusReg->bitmap.xl_da == 1)
        {
            *isReady = true;
        }
        else
        {
            *isReady = false;
        }

    } while (0);

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_IsGyroDataReady(bool *isReady)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;
    uint8_t i2cBuf[LSM9DS1_I2C_WRITE_BUFFER_SIZE];
    memset(i2cBuf, 0, LSM9DS1_I2C_WRITE_BUFFER_SIZE);
    LSM9DS1_REQUEST_S *pRequest = (LSM9DS1_REQUEST_S*)i2cBuf;
    STATUS_REG_U *pStatusReg;

    do
    {
        if (isReady == NULL)
        {
            break;
        }

        pRequest->subAddress.fields.registerAddr = STATUS_REG_G;
        pRequest->subAddress.fields.autoIncrement = LSM9DS1_ADDRESS_AUTOINCREMENT_DISABLE;
        ret = LSM9DS1_ReadBytes(LSM9DS1_XLGYRO_ADDRESS, pRequest, 1);
        if (E_LSM9DS1_SUCCESS != ret)
        {
            break;
        }

        pStatusReg = (STATUS_REG_U*)&pRequest->payload[0];
        if (pStatusReg->bitmap.g_da == 1)
        {
            *isReady = true;
        }
        else
        {
            *isReady = false;
        }

    } while (0);

    return ret;
}

uint32_t LSM9DS1_PollDataBlocking()
{
    uint32_t ret = 0;
    LSM9DS1_OPERATION_STATUS_E status = E_LSM9DS1_FAIL;
    uint8_t samples = 0;
    RAW_DATA_S accelData = { 0 };
    RAW_DATA_S gyroData = { 0 };

    do
    {
        samples = LSM9DS1_GetFifoSamples();

        if (samples > 0)
        {
            ret = samples;
        	do
        	{
                status = LSM9DS1_AccelReadRawData(&accelData);
                if (E_LSM9DS1_SUCCESS != status)
                {
                    ret = 0;
                    break;
                }

                status = LSM9DS1_GyroReadRawData(&gyroData);
                if (E_LSM9DS1_SUCCESS != status)
                {
                    ret = 0;
                    break;
                }

                LSM9DS1_slidingWindowPush(&xlSlidingWindow, &accelData);
                LSM9DS1_slidingWindowPush(&gSlidingWindow, &gyroData);

                samples--;
        	} while (samples > 0);
        }
    } while (0);

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_AccelRawDataAveraged(RAW_DATA_S *pAccelAveraged)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;

    if (pAccelAveraged != NULL)
    {
        pAccelAveraged->rawData[E_X_AXIS] = xlSlidingWindow.windowSum[E_X_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Y_AXIS] = xlSlidingWindow.windowSum[E_Y_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Z_AXIS] = xlSlidingWindow.windowSum[E_Z_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;

        ret = E_LSM9DS1_SUCCESS;
    }

    return ret;
}

LSM9DS1_OPERATION_STATUS_E LSM9DS1_GyroRawDataAveraged(RAW_DATA_S *pAccelAveraged)
{
    LSM9DS1_OPERATION_STATUS_E ret = E_LSM9DS1_FAIL;

    if (pAccelAveraged != NULL)
    {
        pAccelAveraged->rawData[E_X_AXIS] = gSlidingWindow.windowSum[E_X_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Y_AXIS] = gSlidingWindow.windowSum[E_Y_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;
        pAccelAveraged->rawData[E_Z_AXIS] = gSlidingWindow.windowSum[E_Z_AXIS] / LSM9DS1_SLIDING_WINDOW_SIZE;

        ret = E_LSM9DS1_SUCCESS;
    }

    return ret;
}
