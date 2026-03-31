#include "dvc_bmi088.h"
#include "alg_quaternion.h"
#include "task.h"

#define BMI088_OK_OR_RETURN(x) do { if ((x) != HAL_OK) return HAL_ERROR; } while (0)

/**
 * @brief 初始化BMI088传感器
 * @param hspi SPI句柄
 * @retval HAL_StatusTypeDef 初始化状态
 */
HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t dummy = 0;

    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    SPI_DMA_Init();

    //先对两个传感器都进行软重置。
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_SOFTRESET, BMI088_SOFT_RESET_CMD));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD));
    vTaskDelay(pdMS_TO_TICKS(50));
    /*
     * 上电/复位后，ACC 以 I2C 模式启动。
     * 第一次 SPI 读取用于切换接口，可能会返回无效数据。
     */
    BMI088_OK_OR_RETURN(SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &dummy, 1));
    vTaskDelay(pdMS_TO_TICKS(1));

    //验证重置后的芯片ID是否正确。
    BMI088_OK_OR_RETURN(SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &acc_id, 1));
    BMI088_OK_OR_RETURN(SPI_ReadReg(hspi, GYRO, BMI088_REG_GYRO_CHIP_ID, &gyro_id, 1));
    if ((acc_id != BMI088_ACCEL_CHIP_ID) || (gyro_id != BMI088_GYRO_CHIP_ID))
    {
        return HAL_ERROR;
    }

    //加速度计电源和配置。
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CONF, 0x00));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CTRL, BMI088_ACCEL_PWR_ENABLE));
    vTaskDelay(pdMS_TO_TICKS(10));
    //加速度计配置。
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_CONF, BMI088_ACCEL_CONF_1600HZ_NORMAL));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_RANGE, BMI088_ACCEL_RANGE_24G));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT1_IO_CONF, BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP));

    //陀螺仪配置。
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_LPM1, BMI088_GYRO_MODE_NORMAL));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_532_HZ));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_RANGE, BMI088_GYRO_RANGE_2000_DPS));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT_CTRL, BMI088_GYRO_INT_DRDY_ENABLE));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DRDY_MAP));

    return HAL_OK;
}
/**
 * @brief 读取BMI088加速度计数据
 * @param hspi SPI句柄
 * @param data 存储加速度计数据的结构体指针
 * @retval 无
 */
void BMI088_ReadAccel(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t accel_data_raw[6];
    int16_t accel_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_X_LSB, accel_data_raw, 6) != HAL_OK)
    {
        return;
    }

    accel_data[0] = (int16_t)((uint16_t)accel_data_raw[1] << 8 | accel_data_raw[0]);
    accel_data[1] = (int16_t)((uint16_t)accel_data_raw[3] << 8 | accel_data_raw[2]);
    accel_data[2] = (int16_t)((uint16_t)accel_data_raw[5] << 8 | accel_data_raw[4]);

    data->acc.x = accel_data[0] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.y = accel_data[1] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.z = accel_data[2] / BMI088_ACCEL_SENSITIVITY_24G;
}
/**
 * @brief 读取BMI088陀螺仪数据
 * @param hspi SPI句柄
 * @param data 存储陀螺仪数据的结构体指针
 * @retval 无
 */
void BMI088_ReadGyro(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t gyro_data_raw[6];
    int16_t gyro_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI_ReadReg(hspi, GYRO, BMI088_REG_GYRO_X_LSB, gyro_data_raw, 6) != HAL_OK)
    {
        return;
    }

    gyro_data[0] = (int16_t)((uint16_t)gyro_data_raw[1] << 8 | gyro_data_raw[0]);
    gyro_data[1] = (int16_t)((uint16_t)gyro_data_raw[3] << 8 | gyro_data_raw[2]);
    gyro_data[2] = (int16_t)((uint16_t)gyro_data_raw[5] << 8 | gyro_data_raw[4]);

    // Output stays in deg/s for upper layer compatibility.
    data->gyro.x = gyro_data[0] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.y = gyro_data[1] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.z = gyro_data[2] / BMI088_GYRO_SENSITIVITY_2000_DPS;
}
/**
 * @brief 读取BMI088加速度计温度
 * @param hspi SPI句柄
 * @param data 存储温度数据的结构体指针
 * @retval 无
 */
void BMI088_ReadTemp(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t temp_data_raw[2];
    int16_t temp_data;

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_TEMP_LSB, temp_data_raw, 2) != HAL_OK)
    {
        return;
    }

    temp_data = (int16_t)((uint16_t)temp_data_raw[1] << 8 | temp_data_raw[0]);
    data->temp = (temp_data / 326.8f) + 23.0f;
}
/**
 * @brief 互补滤波计算欧拉角
 * @param data 输入的IMU数据（包含加速度、陀螺仪、温度）
 * @param dt 时间间隔（秒）
 * @param kp 比例增益
 * @param ki 积分增益
 * @return 计算得到的欧拉角（包含roll、pitch、yaw）
 */
euler_t BMI088_Complementary_Filter(imu_data_t *data, float dt, float kp, float ki)
{
    static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    euler_t euler = {0};

    if (data == NULL)
    {
        return euler;
    }

    // Use a local copy to avoid repeatedly scaling raw gyro data.
    imu_data_t imu = *data;
    imu.dt = dt;
    imu.gyro.x = imu.gyro.x * (M_PI / 180.0f);
    imu.gyro.y = imu.gyro.y * (M_PI / 180.0f);
    imu.gyro.z = imu.gyro.z * (M_PI / 180.0f);

    mahony_update(&q, imu, kp, ki);
    euler = quat_to_euler(q);

    return euler;
}

