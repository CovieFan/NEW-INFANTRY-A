#include "bsp_imu.h"
#include "main.h"     /* MPU6500_CS_Pin / MPU6500_CS_GPIO_Port / HAL */
#include "spi.h"      /* hspi5 */
#include <math.h>     /* sqrtf */

/* ---------------- MPU6500 寄存器 ---------------- */
#define MPU6500_WHO_AM_I      0x75U
#define MPU6500_PWR_MGMT_1    0x6BU
#define MPU6500_PWR_MGMT_2    0x6CU
#define MPU6500_CONFIG        0x1AU
#define MPU6500_GYRO_CONFIG   0x1BU
#define MPU6500_ACCEL_CONFIG  0x1CU
#define MPU6500_ACCEL_CFG_2   0x1DU
#define MPU6500_ACCEL_XOUT_H  0x3BU   /* burst 起点：accel(6)+temp(2)+gyro(6)=14字节 */

/* 换算：±1000dps ⇒ 1LSB = 1000/32768 dps = 0.0305176 dps
                        ×π/180 = 5.3261e-4 rad/s */
#define MPU6500_GYRO_LSB2RADPS  (5.3261e-4f)
/* int16原始值均值不应超出传感器单轴表示范围；用于拒绝损坏的偏置状态。 */
#define MPU6500_GYRO_MAX_RADPS  (32768.0f * MPU6500_GYRO_LSB2RADPS)

volatile float    imu_gyro_z     = 0.0f;
volatile int16_t  imu_gyro_z_raw = 0;
volatile int16_t  imu_gyro_x_raw = 0;
volatile int16_t  imu_gyro_y_raw = 0;

volatile int16_t  imu_acc_x_raw = 0;
volatile int16_t  imu_acc_y_raw = 0;
volatile int16_t  imu_acc_z_raw = 0;
volatile float    imu_gyro_yaw  = 0.0f;
volatile float    imu_acc_norm   = 0.0f;   /* ★加计三轴模长 |a| (LSB)：静止 ≈ 4096 */
volatile float    imu_grav_ux  = 0.0f;   /* ★重力方向单位向量（机体系），平方和 = 1 */
volatile float    imu_grav_uy  = 0.0f;
volatile float    imu_grav_uz  = 0.0f;



volatile uint8_t  imu_whoami     = 0U;
volatile uint8_t  imu_online     = 0U;
volatile uint8_t  imu_init_ok    = 0U;
volatile uint8_t  imu_startup_done = 0U;
volatile uint8_t  imu_sample_valid = 0U;
volatile bsp_imu_fault_e imu_fault = BSP_IMU_FAULT_NONE;
volatile uint32_t imu_invalid_count = 0U;
volatile uint32_t imu_read_ok    = 0U;
volatile uint32_t imu_read_fail  = 0U;

static uint32_t imu_last_ok_ms = 0U;

volatile float    imu_gyro_z_bias = 0.0f;
volatile float    imu_gyro_bias_x = 0.0f;
volatile float    imu_gyro_bias_y = 0.0f;
volatile float    imu_gyro_bias_z = 0.0f;
volatile bsp_imu_cal_source_e imu_cal_source = BSP_IMU_CAL_PENDING;
volatile uint8_t  imu_cal_ok      = 0U;
volatile uint32_t imu_cal_cnt     = 0U;

static float    cal_sum_x    = 0.0f;
static float    cal_sum_y    = 0.0f;
static float    cal_sum_z    = 0.0f;
static uint32_t cal_start_ms = 0U;

/* ★2026-10-13 静止检测状态：记录标定窗口首帧的三轴原始值与最大偏差 */
static int16_t  cal_x0  = 0;
static int16_t  cal_y0  = 0;
static int16_t  cal_z0  = 0;
static uint8_t  cal_ref = 0U;
static uint16_t cal_dev = 0U;

/* ★重力方向（机体系）低通状态 */
static float   grav_x    = 0.0f;
static float   grav_y    = 0.0f;
static float   grav_z    = 0.0f;
static uint8_t grav_init = 0U;   /* 首帧直接赋值，避免上电爬升污染零偏标定 */

static uint8_t imu_bias_valid(float bias)
{
    return (uint8_t)(isfinite(bias) && (fabsf(bias) <= MPU6500_GYRO_MAX_RADPS));
}

/* 初始化和少样本重试共用，避免某根轴残留上一窗口的数据。 */
static void imu_calibration_reset(uint32_t now)
{
    cal_sum_x = 0.0f;
    cal_sum_y = 0.0f;
    cal_sum_z = 0.0f;
    imu_cal_cnt = 0U;
    cal_ref = 0U;
    cal_dev = 0U;
    cal_start_ms = now;
}


/* ---------------- 片选 ---------------- */
static void imu_cs(uint8_t low)
{
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin,
                      (low != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ---------------- 写一个寄存器（最高位=0） ---------------- */
static uint8_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    HAL_StatusTypeDef r;

    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = val;

    imu_cs(1U);
    r = HAL_SPI_Transmit(&hspi5, tx, 2U, 10U);
    imu_cs(0U);

    return (r == HAL_OK) ? 1U : 0U;
}

/* ---------------- 连续读（最高位=1） ---------------- */
static uint8_t imu_read_regs(uint8_t reg, uint8_t *out, uint16_t len)
{
    uint8_t cmd = (uint8_t)(reg | 0x80U);
    HAL_StatusTypeDef r;

    imu_cs(1U);
    r = HAL_SPI_Transmit(&hspi5, &cmd, 1U, 10U);
    if (r == HAL_OK)
    {
        r = HAL_SPI_Receive(&hspi5, out, len, 10U);
    }
    imu_cs(0U);

    return (r == HAL_OK) ? 1U : 0U;
}

static int16_t imu_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* 无效测量立即撤销YAW许可；不能先置0再减偏置，伪造出非零速度。 */
static void imu_reject_sample(bsp_imu_fault_e fault)
{
    imu_online = 0U;
    imu_sample_valid = 0U;
    imu_gyro_yaw = 0.0f;
    imu_grav_ux = 0.0f;
    imu_grav_uy = 0.0f;
    imu_grav_uz = 0.0f;
    grav_init = 0U;
    imu_fault = fault;
}

/* ---------------- 上电初始化 ---------------- */
void bsp_imu_init(void)
{
    /* 第3列只读回已定义的配置位，不要求保留位具有特定值。 */
    static const uint8_t seq[][3] =
    {
        { MPU6500_PWR_MGMT_1,   0x03U, 0x7FU },  /* 时钟源 = 陀螺 Z；复位位自清除 */
        { MPU6500_PWR_MGMT_2,   0x00U, 0x3FU },  /* 使能 accel + gyro */
        { MPU6500_CONFIG,       0x02U, 0x7FU },  /* 陀螺 DLPF */
        { MPU6500_GYRO_CONFIG,  0x10U, 0xFBU },  /* ±1000 dps */
        { MPU6500_ACCEL_CONFIG, 0x10U, 0xF8U },  /* ±8 g */
        { MPU6500_ACCEL_CFG_2,  0x02U, 0x0FU },  /* accel DLPF */
    };
    uint8_t i;
    uint8_t who = 0U;
    uint8_t readback;

    imu_init_ok = 0U;
    imu_startup_done = 0U;
    imu_reject_sample(BSP_IMU_FAULT_NONE);
    imu_whoami = 0U;
    imu_gyro_z = 0.0f;
    grav_x = 0.0f;
    grav_y = 0.0f;
    grav_z = 0.0f;
    imu_cal_ok = 0U;
    imu_cal_source = BSP_IMU_CAL_PENDING;
    imu_gyro_bias_x = 0.0f;
    imu_gyro_bias_y = 0.0f;
    imu_gyro_bias_z = 0.0f;
    imu_gyro_z_bias = BSP_IMU_YAW_BIAS_DEFAULT;
    imu_calibration_reset(HAL_GetTick());

    imu_cs(0U);                                   /* 先释放片选 */

    if (imu_write_reg(MPU6500_PWR_MGMT_1, 0x80U) == 0U)
    {
        imu_fault = BSP_IMU_FAULT_INIT_SPI;
        imu_startup_done = 1U;
        return;
    }
    HAL_Delay(100U);                                 /* 手册要求 ≥100ms */

    if (imu_read_regs(MPU6500_WHO_AM_I, &who, 1U) == 0U)
    {
        imu_fault = BSP_IMU_FAULT_INIT_SPI;
        imu_startup_done = 1U;
        return;
    }
    imu_whoami = who;                              /* 期望 0x70 */
    if (who != 0x70U)
    {
        imu_fault = BSP_IMU_FAULT_INIT_ID;
        imu_startup_done = 1U;
        return;
    }

    for (i = 0U; i < (uint8_t)(sizeof(seq) / sizeof(seq[0])); i++)
    {
        if (imu_write_reg(seq[i][0], seq[i][1]) == 0U)
        {
            imu_fault = BSP_IMU_FAULT_INIT_SPI;
            imu_startup_done = 1U;
            return;
        }
        HAL_Delay(1U);
        if (imu_read_regs(seq[i][0], &readback, 1U) == 0U)
        {
            imu_fault = BSP_IMU_FAULT_INIT_SPI;
            imu_startup_done = 1U;
            return;
        }
        if ((readback & seq[i][2]) != (seq[i][1] & seq[i][2]))
        {
            imu_fault = BSP_IMU_FAULT_INIT_CONFIG;
            imu_startup_done = 1U;
            return;
        }
    }

    imu_last_ok_ms = HAL_GetTick();
    imu_init_ok    = 1U;
    cal_start_ms    = HAL_GetTick();


}

/* ---------------- 主循环取数 ---------------- */
/* SPI读取失败保留原在线超时策略。 */
static uint8_t imu_read_sample(uint8_t buf[14])
{
    if (imu_init_ok == 0U) { return 0U; }
    if (imu_read_regs(MPU6500_ACCEL_XOUT_H, buf, 14U) == 0U)
    {
        imu_read_fail++;
        imu_sample_valid = 0U;
        imu_fault = BSP_IMU_FAULT_SAMPLE_SPI;
        if ((HAL_GetTick() - imu_last_ok_ms) > BSP_IMU_TIMEOUT_MS)
        {
            imu_online = 0U;
            imu_gyro_z = 0.0f;   /* ★掉线 → 输出归零，防"旧值当恒定偏置" */
            imu_gyro_yaw = 0.0f;   /* ★掉线 → 真 yaw 也归零 */
        }
        return 0U;
    }
    imu_read_ok++;
    return 1U;
}

/* 解包、更新重力方向并检查当前测量，成功后才进入标定/发布。 */
static uint8_t imu_decode_sample(const uint8_t buf[14], float *gyro_x,
                                 float *gyro_y, float *gyro_z, float *yaw_now)
{
    float n2;
    float inv;
        /* ★加计：原来读了就丢，现在用来定"竖直方向" */
    imu_acc_x_raw = imu_be16(&buf[0]);
    imu_acc_y_raw = imu_be16(&buf[2]);
    imu_acc_z_raw = imu_be16(&buf[4]);
    /* ★|a| = sqrt(ax^2+ay^2+az^2)：静止时 ≈ 4096（±8g 量程下 1g = 4096LSB）
       与云台朝向无关（重力大小不变）；乱跳或明显偏小 ⇒ 加计有问题 */
    imu_acc_norm = sqrtf((float)imu_acc_x_raw * (float)imu_acc_x_raw
                         + (float)imu_acc_y_raw * (float)imu_acc_y_raw
                         + (float)imu_acc_z_raw * (float)imu_acc_z_raw);
    
       /* buf[0..5]=accel  buf[6..7]=temp  buf[8..13]=gyro(大端) */
    imu_gyro_x_raw = imu_be16(&buf[8]);
    imu_gyro_y_raw = imu_be16(&buf[10]);
    imu_gyro_z_raw = imu_be16(&buf[12]);

    *gyro_x = (float)imu_gyro_x_raw * MPU6500_GYRO_LSB2RADPS;
    *gyro_y = (float)imu_gyro_y_raw * MPU6500_GYRO_LSB2RADPS;
    *gyro_z = (float)imu_gyro_z_raw * MPU6500_GYRO_LSB2RADPS;
    imu_gyro_z = BSP_IMU_GYRO_Z_SIGN * *gyro_z; /* 单轴Z仅诊断，其SIGN不参与三轴校零。 */
    /* 先检查原始帧：旧低通状态不能把新收到的全零加计伪装为有效重力。 */
    if (!isfinite(imu_acc_norm) || ((imu_acc_norm * imu_acc_norm) <= BSP_IMU_GRAV_MIN_N2))
    {
        imu_invalid_count++;
        imu_reject_sample(BSP_IMU_FAULT_SAMPLE_GRAVITY);
        return 0U;
    }
    /* 加计低通后归一化，提取竖直方向。安装姿态无需对准Z轴；
       动态加速度仍会干扰这个方向，低通不能消除全部影响。 */
    if (grav_init == 0U)
    {
        grav_x    = (float)imu_acc_x_raw;
        grav_y    = (float)imu_acc_y_raw;
        grav_z    = (float)imu_acc_z_raw;
        grav_init = 1U;
    }
    else
    {
        grav_x += BSP_IMU_GRAV_ALPHA * ((float)imu_acc_x_raw - grav_x);
        grav_y += BSP_IMU_GRAV_ALPHA * ((float)imu_acc_y_raw - grav_y);
        grav_z += BSP_IMU_GRAV_ALPHA * ((float)imu_acc_z_raw - grav_z);
    }

    n2 = grav_x * grav_x + grav_y * grav_y + grav_z * grav_z;

    if (isfinite(n2) && (n2 > BSP_IMU_GRAV_MIN_N2))
    {
        inv = 1.0f / sqrtf(n2);

        imu_grav_ux = grav_x * inv;
        imu_grav_uy = grav_y * inv;
        imu_grav_uz = grav_z * inv;

        *yaw_now = BSP_IMU_YAW_SIGN *
                  (*gyro_x * imu_grav_ux + *gyro_y * imu_grav_uy + *gyro_z * imu_grav_uz);
    }
    else
    {
        imu_invalid_count++;
        imu_reject_sample(BSP_IMU_FAULT_SAMPLE_GRAVITY);
        return 0U;
    }
    if (!isfinite(*yaw_now))
    {
        imu_invalid_count++;
        imu_reject_sample(BSP_IMU_FAULT_SAMPLE_NONFINITE);
        return 0U;
    }
    imu_sample_valid = 1U;
    imu_fault = BSP_IMU_FAULT_NONE;

    return 1U;
}

static void imu_calibrate_sample(float gyro_x, float gyro_y, float gyro_z)
{
    uint16_t dx, dy, dz;
    /* ① 标定窗口：只累计零偏；对外输出 0 且不算"在线"
          （上层会把它当无效 -> 阻尼/速度环归零，安全）*/
    cal_sum_x += gyro_x;
    cal_sum_y += gyro_y;
    cal_sum_z += gyro_z;
    imu_cal_cnt++;

    /* ★静止检测：与窗口首帧比，任三轴偏移超阈值就判「动过」，此时不采用这个均值 */
    if (cal_ref == 0U)
    {
        cal_x0  = imu_gyro_x_raw;
        cal_y0  = imu_gyro_y_raw;
        cal_z0  = imu_gyro_z_raw;
        cal_ref = 1U;
        cal_dev = 0U;
    }
    else
    {
        dx = (uint16_t)((imu_gyro_x_raw > cal_x0) ? (imu_gyro_x_raw - cal_x0) : (cal_x0 - imu_gyro_x_raw));
        dy = (uint16_t)((imu_gyro_y_raw > cal_y0) ? (imu_gyro_y_raw - cal_y0) : (cal_y0 - imu_gyro_y_raw));
        dz = (uint16_t)((imu_gyro_z_raw > cal_z0) ? (imu_gyro_z_raw - cal_z0) : (cal_z0 - imu_gyro_z_raw));
        if (dx > cal_dev) { cal_dev = dx; }
        if (dy > cal_dev) { cal_dev = dy; }
        if (dz > cal_dev) { cal_dev = dz; }
    }

    if ((HAL_GetTick() - cal_start_ms) >= BSP_IMU_CAL_MS)
    {
        if (imu_cal_cnt >= BSP_IMU_CAL_MIN_CNT)
        {
            float bias_x = cal_sum_x / (float)imu_cal_cnt;
            float bias_y = cal_sum_y / (float)imu_cal_cnt;
            float bias_z = cal_sum_z / (float)imu_cal_cnt;

            if ((cal_dev <= BSP_IMU_CAL_MAX_DEV_LSB) &&
                imu_bias_valid(bias_x) && imu_bias_valid(bias_y) && imu_bias_valid(bias_z))
            {
                imu_gyro_bias_x = bias_x;
                imu_gyro_bias_y = bias_y;
                imu_gyro_bias_z = bias_z;
                imu_cal_source = BSP_IMU_CAL_VECTOR;
            }
            else
            {
                /* 未采用本次三轴均值；保留旧标量处理，不伪造三轴默认值。 */
                imu_cal_source = BSP_IMU_CAL_LEGACY_DEFAULT;
            }
            imu_cal_ok      = 1U;
            imu_startup_done = 1U;
        }
        else
        {
            /* 帧太少（主循环被卡过？）-> 重来一轮 */
            imu_calibration_reset(HAL_GetTick());
        }
    }

    imu_gyro_yaw = 0.0f;
    imu_online = 0U;
}

static uint8_t imu_publish_yaw(float gyro_x, float gyro_y, float gyro_z, float yaw_now)
{
    float corrected;
    if (imu_cal_source == BSP_IMU_CAL_VECTOR)
    {
        float bias_x = imu_gyro_bias_x;
        float bias_y = imu_gyro_bias_y;
        float bias_z = imu_gyro_bias_z;
        if (!imu_bias_valid(bias_x) || !imu_bias_valid(bias_y) || !imu_bias_valid(bias_z))
        {
            imu_invalid_count++;
            imu_reject_sample(BSP_IMU_FAULT_SAMPLE_NONFINITE);
            return 0U;
        }
        /* 各轴先校零再投影；姿态变化时不重复扣除旧的投影标量。 */
        corrected = BSP_IMU_YAW_SIGN *
            ((gyro_x - bias_x) * imu_grav_ux + (gyro_y - bias_y) * imu_grav_uy +
             (gyro_z - bias_z) * imu_grav_uz);
    }
    else if (imu_cal_source == BSP_IMU_CAL_LEGACY_DEFAULT)
    {
        corrected = yaw_now - imu_gyro_z_bias;
    }
    else
    {
        imu_invalid_count++;
        imu_reject_sample(BSP_IMU_FAULT_CAL_STATE);
        return 0U;
    }
    if (!isfinite(corrected))
    {
        imu_invalid_count++;
        imu_reject_sample(BSP_IMU_FAULT_SAMPLE_NONFINITE);
        return 0U;
    }
    imu_gyro_yaw = corrected;
    imu_online = 1U;
    return 1U;
}

void bsp_imu_update(void)
{
    uint8_t buf[14];
    float gyro_x, gyro_y, gyro_z;
    float yaw_now = 0.0f;

    if (imu_read_sample(buf) == 0U) { return; }
    if (imu_decode_sample(buf, &gyro_x, &gyro_y, &gyro_z, &yaw_now) == 0U) { return; }
    if (imu_cal_ok == 0U)
    {
        imu_calibrate_sample(gyro_x, gyro_y, gyro_z);
    }
    else if (imu_publish_yaw(gyro_x, gyro_y, gyro_z, yaw_now) == 0U) { return; }
    imu_last_ok_ms = HAL_GetTick();
}
