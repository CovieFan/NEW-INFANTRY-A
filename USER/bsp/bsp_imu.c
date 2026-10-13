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

volatile float    g_imu_gyro_z     = 0.0f;
volatile int16_t  g_imu_gyro_z_raw = 0;
volatile int16_t  g_imu_gyro_x_raw = 0;
volatile int16_t  g_imu_gyro_y_raw = 0;

volatile int16_t  g_imu_acc_x_raw = 0;
volatile int16_t  g_imu_acc_y_raw = 0;
volatile int16_t  g_imu_acc_z_raw = 0;
volatile float    g_imu_gyro_yaw  = 0.0f;
volatile float    g_imu_acc_norm   = 0.0f;   /* ★加计三轴模长 |a| (LSB)：静止 ≈ 4096 */
volatile float    g_imu_grav_ux  = 0.0f;   /* ★重力方向单位向量（机体系），平方和 = 1 */
volatile float    g_imu_grav_uy  = 0.0f;
volatile float    g_imu_grav_uz  = 0.0f;



volatile uint8_t  g_imu_whoami     = 0U;
volatile uint8_t  g_imu_online     = 0U;
volatile uint32_t g_imu_read_ok    = 0U;
volatile uint32_t g_imu_read_fail  = 0U;

static uint32_t s_imu_last_ok_ms = 0U;

volatile float    g_imu_gyro_z_bias = 0.0f;
volatile uint8_t  g_imu_cal_ok      = 0U;
volatile uint32_t g_imu_cal_cnt     = 0U;

static float    s_cal_sum      = 0.0f;
static uint32_t s_cal_start_ms = 0U;

/* ★2026-10-13 静止检测状态：记录标定窗口首帧的三轴原始值与最大偏差 */
static int16_t  s_cal_x0  = 0;
static int16_t  s_cal_y0  = 0;
static int16_t  s_cal_z0  = 0;
static uint8_t  s_cal_ref = 0U;
static uint16_t s_cal_dev = 0U;

/* ★重力方向（机体系）低通状态 */
static float   s_grav_x    = 0.0f;
static float   s_grav_y    = 0.0f;
static float   s_grav_z    = 0.0f;
static uint8_t s_grav_init = 0U;   /* 首帧直接赋值，避免上电爬升污染零偏标定 */


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

/* ---------------- 上电初始化 ---------------- */
void bsp_imu_init(void)
{
    static const uint8_t seq[][2] =
    {
        { MPU6500_PWR_MGMT_1,   0x03U },  /* 时钟源 = 陀螺 Z */
        { MPU6500_PWR_MGMT_2,   0x00U },  /* 使能 accel + gyro */
        { MPU6500_CONFIG,       0x02U },  /* 陀螺 DLPF ≈92Hz */
        { MPU6500_GYRO_CONFIG,  0x10U },  /* ±1000 dps */
        { MPU6500_ACCEL_CONFIG, 0x10U },  /* ±8 g */
        { MPU6500_ACCEL_CFG_2,  0x02U },  /* accel DLPF ≈92Hz */
    };
    uint8_t i;
    uint8_t who = 0U;

    imu_cs(0U);                                   /* 先释放片选 */

    (void)imu_write_reg(MPU6500_PWR_MGMT_1, 0x80U);  /* 软复位 */
    HAL_Delay(100U);                                 /* 手册要求 ≥100ms */

    (void)imu_read_regs(MPU6500_WHO_AM_I, &who, 1U);
    g_imu_whoami = who;                              /* 期望 0x70 */

    for (i = 0U; i < (uint8_t)(sizeof(seq) / sizeof(seq[0])); i++)
    {
        (void)imu_write_reg(seq[i][0], seq[i][1]);
        HAL_Delay(1U);
    }

    s_imu_last_ok_ms = HAL_GetTick();
    g_imu_online     = 0U;

        /* 零偏标定重新开始（本函数前面已经 Delay 过 ≥106ms，云台应该静止）*/
    g_imu_cal_ok      = 0U;
    g_imu_cal_cnt     = 0U;
    g_imu_gyro_z_bias = BSP_IMU_YAW_BIAS_DEFAULT;   /* ★预置实测兜底值：标定不合格时就用它 */
    s_cal_ref         = 0U;
    s_cal_dev         = 0U;
    s_cal_sum         = 0.0f;
    s_cal_start_ms    = HAL_GetTick();


}

/* ---------------- 1kHz 取数 ---------------- */
void bsp_imu_update(void)
{
    uint8_t buf[14];
    float   yaw_now = 0.0f;   /* ★投影后的真 yaw 角速度 (rad/s) */
    float   n2;
    float   inv;
    uint16_t dx, dy, dz;
    if (imu_read_regs(MPU6500_ACCEL_XOUT_H, buf, 14U) == 0U)
    {
        g_imu_read_fail++;
        if ((HAL_GetTick() - s_imu_last_ok_ms) > BSP_IMU_TIMEOUT_MS)
        {
            g_imu_online = 0U;
            g_imu_gyro_z = 0.0f;   /* ★掉线 → 输出归零，防"旧值当恒定偏置" */
            g_imu_gyro_yaw = 0.0f;   /* ★掉线 → 真 yaw 也归零 */
        }
        return;
    }

        /* ★加计：原来读了就丢，现在用来定"竖直方向" */
    g_imu_acc_x_raw = imu_be16(&buf[0]);
    g_imu_acc_y_raw = imu_be16(&buf[2]);
    g_imu_acc_z_raw = imu_be16(&buf[4]);
    /* ★|a| = sqrt(ax^2+ay^2+az^2)：静止时 ≈ 4096（±8g 量程下 1g = 4096LSB）
       与云台朝向无关（重力大小不变）；乱跳或明显偏小 ⇒ 加计有问题 */
    g_imu_acc_norm = sqrtf((float)g_imu_acc_x_raw * (float)g_imu_acc_x_raw
                         + (float)g_imu_acc_y_raw * (float)g_imu_acc_y_raw
                         + (float)g_imu_acc_z_raw * (float)g_imu_acc_z_raw);
    
       /* buf[0..5]=accel  buf[6..7]=temp  buf[8..13]=gyro(大端) */
    g_imu_gyro_x_raw = imu_be16(&buf[8]);
    g_imu_gyro_y_raw = imu_be16(&buf[10]);
    g_imu_gyro_z_raw = imu_be16(&buf[12]);

    g_imu_gyro_z = BSP_IMU_GYRO_Z_SIGN * (float)g_imu_gyro_z_raw * MPU6500_GYRO_LSB2RADPS;   /* 仅诊断：单轴 Z，不再参与控制 */
    /* ===== ★投影：真 yaw 角速度 = ω · ĝ  （ĝ = 重力方向单位向量）=====
       为什么这么干：板子装歪了，竖直方向不在任何一根陀螺轴上，
                     单看某一轴会漏掉大部分真 yaw 角速度。
       ① 加计慢速低通，抗底盘加减速/振动带来的假重力
       ② 归一化成单位向量，4400 与 4096 的幅值差被消掉，只用方向
       好处：不需要知道板子装成什么角度，也不需要额外标定 */
    if (s_grav_init == 0U)
    {
        s_grav_x    = (float)g_imu_acc_x_raw;
        s_grav_y    = (float)g_imu_acc_y_raw;
        s_grav_z    = (float)g_imu_acc_z_raw;
        s_grav_init = 1U;
    }
    else
    {
        s_grav_x += BSP_IMU_GRAV_ALPHA * ((float)g_imu_acc_x_raw - s_grav_x);
        s_grav_y += BSP_IMU_GRAV_ALPHA * ((float)g_imu_acc_y_raw - s_grav_y);
        s_grav_z += BSP_IMU_GRAV_ALPHA * ((float)g_imu_acc_z_raw - s_grav_z);
    }

    n2 = s_grav_x * s_grav_x + s_grav_y * s_grav_y + s_grav_z * s_grav_z;

    if (n2 > BSP_IMU_GRAV_MIN_N2)
    {
        inv = 1.0f / sqrtf(n2);

        g_imu_grav_ux = s_grav_x * inv;
        g_imu_grav_uy = s_grav_y * inv;
        g_imu_grav_uz = s_grav_z * inv;

        yaw_now = BSP_IMU_YAW_SIGN *
                  ( (float)g_imu_gyro_x_raw * MPU6500_GYRO_LSB2RADPS * g_imu_grav_ux
                  + (float)g_imu_gyro_y_raw * MPU6500_GYRO_LSB2RADPS * g_imu_grav_uy
                  + (float)g_imu_gyro_z_raw * MPU6500_GYRO_LSB2RADPS * g_imu_grav_uz );
    }
    else
    {
        g_imu_grav_ux = 0.0f;
        g_imu_grav_uy = 0.0f;
        g_imu_grav_uz = 0.0f;
        yaw_now       = 0.0f;   /* 加计不可信，不给假数据 */
    }

    if (g_imu_cal_ok == 0U)
    {
        /* ① 标定窗口：只累计零偏；对外输出 0 且不算"在线"
              （上层会把它当无效 -> 阻尼/速度环归零，安全）*/
        s_cal_sum += yaw_now;
        g_imu_cal_cnt++;

        /* ★静止检测：与窗口首帧比，任三轴偏移超阈值就判「动过」，此时不采用这个均值 */
        if (s_cal_ref == 0U)
        {
            s_cal_x0  = g_imu_gyro_x_raw;
            s_cal_y0  = g_imu_gyro_y_raw;
            s_cal_z0  = g_imu_gyro_z_raw;
            s_cal_ref = 1U;
            s_cal_dev = 0U;
        }
        else
        {
            dx = (uint16_t)((g_imu_gyro_x_raw > s_cal_x0) ? (g_imu_gyro_x_raw - s_cal_x0) : (s_cal_x0 - g_imu_gyro_x_raw));
            dy = (uint16_t)((g_imu_gyro_y_raw > s_cal_y0) ? (g_imu_gyro_y_raw - s_cal_y0) : (s_cal_y0 - g_imu_gyro_y_raw));
            dz = (uint16_t)((g_imu_gyro_z_raw > s_cal_z0) ? (g_imu_gyro_z_raw - s_cal_z0) : (s_cal_z0 - g_imu_gyro_z_raw));
            if (dx > s_cal_dev) { s_cal_dev = dx; }
            if (dy > s_cal_dev) { s_cal_dev = dy; }
            if (dz > s_cal_dev) { s_cal_dev = dz; }
        }

        if ((HAL_GetTick() - s_cal_start_ms) >= BSP_IMU_CAL_MS)
        {
            if (g_imu_cal_cnt >= BSP_IMU_CAL_MIN_CNT)
            {
                if (s_cal_dev <= BSP_IMU_CAL_MAX_DEV_LSB)
                {
                    g_imu_gyro_z_bias = s_cal_sum / (float)g_imu_cal_cnt;   /* ★确实静止 -> 用实测标定值 */
                }
                /* 否则（动过）-> 保留预置的 BSP_IMU_YAW_BIAS_DEFAULT */
                g_imu_cal_ok      = 1U;
            }
            else
            {
                /* 帧太少（主循环被卡过？）-> 重来一轮 */
                s_cal_sum      = 0.0f;
                g_imu_cal_cnt  = 0U;
                s_cal_start_ms = HAL_GetTick();
                s_cal_ref         = 0U;
                s_cal_dev         = 0U;
            }
        }

        g_imu_gyro_yaw = 0.0f;
        g_imu_online = 0U;
    }
    else
    {
        /* ② 正常输出：先减零偏 */
        g_imu_gyro_yaw = yaw_now - g_imu_gyro_z_bias;
        g_imu_online = 1U;
    }

    g_imu_read_ok++;
    s_imu_last_ok_ms = HAL_GetTick();
}
