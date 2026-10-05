#include "bsp_imu.h"
#include "main.h"     /* MPU6500_CS_Pin / MPU6500_CS_GPIO_Port / HAL */
#include "spi.h"      /* hspi5 */

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
volatile uint8_t  g_imu_whoami     = 0U;
volatile uint8_t  g_imu_online     = 0U;
volatile uint32_t g_imu_read_ok    = 0U;
volatile uint32_t g_imu_read_fail  = 0U;

static uint32_t s_imu_last_ok_ms = 0U;

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
}

/* ---------------- 1kHz 取数 ---------------- */
void bsp_imu_update(void)
{
    uint8_t buf[14];

    if (imu_read_regs(MPU6500_ACCEL_XOUT_H, buf, 14U) == 0U)
    {
        g_imu_read_fail++;
        if ((HAL_GetTick() - s_imu_last_ok_ms) > BSP_IMU_TIMEOUT_MS)
        {
            g_imu_online = 0U;
            g_imu_gyro_z = 0.0f;   /* ★掉线 → 输出归零，防"旧值当恒定偏置" */
        }
        return;
    }

    /* buf[0..5]=accel  buf[6..7]=temp  buf[8..13]=gyro(大端) */
    g_imu_gyro_x_raw = imu_be16(&buf[8]);
    g_imu_gyro_y_raw = imu_be16(&buf[10]);
    g_imu_gyro_z_raw = imu_be16(&buf[12]);

    g_imu_gyro_z = BSP_IMU_GYRO_Z_SIGN * (float)g_imu_gyro_z_raw * MPU6500_GYRO_LSB2RADPS;

    g_imu_read_ok++;
    s_imu_last_ok_ms = HAL_GetTick();
    g_imu_online     = 1U;
}