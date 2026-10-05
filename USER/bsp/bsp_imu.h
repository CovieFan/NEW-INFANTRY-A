/* ============================================================
 * bsp_imu.h —— A 板板载 MPU6500 驱动
 * 硬件：SPI5  PF7(SCK) / PF8(MISO) / PF9(MOSI)，CS = PF6
 * 配置：照 ARBATOS boards\DjiAF427\devices\Mpu6500.c
 * ============================================================ */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

/* 陀螺 Z 轴正方向修正：实测后填 +1.0f 或 -1.0f
   判据：手动朝「g_yaw_angle_acc 增大」的方向转云台，
        看 g_imu_gyro_z_raw 是正还是负；反了就填 -1.0f */
#define BSP_IMU_GYRO_Z_SIGN   (+1.0f)

#define BSP_IMU_TIMEOUT_MS    20U   /* 超过这么久没读到 → 判掉线 */

void bsp_imu_init(void);      /* 上电调用一次：复位 + 自检 + 配置 */
void bsp_imu_update(void);    /* 1kHz 调用一次：读一帧 + 在线判定 */

/* ---- Watch 用 ---- */
extern volatile float    g_imu_gyro_z;      /* 陀螺 Z 轴角速度 (rad/s)，已乘 SIGN */
extern volatile int16_t  g_imu_gyro_z_raw;  /* 原始读数（定符号用）*/
extern volatile int16_t  g_imu_gyro_x_raw;
extern volatile int16_t  g_imu_gyro_y_raw;
extern volatile uint8_t  g_imu_whoami;      /* 自检：应为 0x70 */
extern volatile uint8_t  g_imu_online;      /* 1 = 20ms 内读到过 */
extern volatile uint32_t g_imu_read_ok;     /* 成功帧数 */
extern volatile uint32_t g_imu_read_fail;   /* 失败帧数 */

#endif