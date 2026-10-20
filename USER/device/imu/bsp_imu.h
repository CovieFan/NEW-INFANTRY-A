/* ============================================================
 * bsp_imu.h —— A 板板载 MPU6500 驱动
 * 硬件：SPI5  PF7(SCK) / PF8(MISO) / PF9(MOSI)，CS = PF6
 * 配置：照 ARBATOS boards\DjiAF427\devices\Mpu6500.c
 * ============================================================ */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

/* 陀螺 Z 轴正方向修正：实测后填 +1.0f 或 -1.0f
   判据：手动朝「yaw_angle_acc 增大」的方向转云台，
        看 imu_gyro_z_raw 是正还是负；反了就填 -1.0f */
#define BSP_IMU_GYRO_Z_SIGN   (+1.0f)

/* ▲上面这个宏已被「投影法」取代：现在只给诊断变量 imu_gyro_z 用，
   真正参与控制的是下面这个 YAW 符号 */
/* ★真 yaw 轴角速度的符号修正：实测后填 +1.0f 或 -1.0f
   判据：手转云台朝「yaw_angle_acc 增大」的方向，看 imu_gyro_yaw 是正是负 */
#define BSP_IMU_YAW_SIGN      (+1.0f)

/* 重力方向低通系数：越小越稳、越慢跟上倾斜。0.002 在 ~3kHz 主循环下 ≈ 1Hz */
#define BSP_IMU_GRAV_ALPHA    (0.002f)

/* 加计模长平方下限：|a| < 2000LSB(≈0.49g) 认为加计不可信 → 投影输出 0
   （实测静止 |a| ≈ 4400 ⇒ n2 ≈ 1.9e7，余量很大）*/
#define BSP_IMU_GRAV_MIN_N2   (4000000.0f)

#define BSP_IMU_TIMEOUT_MS    20U   /* 超过这么久没读到 → 判掉线 */
/* ---- 上电静止零偏标定 ----
   为什么必须做：速控云台的反馈全是陀螺；零偏会让云台匀速自己转
   （实测零偏 ≈ -0.0053 rad/s = -0.3 °/s ⇒ 每 20 秒漂 6°）。 */
#define BSP_IMU_CAL_MS        1000U   /* 上电后标定窗口长度(ms)；这段时间 imu_online 保持 0 */
#define BSP_IMU_CAL_MIN_CNT   500U    /* 窗口内至少采这么多帧才算标定成功，否则重来一轮 */

/* ★2026-10-13 实测沉淀（EmberProbe 在线测得）：
   上电 1 秒标定不可信 —— 同一块板两次上电分别标出 +0.0279 / -0.0468 rad/s（差 4.3 °/s）。
   故：① 预置下面这个实测值作兜底；② 只有「确实静止」的标定才允许覆盖它。
   实测关系：真实漂移速率 = 真实零偏 - 标定值；已用在线写入三步验证 1.233 -> 0.601 -> 0 °/s。
   实测条件：A 板立装、室温约 25 度、云台静止、底盘静止。 */
#define BSP_IMU_YAW_BIAS_DEFAULT  (-0.0674f)
#define BSP_IMU_CAL_MAX_DEV_LSB   100U   /* 标定窗口内三轴相对首帧的最大偏差上限(LSB) ≈3 °/s，超了判「动过」 */


void bsp_imu_init(void);      /* 上电调用一次：复位 + 自检 + 配置 */
void bsp_imu_update(void);    /* 1kHz 调用一次：读一帧 + 在线判定 */

/* ---- Watch 用 ---- */
extern volatile float    imu_gyro_z;      /* 陀螺 Z 轴角速度 (rad/s)，已乘 SIGN */
extern volatile int16_t  imu_gyro_z_raw;  /* 原始读数（定符号用）*/
extern volatile int16_t  imu_gyro_x_raw;
extern volatile int16_t  imu_gyro_y_raw;

extern volatile int16_t  imu_acc_x_raw;   /* 加计原始值（只取方向，不需要单位）*/
extern volatile int16_t  imu_acc_y_raw;
extern volatile int16_t  imu_acc_z_raw;
extern volatile float    imu_acc_norm;  /* ★加计三轴模长 |a| (LSB)：静止 ≈ 4096 */
extern volatile float    imu_gyro_yaw;    /* ★投影后的真 yaw 角速度 (rad/s) */
extern volatile float    imu_grav_ux;   /* ★重力方向单位向量（机体系），平方和 = 1 */
extern volatile float    imu_grav_uy;
extern volatile float    imu_grav_uz;


extern volatile uint8_t  imu_whoami;      /* 自检：应为 0x70 */
extern volatile uint8_t  imu_online;      /* 1 = 20ms 内读到过 */
extern volatile uint32_t imu_read_ok;     /* 成功帧数 */
extern volatile uint32_t imu_read_fail;   /* 失败帧数 */

extern volatile float    imu_gyro_z_bias; /* 标定出的零偏 (rad/s) */
extern volatile uint8_t  imu_cal_ok;      /* 1 = 零偏标定完成 */
extern volatile uint32_t imu_cal_cnt;     /* 标定窗口内采到的帧数 */

#endif
