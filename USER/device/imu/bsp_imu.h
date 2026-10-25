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

/* 加计模长平方下限：|a| <= 2000LSB(≈0.49g) 拒绝投影，不参与标定或控制
   （实测静止 |a| ≈ 4400 ⇒ n2 ≈ 1.9e7，余量很大）*/
#define BSP_IMU_GRAV_MIN_N2   (4000000.0f)

#define BSP_IMU_TIMEOUT_MS    20U   /* 超过这么久没读到 → 判掉线 */
/* ---- 上电静止零偏标定 ----
   为什么必须做：速控云台的反馈全是陀螺；零偏会让云台匀速自己转
   （实测零偏 ≈ -0.0053 rad/s = -0.3 °/s ⇒ 每 20 秒漂 6°）。 */
#define BSP_IMU_CAL_MS        1000U   /* 上电后标定窗口长度(ms)；这段时间 imu_online 保持 0 */
#define BSP_IMU_CAL_MIN_CNT   500U    /* 窗口内至少采这么多帧才算标定成功，否则重来一轮 */

/* 旧投影标量兜底：三轴标定未通过时沿用，不能拆成三轴默认偏置。
   历史实测条件为A板立装、室温约25度；该值仍有姿态/温度适用性限制。 */
#define BSP_IMU_YAW_BIAS_DEFAULT  (-0.0674f)
#define BSP_IMU_CAL_MAX_DEV_LSB   100U   /* 标定窗口内三轴相对首帧的最大偏差上限(LSB) ≈3 °/s，超了判「动过」 */


typedef enum
{
    BSP_IMU_FAULT_NONE = 0,
    BSP_IMU_FAULT_INIT_SPI,
    BSP_IMU_FAULT_INIT_ID,
    BSP_IMU_FAULT_INIT_CONFIG,
    BSP_IMU_FAULT_SAMPLE_SPI,
    BSP_IMU_FAULT_SAMPLE_GRAVITY,
    BSP_IMU_FAULT_SAMPLE_NONFINITE,
    BSP_IMU_FAULT_CAL_STATE
} bsp_imu_fault_e;

typedef enum
{
    BSP_IMU_CAL_PENDING = 0,
    BSP_IMU_CAL_VECTOR,          /* 三轴静止均值已采用；先逐轴扣偏置，再投影 */
    BSP_IMU_CAL_LEGACY_DEFAULT   /* 本次均值未采用，沿用旧投影标量兜底 */
} bsp_imu_cal_source_e;

void bsp_imu_init(void);      /* 上电调用：复位、自检、配置写入及读回 */
void bsp_imu_update(void);    /* 主循环调用：读取、检查投影、标定或发布速度 */

/* ---- Watch 用 ---- */
extern volatile float    imu_gyro_z;      /* 陀螺 Z 轴角速度 (rad/s)，已乘 SIGN */
extern volatile int16_t  imu_gyro_z_raw;  /* 原始读数（定符号用）*/
extern volatile int16_t  imu_gyro_x_raw;
extern volatile int16_t  imu_gyro_y_raw;

extern volatile int16_t  imu_acc_x_raw;   /* 加计原始值（只取方向，不需要单位）*/
extern volatile int16_t  imu_acc_y_raw;
extern volatile int16_t  imu_acc_z_raw;
extern volatile float    imu_acc_norm;  /* ★加计三轴模长 |a| (LSB)：静止 ≈ 4096 */
extern volatile float    imu_gyro_yaw;    /* 修正零偏后的竖直角速度分量 (rad/s)，不等于绝对航向 */
extern volatile float    imu_grav_ux;   /* ★重力方向单位向量（机体系），平方和 = 1 */
extern volatile float    imu_grav_uy;
extern volatile float    imu_grav_uz;


extern volatile uint8_t  imu_whoami;      /* 自检：应为 0x70 */
extern volatile uint8_t  imu_online;      /* 初始化合格、标定结束、最近投影有效；SPI失败保留20ms超时 */
extern volatile uint8_t  imu_init_ok;     /* 1 = ID、配置写入与读回均通过 */
extern volatile uint8_t  imu_startup_done;/* 启动流程已结束：标定完成或初始化失败；供独立PITCH启动 */
extern volatile uint8_t  imu_sample_valid;/* 最近一次读取的投影有效；标定期间也可为1 */
extern volatile bsp_imu_fault_e imu_fault;
extern volatile uint32_t imu_invalid_count; /* SPI成功但投影/最终速度被拒绝的帧数 */
extern volatile uint32_t imu_read_ok;     /* SPI burst成功帧数，不等于投影有效帧数 */
extern volatile uint32_t imu_read_fail;   /* 失败帧数 */

extern volatile float    imu_gyro_bias_x; /* 传感器X/Y/Z轴零偏，rad/s，未乘SIGN */
extern volatile float    imu_gyro_bias_y;
extern volatile float    imu_gyro_bias_z;
extern volatile bsp_imu_cal_source_e imu_cal_source;
extern volatile float    imu_gyro_z_bias; /* 仅LEGACY_DEFAULT使用的旧投影标量；VECTOR不使用 */
extern volatile uint8_t  imu_cal_ok;      /* 1 = 流程结束；看cal_source区分三轴标定/旧兜底 */
extern volatile uint32_t imu_cal_cnt;     /* 标定窗口内采到的帧数 */

#endif
