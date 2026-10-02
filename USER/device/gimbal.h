#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/* ============ 云台电机（2026-09-29 定） ============
   YAW   = GM6020，挂 CAN1，反馈 0x205（拨码 ID 1），命令帧 0x1FF 槽位 0
   PITCH = 6623，  挂 CAN2，反馈 0x206（拨码 ID 2），命令帧 0x1FF 槽位 1
   依据：ARBATOS Robotconfig/INFANTRY-A/ConfigHardware.inc + MotorModelDb.c
         + 《6020电机.pdf》P6 拨码表 + 用户口述 */

#define GIMBAL_AXIS_YAW     0U
#define GIMBAL_AXIS_PITCH   1U
#define GIMBAL_AXIS_COUNT   2U

/* -------- 反馈 ID -------- */
#define GIMBAL_YAW_FB_ID    0x205U
#define GIMBAL_PITCH_FB_ID  0x206U


/* ============ 步5d：PITCH 开环（6623 @ CAN2）============
   6623 反馈 ID = 0x206 → 按 ARBATOS 规则进 0x1FF 的槽位 1（字节 2-3）
   ⚠️ 这一帧在【CAN2】上（不是 CAN1）—— 和 6020 那帧是两条总线、互不干扰 */
#define GIMBAL_PITCH_CMD_ID     0x1FFU   /* CAN2 上 6623 的控制帧 */
#define GIMBAL_PITCH_CMD_SLOT   1U       /* 0x206 - 0x205 = 1 */

/* ★测试期量程：先 1000
   （ARBATOS 记 6623 量程 ±5000，但我们【没有手册可核】，所以先小；不动就加倍）*/
#define GIMBAL_PITCH_CMD_MAX    2000

extern volatile int16_t  g_gimbal_tx_pitch;       /* 发给 6623 的命令值 */
extern volatile uint32_t g_gimbal_tx_pitch_ok;    /* 成功帧数 */
extern volatile uint32_t g_gimbal_tx_pitch_fail;  /* ⭐失败帧数（CAN2 专用，很关键）*/

void gimbal_send_pitch(int16_t cmd);



/* -------- 反馈帧格式（对应 ARBATOS 的两种 rx_format）--------
   RM_STD （6020 用）：角度@0 / 转速@2 / 实测电流@4 / 温度@6
   FMT_6623（6623 用）：角度@0 / 实测电流@2 / 设定电流回读@4 / 无转速 / 无温度
   ⚠️ 两者第 2-3 字节含义完全不同：一个是转速，一个是电流！ */
#define GIMBAL_FMT_RM_STD   0U
#define GIMBAL_FMT_6623     1U

typedef struct
{
    int16_t  angle;         /* 机械角度 0~8191（对应 0~360°），会回绕 */
    int16_t  speed_rpm;     /* ⚠️ has_speed==0 时恒为 0，不是真实转速 */
    int16_t  current_set;   /* 设定电流回读：RM_STD 用偏移 4；6623 用偏移 4 */
    int16_t  current_meas;  /* 实测电流：RM_STD 用偏移 4；6623 用偏移 2 */
    uint8_t  temp;          /* 温度℃；6623 无此字段 → 恒为 0 */
    uint8_t  has_speed;     /* 1 = 该电机有转速反馈 */
    uint8_t  has_temp;      /* 1 = 该电机有温度反馈 */
    uint8_t  online;        /* 1 = 至少收到过一次反馈 */
    uint32_t last_ms;       /* 最近一次收到反馈的时刻 */
    uint8_t  raw[8];        /* 最近一帧原始 8 字节，方便手工核对 */
} gimbal_motor_t;

extern volatile gimbal_motor_t gimbal_fb[GIMBAL_AXIS_COUNT];
extern volatile uint32_t       g_gimbal_rx_cnt[GIMBAL_AXIS_COUNT];

void gimbal_init(void);     /* 只注册接收；本步不发任何指令 */


/* ================= 步5b-1：YAW 开环（命令帧） =================
   ARBATOS CanCommandTxRouteHelpers.inc:148-153 的规则：
     反馈 ID 0x205~0x208 → 命令写进 0x1FF 的槽位 (ID - 0x205)
   我们 YAW 反馈 = 0x205 → 槽位 0 → 字节 0-1
   ⚠️ 0x1FF 是【电压】帧（手册范围 ±25000），我们保守限到 GIMBAL_YAW_CMD_MAX */
#define GIMBAL_YAW_CMD_ID       0x1FFU   /* CAN1 上 6020 的控制帧 */
#define GIMBAL_YAW_CMD_SLOT     0U       /* 0x205 - 0x205 = 0 */

/* ★总开关（保险丝）：0 = 云台恒发 0，不响应摇杆 */
#define GIMBAL_RAW_TEST_ENABLE  1U

/* ★测试期量程：先 3000，稳了再往上加（上限 16384） */
#define GIMBAL_YAW_CMD_MAX      3000

extern volatile int16_t  g_gimbal_tx_yaw;     /* 实际发出去的命令值（限幅后） */
extern volatile uint32_t g_gimbal_tx_ok;      /* 发送成功帧数 */
extern volatile uint32_t g_gimbal_tx_fail;    /* ⭐发送失败帧数（以前会被静默丢掉） */

void gimbal_run(void);               /* 1kHz 调用：本步 = 开环测试 */
void gimbal_send_yaw(int16_t cmd);   /* 组帧 + 发 0x1FF（5c 也会用） */



#endif