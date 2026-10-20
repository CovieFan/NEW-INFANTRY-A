#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>

/* 
 * 职责：把"遥控器第几通道的原始值"变成"某个方向上的 ±1 意图"。
 * 不再关心通道号 / 死区 / 行程补偿 */
#define INPUT_AXIS_CHASSIS_X       0U   /* 底盘前进/后退（正 = 前进） */
#define INPUT_AXIS_CHASSIS_Y       1U   /* 底盘左右平移（正 = 左）   */
#define INPUT_AXIS_CHASSIS_WZ      2U   /* 底盘自转（正 = 左转）     */
#define INPUT_AXIS_GIMBAL_YAW      3U   /* 云台 yaw（正 = 向左）     */
#define INPUT_AXIS_GIMBAL_PITCH    4U   /* 云台 pitch（正 = 向上）   */
#define INPUT_AXIS_COUNT           5U

/*左横=2 左纵=3 右横=0 右纵=1 */
#define INPUT_CH_CHASSIS_X         3U
#define INPUT_CH_CHASSIS_Y         2U
#define INPUT_CH_CHASSIS_WZ        0U
/* ⚠️ 测试期临时映射：云台 yaw 借右摇杆纵向 ch1（避免和底盘抢 ch0）。
   正式版会改回 ch0 + 拨杆模式仲裁 —— 和 ARBATOS 的 INPUT_AXIS_GIMBAL_YAW = ch0 一致。 */
#define INPUT_CH_GIMBAL_YAW        0U
#define INPUT_CH_GIMBAL_PITCH      1U

#define INPUT_DEADBAND             60   /* 摇杆死区，防中位漂移 */

#define INPUT_SW_UP     1U/*上1中3下2*/
#define INPUT_SW_DOWN   2U
#define INPUT_SW_MID    3U

/* 底盘自转开关：拨杆在中位 -> 1；掉线 / 无效值 -> 0 */
uint8_t input_spin_on(void);
/* 返回该角色方向上的归一化意图：-1.0 ~ +1.0（死区内为 0，角色非法为 0） */
float input_axis(uint8_t role);

#endif