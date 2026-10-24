#include "input.h"
#include "remote.h"
typedef struct
{
    uint8_t chan_idx;                    /* 该角色用哪个物理通道（0~3） */
    uint8_t invert;    /* 1 = 把符号翻转 */
} input_map_t;

static const input_map_t input_map[INPUT_AXIS_COUNT] =
{
    { INPUT_CH_CHASSIS_X,    0U },
    { INPUT_CH_CHASSIS_Y,    0U },
    { INPUT_CH_CHASSIS_WZ,   0U },
    { INPUT_CH_GIMBAL_YAW,   1U }, /* 右杆横向反相：左推 -> YAW 左转 */
    { INPUT_CH_GIMBAL_PITCH, 0U },
};

/* 摇杆行程
   rc_full_pos[ch] = 通道 ch「读数为正」方向推到底的幅度
   rc_full_neg[ch] = 通道 ch「读数为负」方向推到底的幅度
   正常轴两个方向都是 660（DJI：1684-1024） */
static const float rc_full_pos[4] = { 660.0f, 660.0f, 660.0f, 660.0f };
static const float rc_full_neg[4] = { 660.0f, 660.0f, 660.0f, 660.0f };

static int16_t input_read_ch(uint8_t chan_idx)
{
    switch (chan_idx)
    {
        case 0U: return remote.ch0;
        case 1U: return remote.ch1;
        case 2U: return remote.ch2;
        case 3U: return remote.ch3;
        default: return 0;
    }
}

float input_axis(uint8_t axis_id)        
/* 把通道值归一化 [-1, 1] */
/*归一化 = 把不同范围的值，统一映射到同一个标准范围。*/
{
    const input_map_t *map;              /* 角色 -> 通道映射项 */
    int16_t raw;                         /* 通道读数：约 -660 ~ +660，中位 0 */
    float   raw_f;                       /* 读数转浮点 */
    float   mag;                         /* |raw_f|，幅值 */
    float   full_scale;                  /* 该通道"该方向"的满行程（现为 660） */
    float   norm;                        /* 返回范围 [-1, 1] */

    if (axis_id >= INPUT_AXIS_COUNT) { return 0.0f; }
    map = &input_map[axis_id];
    if (map->chan_idx > 3U) { return 0.0f; }   /* 越界保护*/

    raw   = input_read_ch(map->chan_idx);
    raw_f = (float)raw;//转浮点
    mag   = (raw_f < 0.0f) ? -raw_f : raw_f;//转绝对值

    if (mag < (float)INPUT_DEADBAND) { return 0.0f; }   /* 死区内归零 */

    full_scale = (raw_f >= 0.0f) ? rc_full_pos[map->chan_idx] : rc_full_neg[map->chan_idx];
    if (full_scale <= (float)INPUT_DEADBAND) { return 0.0f; }  /* 防除零 */
    if (mag > full_scale) { mag = full_scale; }                /* 超量程削平 */

    /* 死区外重映射：从死区边缘连续地长到 1（顺带修掉"刚出死区就跳"） */
    norm = (mag - (float)INPUT_DEADBAND) / (full_scale - (float)INPUT_DEADBAND);
    if (raw_f < 0.0f)      { norm = -norm; }
    if (map->invert != 0U) { norm = -norm; }
    return norm;
}


/*拨杆中位 = 底盘自转开启*/
uint8_t input_spin_on(void)
{
    if (remote.online == 0U) { return 0U; }
    return (remote.sw2 == INPUT_SW_MID) ? 1U : 0U;
}
