/* input.c —— 输入抽象层（参数进表，代码只查表）
   通道号 / 是否翻转 写在下面这张表里，应用层完全不知道。 */
#include "input.h"
#include "remote.h"

/* ---- 参数表：角色 → {通道号, 是否翻符号}（对照 ARBATOS 的 .rc_ch / .invert）---- */
typedef struct
{
    uint8_t ch;        /* 0~3；0xFF = 该角色不用 */
    uint8_t invert;    /* 1 = 把符号翻转 */
} input_map_t;

static const input_map_t s_input_map[INPUT_AXIS_COUNT] =
{
    /* INPUT_AXIS_CHASSIS_X    */ { INPUT_CH_CHASSIS_X,    0U },
    /* INPUT_AXIS_CHASSIS_Y    */ { INPUT_CH_CHASSIS_Y,    0U },
    /* INPUT_AXIS_CHASSIS_WZ   */ { INPUT_CH_CHASSIS_WZ,   0U },
    /* INPUT_AXIS_GIMBAL_YAW   */ { INPUT_CH_GIMBAL_YAW,   0U },
    /* INPUT_AXIS_GIMBAL_PITCH */ { INPUT_CH_GIMBAL_PITCH, 0U },
};

/* ---- 摇杆行程（实测标定）----
   rc_full_pos[ch] = 通道 ch「读数为正」方向推到底的幅度
   rc_full_neg[ch] = 通道 ch「读数为负」方向推到底的幅度
   正常轴两个方向都是 660（DJI：1684-1024）。
   ⚠️ ch2 的某一侧实测只有 ~150 —— 在这里单独补偿（是"补偿"，不是"修复"）。 */
static const float rc_full_pos[4] = { 660.0f, 660.0f, 150.0f, 660.0f };
static const float rc_full_neg[4] = { 660.0f, 660.0f, 660.0f, 660.0f };

static int16_t input_read_ch(uint8_t ch)
{
    switch (ch)
    {
        case 0U: return remote.ch0;
        case 1U: return remote.ch1;
        case 2U: return remote.ch2;
        case 3U: return remote.ch3;
        default: return 0;
    }
}

float input_axis(uint8_t role)
{
    const input_map_t *m;
    int16_t ch;
    float   v;
    float   a;
    float   fs;

    if (role >= INPUT_AXIS_COUNT) { return 0.0f; }

    m = &s_input_map[role];
    if (m->ch > 3U) { return 0.0f; }                   /* 0xFF = 该角色不用 */

    ch = input_read_ch(m->ch);
    v  = (float)ch;
    a  = (v < 0.0f) ? -v : v;

    if (a < (float)INPUT_DEADBAND) { return 0.0f; }     /* 死区内 -> 0 */

    fs = (v >= 0.0f) ? rc_full_pos[m->ch] : rc_full_neg[m->ch];
    if (fs <= (float)INPUT_DEADBAND) { return 0.0f; }   /* 防除零 */
    if (a > fs) { a = fs; }                             /* 超量程 -> 削平 */

    /* 死区外重映射：从死区边缘连续地长到 1（顺便修掉"刚出死区就跳"） */
    a = (a - (float)INPUT_DEADBAND) / (fs - (float)INPUT_DEADBAND);
    if (v < 0.0f)        { a = -a; }
    if (m->invert != 0U) { a = -a; }
    return a;
}