#include "function.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "filter.h"
#include "can.h"


/* Macro definition ---------------------------------------------------------------*/
#define TEST_NUM 30
#define VOLTAGE 5.0 / 3.3
#define KILOGRAM_TO_NEWTON  9.8
#define DEAD_ZONE  10.0f     /* 力值死区: ±10 范围内保持不动 */
#define FORCE_STEP  2.0f      /* 力值档位间隔: ±10N~±30N 每2N一档, 对应1km/h */
#define SPEED_MAX   10        /* 最大速度档位 (km/h) */

/* Variable definition -------------------------------------------------------------*/
floatuint_t force[ADC_CHANNEL_NUM] = { 0 };
float zero[ADC_CHANNEL_NUM] = { 0 };
uint8_t matrix_count = 0;

/* External Variable references ---------------------------------------------------------*/
extern uint8_t rev_data[28];



/* Function definition -------------------------------------------------------------*/

/**
  * @brief  力值映射为速度档位
  * @param  fy: Y轴力值(N)
  * @retval 速度档位: -10(后退10km/h) ~ +10(前进10km/h), 0=驻车
  * @note   |fy|≤10N: 驻车; 10N~30N: 每2N一档, 对应1~10km/h
  */
static int16_t ForceToSpeed(float fy)
{
    int16_t speed = 0;

    if (fy > DEAD_ZONE) {
        speed = (int16_t)((fy - DEAD_ZONE) / FORCE_STEP) + 1;  /* 前进 */
        if (speed > SPEED_MAX) {
            speed = SPEED_MAX;
        }
    } else if (fy < -DEAD_ZONE) {
        speed = (int16_t)((-fy - DEAD_ZONE) / FORCE_STEP) + 1; /* 后退 */
        if (speed > SPEED_MAX) {
            speed = SPEED_MAX;
        }
        speed = -speed;
    }
    return speed;
}

/**
  * @brief  车轮控制逻辑
  * @param  fx: X轴力值 (左右方向 → 控制转向)
  * @param  fy: Y轴力值 (前后方向 → 控制前进/后退速度)
  * @note   力值在 ±DEAD_ZONE 范围内时保持不动
  *         驱动: ±10N~±30N 每2N一档, 对应 ±1~±10km/h (10档)
  *         转向: 超出阈值后左转/右转30度, 4轮同时控制(4WS)
  *         指令与上次结果相同时不重复发送, 只有变化时才发送新指令
  */
void Wheel_Control(float fx, float fy)
{
    static int16_t last_speed = 99;                /* 上次发送的速度档位(初始非有效值, 保证首次发送) */
    static const uint8_t *last_steer_cmd = NULL;   /* 上次发送的转向指令 */
    const uint8_t *steer_cmd;  /* 转向指令 */
    int16_t speed;             /* 速度档位 */

    /* ---- Y轴: 力值映射为速度档位(-10~+10) ---- */
    speed = ForceToSpeed(fy);

    /* ---- X轴: 左转/右转判断 ---- */
    if (fx > DEAD_ZONE) {
        steer_cmd = CMD_RIGHT_DATA;         /* 右转 */
    } else if (fx < -DEAD_ZONE) {
        steer_cmd = CMD_LEFT_DATA;          /* 左转 */
    } else {
        steer_cmd = CMD_STRAIGHT_DATA;      /* 直行(转向回正) */
    }

    /* ---- 驱动/转向指令均与上次相同, 无需重新发送 ---- */
    if (speed == last_speed && steer_cmd == last_steer_cmd) {
        return;
    }

    /* ---- 速度档位发生变化, 发送驱动指令到4个轮子 ---- */
    if (speed != last_speed) {
        CAN_SendDriveSpeedMsg(WHEEL_LF, speed);  /* 左前轮 */
        CAN_SendDriveSpeedMsg(WHEEL_RF, speed);  /* 右前轮 */
        CAN_SendDriveSpeedMsg(WHEEL_LR, speed);  /* 左后轮 */
        CAN_SendDriveSpeedMsg(WHEEL_RR, speed);  /* 右后轮 */
        last_speed = speed;
    }

    /* ---- 转向指令发生变化, 发送转向指令到4个轮子 ---- */
    if (steer_cmd != last_steer_cmd) {
        CAN_SendSteerMsg(WHEEL_LF, steer_cmd);  /* 左前轮 */
        CAN_SendSteerMsg(WHEEL_RF, steer_cmd);  /* 右前轮 */
        CAN_SendSteerMsg(WHEEL_LR, steer_cmd);  /* 左后轮 */
        CAN_SendSteerMsg(WHEEL_RR, steer_cmd);  /* 右后轮 */
        last_steer_cmd = steer_cmd;
    }
}

extern uint8_t can_rx_data[8];
extern uint8_t car_count;
uint8_t q = 0, z = 0;

void send_ret(void)
{
    float a[ADC_CHANNEL_NUM] = {}, a_tmp;
    for (uint8_t i = 0; i < ADC_CHANNEL_NUM; ++i) {
        a_tmp = adcData() - zero[i];
        a[i] = a_tmp;
        a[i] = a[i] * VOLTAGE;
    }

    /* 根据3维力传感器数据控制车轮运动 */
    /* a[0] = X轴 → 左右转向    */
    /* a[1] = Y轴 → 前进/后退   */
    /* a[2] = Z轴 → 暂不使用    */
    Wheel_Control(a[0], a[1]);
//    Wheel_Control(can_rx_data[6], can_rx_data[7]);
}

void zero_clearing(void)
{
    float sum[ADC_CHANNEL_NUM] = { 0 };
    float adc_data[TEST_NUM][ADC_CHANNEL_NUM] = { 0 };
    uint8_t i = 0, j = 0;
    for (i = 0; i < TEST_NUM; i++) {
        HAL_Delay(20);
        for (j = 0; j < ADC_CHANNEL_NUM; j++) {
            adc_data[i][j] = adcData();
            sum[j] += adc_data[i][j];
        }
    }
    for (i = 0; i < ADC_CHANNEL_NUM; i++) {
        zero[i] = sum[i] / TEST_NUM;
    }
    return;
}
