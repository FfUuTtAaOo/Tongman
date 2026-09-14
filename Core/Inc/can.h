/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* 车轮驱动 CAN ID (前进/后退) */
#define CAN_ID_DRIVE_LF  0x12  /* 左前轮驱动 */
#define CAN_ID_DRIVE_RF  0x15  /* 右前轮驱动 */
#define CAN_ID_DRIVE_LR  0x18  /* 左后轮驱动 */
#define CAN_ID_DRIVE_RR  0x1B  /* 右后轮驱动 */

/* 车轮转向 CAN ID (左转/右转) */
#define CAN_ID_STEER_LF  0x13  /* 左前轮转向 */
#define CAN_ID_STEER_RF  0x16  /* 右前轮转向 */
#define CAN_ID_STEER_LR  0x19  /* 左后轮转向 */
#define CAN_ID_STEER_RR  0x1C  /* 右后轮转向 */

/* 测试用: FX/FY轴力值上报 CAN ID */
#define CAN_ID_FX_TEST   0x100  /* FX (X轴) */
#define CAN_ID_FY_TEST   0x101  /* FY (Y轴) */

/* 模式切换指令 CAN ID (接收): 数据 00 00 00 00 00 00 00 01=矩阵 / ...00 02=阿克曼 */
#define CAN_ID_MODE_CMD  0x100

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

/* 车轮索引 */
typedef enum {
    WHEEL_LF = 0,   /* 左前轮 */
    WHEEL_RF,       /* 右前轮 */
    WHEEL_LR,       /* 左后轮 */
    WHEEL_RR,       /* 右后轮 */
    WHEEL_NUM       /* 车轮总数 */
} WheelIndex_t;

/* 车轮控制模式: 决定驱动/转向指令的倒数第二位(byte6)
 *   阿克曼模式: byte6 = 0x04
 *   矩阵模式:   byte6 = 0x00
 */
typedef enum {
    WHEEL_MODE_MATRIX = 0,   /* 矩阵模式 */
    WHEEL_MODE_ACKERMANN     /* 阿克曼模式 */
} WheelMode_t;

#define WHEEL_MODE_BYTE_ACK     0x04u  /* 阿克曼模式指令字节 */
#define WHEEL_MODE_BYTE_MATRIX  0x00u  /* 矩阵模式指令字节 */

/* 车轮控制模式设置/获取 (上电默认阿克曼模式) */
void CAN_SetWheelMode(WheelMode_t mode);
WheelMode_t CAN_GetWheelMode(void);
uint8_t CAN_GetWheelModeByte(void);   /* 当前模式对应的 byte6: 0x04/0x00 */

void CAN_Wheel_Init(void);
void configure_CAN_filter(void);
HAL_StatusTypeDef CAN_SendDriveMsg(WheelIndex_t wheel, const uint8_t *data);
HAL_StatusTypeDef CAN_SendDriveSpeedMsg(WheelIndex_t wheel, int16_t speed_kmh);
HAL_StatusTypeDef CAN_SendSteerMsg(WheelIndex_t wheel, const uint8_t *data);

/* 测试用: 将FX/FY轴力值按指定格式通过CAN发出 */
void CAN_SendFXData(float fx);
void CAN_SendFYData(float fy);

/* 指令数据数组 (转向指令倒数第二位byte6: 0x04=阿克曼模式, 0x00=矩阵模式) */
extern const uint8_t CMD_LEFT_ACK[8];      /* 阿克曼 左转30度 */
extern const uint8_t CMD_RIGHT_ACK[8];     /* 阿克曼 右转30度 */
extern const uint8_t CMD_LEFT_MATRIX[8];   /* 矩阵 左转30度 */
extern const uint8_t CMD_RIGHT_MATRIX[8];  /* 矩阵 右转30度 */
extern const uint8_t CMD_STRAIGHT_DATA[8]; /* 直行/驻车(转向轴回正), 两种模式一致 */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

