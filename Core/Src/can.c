/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ========== 车轮控制指令数据 (8字节) ========== */
/* 转向指令倒数第二位(byte6): 0x04=阿克曼模式, 0x00=矩阵模式 */
const uint8_t CMD_LEFT_ACK[8]      = {0x00, 0x00, 0xB8, 0x8B, 0x00, 0x00, 0x04, 0x00}; /* 阿克曼 左转30度 */
const uint8_t CMD_RIGHT_ACK[8]     = {0x00, 0x00, 0x48, 0x74, 0x00, 0x00, 0x04, 0x00}; /* 阿克曼 右转30度 */
const uint8_t CMD_LEFT_MATRIX[8]   = {0x00, 0x00, 0xB8, 0x8B, 0x00, 0x00, 0x00, 0x00}; /* 矩阵 左转30度 */
const uint8_t CMD_RIGHT_MATRIX[8]  = {0x00, 0x00, 0x48, 0x74, 0x00, 0x00, 0x00, 0x00}; /* 矩阵 右转30度 */
const uint8_t CMD_STRAIGHT_DATA[8] = {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00}; /* 直行/驻车(转向轴回正) */

/* 车轮控制模式, 上电默认阿克曼模式 */
static WheelMode_t wheel_mode = WHEEL_MODE_ACKERMANN;

/**
  * @brief  设置车轮控制模式
  */
void CAN_SetWheelMode(WheelMode_t mode)
{
    wheel_mode = mode;
}

/**
  * @brief  获取当前车轮控制模式
  */
WheelMode_t CAN_GetWheelMode(void)
{
    return wheel_mode;
}

/**
  * @brief  获取当前模式对应的指令字节 (驱动/转向帧的倒数第二位)
  */
uint8_t CAN_GetWheelModeByte(void)
{
    return (wheel_mode == WHEEL_MODE_ACKERMANN) ? WHEEL_MODE_BYTE_ACK : WHEEL_MODE_BYTE_MATRIX;
}

/* 车轮控制 CAN 发送帧头: 每个轮子独立一组 (驱动 + 转向) */
typedef struct {
    CAN_TxHeaderTypeDef Drive;  /* 驱动帧头 (前进/后退) */
    CAN_TxHeaderTypeDef Steer;  /* 转向帧头 (左转/右转) */
} WheelCanTx_t;

CAN_RxHeaderTypeDef   RxMessage;
uint8_t rx_msg[8] = { 0 };
uint8_t can_rx_data[8] = { 0 };

static WheelCanTx_t WheelTx[WHEEL_NUM];  /* 4个变量, 分别对应 左前/右前/左后/右后 */

/**
  * @brief  初始化单个 CAN 发送帧头
  */
static void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *header, uint32_t stdId)
{
    header->StdId = stdId;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8;
    header->TransmitGlobalTime = DISABLE;
}

/**
  * @brief  初始化4个车轮的 CAN 发送帧头
  */
void CAN_Wheel_Init(void)
{
    static const uint32_t driveIds[WHEEL_NUM] = {
        CAN_ID_DRIVE_LF, CAN_ID_DRIVE_RF, CAN_ID_DRIVE_LR, CAN_ID_DRIVE_RR
    };
    static const uint32_t steerIds[WHEEL_NUM] = {
        CAN_ID_STEER_LF, CAN_ID_STEER_RF, CAN_ID_STEER_LR, CAN_ID_STEER_RR
    };
    uint8_t i;

    for (i = 0; i < WHEEL_NUM; i++) {
        CAN_TxHeader_Init(&WheelTx[i].Drive, driveIds[i]);
        CAN_TxHeader_Init(&WheelTx[i].Steer, steerIds[i]);
    }
}

/**
  * @brief  等待空闲邮箱并发送一帧 CAN 消息
  * @note   CAN1 只有3个发送邮箱, 连续发送多帧时必须等待有空闲邮箱, 否则后续帧会被丢弃
  */
static HAL_StatusTypeDef CAN_SendMsg(CAN_TxHeaderTypeDef *header, const uint8_t *data)
{
    uint32_t txMailbox;
    uint32_t timeout = 100000; /* 等待超时计数 */

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        if (--timeout == 0)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(&hcan, header, data, &txMailbox);
}

/**
  * @brief  发送驱动指令到指定车轮
  * @param  wheel: 车轮索引 (WHEEL_LF / WHEEL_RF / WHEEL_LR / WHEEL_RR)
  * @param  data:  8字节数据指针
  * @retval HAL 状态
  */
HAL_StatusTypeDef CAN_SendDriveMsg(WheelIndex_t wheel, const uint8_t *data)
{
    if (wheel >= WHEEL_NUM) {
        return HAL_ERROR;
    }
    return CAN_SendMsg(&WheelTx[wheel].Drive, data);
}

/**
  * @brief  速度值(-10~+10km/h)编码为驱动轴8字节CAN帧数据
  * @param  speed_kmh: 速度值, -10=后退10km/h, +10=前进10km/h, 0=驻车
  * @param  data:  输出的8字节数据
  * @note   编码规则: byte0/byte1 小端存放速度值(含方向),
  *         0(0x0000) = -10km/h, 65535(0xFFFF) = +10km/h, 32768(0x8000) = 驻车
  *         byte6: 模式位, 阿克曼=0x04, 矩阵=0x00 (前进/后退相同)
  */
static void CAN_BuildDriveData(int16_t speed_kmh, uint8_t *data)
{
    uint16_t value;
    int16_t speed = speed_kmh;

    if (speed > 10)  speed = 10;
    if (speed < -10) speed = -10;

    /* 速度映射: value = (speed+10)/20 * 65535, 四舍五入 */
    value = (uint16_t)(((uint32_t)(speed + 10) * 65535u + 10u) / 20u);

    data[0] = (uint8_t)(value & 0xFF);        /* 速度低字节 */
    data[1] = (uint8_t)((value >> 8) & 0xFF); /* 速度高字节 */
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = CAN_GetWheelModeByte();         /* 模式位: 阿克曼0x04 / 矩阵0x00 */
    data[7] = 0x00;
}

/**
  * @brief  按速度值发送驱动指令到指定车轮
  * @param  wheel: 车轮索引 (WHEEL_LF / WHEEL_RF / WHEEL_LR / WHEEL_RR)
  * @param  speed_kmh: 速度值, -10(后退10km/h) ~ +10(前进10km/h), 0=驻车
  * @retval HAL 状态
  */
HAL_StatusTypeDef CAN_SendDriveSpeedMsg(WheelIndex_t wheel, int16_t speed_kmh)
{
    uint8_t data[8];

    if (wheel >= WHEEL_NUM) {
        return HAL_ERROR;
    }
    CAN_BuildDriveData(speed_kmh, data);
    return CAN_SendMsg(&WheelTx[wheel].Drive, data);
}

/**
  * @brief  发送转向指令到指定车轮
  * @param  wheel: 车轮索引 (WHEEL_LF / WHEEL_RF / WHEEL_LR / WHEEL_RR)
  * @param  data:  8字节数据指针
  * @retval HAL 状态
  */
HAL_StatusTypeDef CAN_SendSteerMsg(WheelIndex_t wheel, const uint8_t *data)
{
    if (wheel >= WHEEL_NUM) {
        return HAL_ERROR;
    }
    return CAN_SendMsg(&WheelTx[wheel].Steer, data);
}

/**
  * @brief  测试用: 将力值按指定格式通过CAN发出 (8字节)
  * @param  stdId: CAN 标准帧 ID
  * @param  value: 力值(浮点), 正=正向, 负=反向
  * @note   数据格式(以 value = 0.123456 为例): 00 00 01 02 03 04 05 06
  *           byte0~1: 整数部分, 16位有符号大端(负值用补码, 如 -1 -> FF FF)
  *           byte2~7: 小数部分6位, 每位占1个字节能取0~9, 高位在前
  *         即: 整数用2字节表示, 小数位用6字节表示, 共8字节
  */
static void CAN_SendForceData(uint32_t stdId, float value)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    int32_t scaled;      /* value 放大 1e6 后的定点整数 */
    int32_t rem;         /* 小数部分(可能为负) */
    int16_t int_part;    /* 整数部分 */
    uint32_t frac;       /* 小数部分绝对值(0 ~ 999999) */
    uint32_t div = 100000u;
    uint8_t i;

    /* 四舍五入保留6位小数, 转为定点整数 */
    if (value >= 0.0f) {
        scaled = (int32_t)(value * 1000000.0f + 0.5f);
    } else {
        scaled = (int32_t)(value * 1000000.0f - 0.5f);
    }

    /* C99 除法向零截断, 取模结果符号与被除数一致 */
    int_part = (int16_t)(scaled / 1000000);
    rem      = scaled % 1000000;
    frac     = (rem < 0) ? (uint32_t)(-rem) : (uint32_t)rem;

    /* 整数部分: 16位有符号大端 */
    data[0] = (uint8_t)(((uint16_t)int_part >> 8) & 0xFF);
    data[1] = (uint8_t)((uint16_t)int_part & 0xFF);

    /* 小数部分: 6位十进制数字, 每位占1字节, 高位在前 */
    for (i = 0; i < 6; i++) {
        data[2 + i] = (uint8_t)((frac / div) % 10u);
        div /= 10u;
    }

    CAN_TxHeader_Init(&header, stdId);
    CAN_SendMsg(&header, data);
}

/**
  * @brief  测试用: 将FX轴力值通过CAN发出 (ID 0x100)
  */
void CAN_SendFXData(float fx)
{
    CAN_SendForceData(CAN_ID_FX_TEST, fx);
}

/**
  * @brief  测试用: 将FY轴力值通过CAN发出 (ID 0x101)
  */
void CAN_SendFYData(float fy)
{
    CAN_SendForceData(CAN_ID_FY_TEST, fy);
}

void configure_CAN_filter(void)
{
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE END 1 */
