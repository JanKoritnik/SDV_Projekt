/*
 * demo_app.h
 *
 *  Created on: Apr 7, 2026
 *      Author: blpongrac
 */

#ifndef INC_DEMO_APP_H_
#define INC_DEMO_APP_H_

#include "main.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// BNO086 SparkFun breakout: ADDR pin low = 0x4A, high = 0x4B
#define BNO_ADDR  (0x4B << 1)

void BNO_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart, uint16_t intPin);
void BNO_App(void);

void     DWT_Init(void);
uint32_t DWT_GetMicros(void);

/* Busy-wait delay using DWT — interrupt-independent, safe in ISR */
#define DWT_DELAY_US(us) do { uint32_t _t = DWT_GetMicros(); \
    while ((DWT_GetMicros() - _t) < (uint32_t)(us)); } while(0)

// Rotation results (written by sensor callback, read by application)
extern volatile float bno_roll, bno_pitch, bno_yaw;   // degrees, from quaternion
extern volatile float bno_qw, bno_qx, bno_qy, bno_qz; // raw quaternion
extern volatile float bno_gx, bno_gy, bno_gz;

/* User button (B1, PC13) — postavi na 1 ob pritisku, beri/pobriši kjerkoli */
extern volatile uint8_t g_btn_flag;

#endif /* INC_DEMO_APP_H_ */
