#include "cybergear.h"

/* ═══════════════════════════════════════════════════════════
 *  Interni (static) pomožni funkciji
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Pošlje CAN extended frame.
 */
static void _CG_SendFrame(CAN_HandleTypeDef *hcan, uint8_t cmd_id,uint8_t motor_id, uint8_t *data) {
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;

	txHeader.ExtId = ((uint32_t) cmd_id << 24) | motor_id;
	txHeader.IDE = CAN_ID_EXT;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;
	txHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox);
}

/**
 * @brief  Zapiše float parameter prek CMD_RAM_WRITE.
 *         Vrednost se omeji na [min, max].
 */
static void _CG_WriteFloat(CAN_HandleTypeDef *hcan, uint8_t motor_id,uint16_t addr, float value, float min, float max) {
	uint8_t data[8] = { 0 };

	// Omeji vrednost
	if (value > max)
		value = max;
	if (value < min)
		value = min;

	data[0] = (uint8_t) (addr & 0x00FF);
	data[1] = (uint8_t) (addr >> 8);
	memcpy(&data[4], &value, 4);

	_CG_SendFrame(hcan, CG_CMD_RAM_WRITE, motor_id, data);
}

/**
 * @brief  Zapiše uint8 parameter prek CMD_RAM_WRITE.
 */
static void _CG_WriteUint8(CAN_HandleTypeDef *hcan, uint8_t motor_id,uint16_t addr, uint8_t value) {
	uint8_t data[8] = { 0 };

	data[0] = (uint8_t) (addr & 0x00FF);
	data[1] = (uint8_t) (addr >> 8);
	data[4] = value;

	_CG_SendFrame(hcan, CG_CMD_RAM_WRITE, motor_id, data);
}

/* ═══════════════════════════════════════════════════════════
 *  Javne funkcije
 * ═══════════════════════════════════════════════════════════ */

void CG_Enable(CAN_HandleTypeDef *hcan, uint8_t motor_id) {
	uint8_t data[8] = { 0 };
	_CG_SendFrame(hcan, CG_CMD_ENABLE, motor_id, data);
}

void CG_Stop(CAN_HandleTypeDef *hcan, uint8_t motor_id) {
	uint8_t data[8] = { 0 };
	_CG_SendFrame(hcan, CG_CMD_STOP, motor_id, data);
}

void CG_StopAll(CAN_HandleTypeDef *hcan, uint8_t ids[CG_NUM_MOTORS]) {
	for (int i = 0; i < CG_NUM_MOTORS; i++) {
		CG_Stop(hcan, ids[i]);
		HAL_Delay(CG_INIT_DELAY_MS);
	}
}

void CG_SetRunMode(CAN_HandleTypeDef *hcan, uint8_t motor_id, uint8_t mode) {
	_CG_WriteUint8(hcan, motor_id, CG_ADDR_RUN_MODE, mode);
}

void CG_SetLimitCurrent(CAN_HandleTypeDef *hcan, uint8_t motor_id,
		float current_a) {
	_CG_WriteFloat(hcan, motor_id, CG_ADDR_LIMIT_CURRENT, current_a, 0.0f,CG_I_MAX);
}

void CG_SetLimitSpeed(CAN_HandleTypeDef *hcan, uint8_t motor_id,
		float speed_rads) {
	_CG_WriteFloat(hcan, motor_id, CG_ADDR_LIMIT_SPEED, speed_rads, 0.0f,CG_V_MAX);
}

void CG_SetSpeed(CAN_HandleTypeDef *hcan, uint8_t motor_id, float speed_rads) {
	_CG_WriteFloat(hcan, motor_id, CG_ADDR_SPEED_REF, speed_rads, CG_V_MIN,CG_V_MAX);
}

void CG_SetPosition(CAN_HandleTypeDef *hcan, uint8_t motor_id, float position) {
	_CG_WriteFloat(hcan, motor_id, CG_ADDR_POSITION_REF, position, CG_POS_MIN,
			CG_POS_MAX);
}

void CG_SetCurrent(CAN_HandleTypeDef *hcan, uint8_t motor_id, float current_a) {
	_CG_WriteFloat(hcan, motor_id, CG_ADDR_I_REF, current_a, CG_I_MIN, CG_I_MAX);
}

void CG_SetMechZero(CAN_HandleTypeDef *hcan, uint8_t motor_id) {
	uint8_t data[8] = { 0 };
	data[0] = 1;  // protokol zahteva Byte[0]=1
	_CG_SendFrame(hcan, CG_CMD_SET_MECH_POSITION_TO_ZERO, motor_id, data);
}

void CG_InitAllMotors(CAN_HandleTypeDef *hcan, uint8_t ids[CG_NUM_MOTORS]) {
	for (int i = 0; i < CG_NUM_MOTORS; i++) {
		// 1. Ustavi motor (motor mora biti ustavljen pred nastavo nule)
		CG_Stop(hcan, ids[i]);
		HAL_Delay(CG_INIT_DELAY_MS);

		// 2. Nastavi trenutno pozicijo kot mehansko nulo
		CG_SetMechZero(hcan, ids[i]);
		HAL_Delay(CG_INIT_DELAY_MS);

		// 3. Nastavi position mode
		CG_SetRunMode(hcan, ids[i], CG_MODE_POSITION);
		HAL_Delay(CG_INIT_DELAY_MS);

		// 4. Omogoči motor
		CG_Enable(hcan, ids[i]);
		HAL_Delay(CG_INIT_DELAY_MS);

		// 5. Nastavi limit hitrosti za position following
		CG_SetLimitSpeed(hcan, ids[i], CG_DEFAULT_LIMIT_SPEED);
		HAL_Delay(CG_INIT_DELAY_MS);

		// 6. Nastavi limit toka
		CG_SetLimitCurrent(hcan, ids[i], CG_DEFAULT_LIMIT_CURRENT);
		HAL_Delay(CG_INIT_DELAY_MS);
	}
}
