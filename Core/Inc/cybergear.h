#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include "main.h"
#include "string.h"

/* ── CAN komande ─────────────────────────────────────────── */
#define CG_CMD_POSITION                  0x01
#define CG_CMD_REQUEST                   0x02
#define CG_CMD_ENABLE                    0x03
#define CG_CMD_STOP                      0x04
#define CG_CMD_SET_MECH_POSITION_TO_ZERO 0x06
#define CG_CMD_SET_CAN_ID                0x07
#define CG_CMD_RAM_READ                  0x11
#define CG_CMD_RAM_WRITE                 0x12
#define CG_CMD_GET_STATUS                0x15

/* ── Register naslovi ────────────────────────────────────── */
#define CG_ADDR_SPEED_KP              0x2014
#define CG_ADDR_SPEED_KI              0x2015
#define CG_ADDR_POSITION_KP           0x2016
#define CG_ADDR_RUN_MODE              0x7005
#define CG_ADDR_I_REF                 0x7006
#define CG_ADDR_SPEED_REF             0x700A
#define CG_ADDR_LIMIT_TORQUE          0x700B
#define CG_ADDR_CURRENT_KP            0x7010
#define CG_ADDR_CURRENT_KI            0x7011
#define CG_ADDR_CURRENT_FILTER_GAIN   0x7014
#define CG_ADDR_POSITION_REF          0x7016
#define CG_ADDR_LIMIT_SPEED           0x7017
#define CG_ADDR_LIMIT_CURRENT         0x7018

/* ── Načini delovanja ────────────────────────────────────── */
#define CG_MODE_MOTION                0x00
#define CG_MODE_POSITION              0x01
#define CG_MODE_SPEED                 0x02
#define CG_MODE_CURRENT               0x03

/* ── Meje vrednosti ──────────────────────────────────────── */
#define CG_POS_MIN                   -12.5f
#define CG_POS_MAX                    12.5f
#define CG_V_MIN                     -30.0f
#define CG_V_MAX                      30.0f
#define CG_T_MIN                     -12.0f
#define CG_T_MAX                      12.0f
#define CG_I_MIN                     -27.0f
#define CG_I_MAX                      27.0f

/* ── Konfiguracijske konstante ───────────────────────────── */
#define CG_NUM_MOTORS                 4
#define CG_DEFAULT_LIMIT_CURRENT      3.0f   // A
#define CG_DEFAULT_LIMIT_SPEED        2.0f  // rad/s za position following
#define CG_INIT_DELAY_MS              10     // delay med ukazi pri init

/* ── Return kode ─────────────────────────────────────────── */
#define CG_OK                         0x00
#define CG_ERR                        0x01

/* ═══════════════════════════════════════════════════════════
 *  Javne funkcije
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Inicializira vse 4 motorje v current mode.
 *         Zaporedje za vsak motor: stop → set_run_mode(CURRENT) →
 *         set_limit_current → enable.
 * @param  hcan   Pointer na CAN handle (npr. &hcan1)
 * @param  ids    Array s 4 motor ID-ji (npr. {6,5,7,8})
 */
void CG_InitAllMotors(CAN_HandleTypeDef *hcan, uint8_t ids[CG_NUM_MOTORS]);

/**
 * @brief  Nastavi tok motorja v A (-27 .. +27).
 */
void CG_SetCurrent(CAN_HandleTypeDef *hcan, uint8_t motor_id, float current_a);

/**
 * @brief  Nastavi trenutno pozicijo motorja kot mehansko nulo (0x06).
 */
void CG_SetMechZero(CAN_HandleTypeDef *hcan, uint8_t motor_id);

/**
 * @brief  Nastavi hitrost motorja v rad/s (-30 .. +30).
 */
void CG_SetSpeed(CAN_HandleTypeDef *hcan, uint8_t motor_id, float speed_rads);

/**
 * @brief  Nastavi pozicijo motorja v rad (-12.5 .. +12.5).
 */
void CG_SetPosition(CAN_HandleTypeDef *hcan, uint8_t motor_id, float position);

/**
 * @brief  Nastavi limit toka motorja.
 */
void CG_SetLimitCurrent(CAN_HandleTypeDef *hcan, uint8_t motor_id, float current_a);

/**
 * @brief  Nastavi limit hitrosti motorja.
 */
void CG_SetLimitSpeed(CAN_HandleTypeDef *hcan, uint8_t motor_id, float speed_rads);

/**
 * @brief  Nastavi run mode motorja (CG_MODE_SPEED, CG_MODE_POSITION ...).
 */
void CG_SetRunMode(CAN_HandleTypeDef *hcan, uint8_t motor_id, uint8_t mode);

/**
 * @brief  Omogoči motor (CMD_ENABLE).
 */
void CG_Enable(CAN_HandleTypeDef *hcan, uint8_t motor_id);

/**
 * @brief  Ustavi motor (CMD_STOP).
 */
void CG_Stop(CAN_HandleTypeDef *hcan, uint8_t motor_id);

/**
 * @brief  Ustavi vse 4 motorje.
 */
void CG_StopAll(CAN_HandleTypeDef *hcan, uint8_t ids[CG_NUM_MOTORS]);

#endif /* CYBERGEAR_H */
