/*
 * control_loop.c
 *
 *  TIM2 @ 50 Hz (20 ms): BNO086 read → CyberGear positions → DDSM115 velocity
 *
 *  APB1 timer clock = 84 MHz (SYSCLK 84 MHz, APB1 /2 → PCLK1 42 MHz, timer ×2)
 *  PSC = 8399  →  tick = 84 MHz / 8400  = 10 kHz
 *  ARR =  199  →  rate = 10 kHz / 200   = 50 Hz  (20 ms)
 *
 *  TIM2 is driven via direct register access — no HAL TIM driver required.
 *  TIM2_IRQn priority = 5  (must be > 0 so SysTick can still tick inside HAL calls)
 */

#include "control_loop.h"
#include "demo_app.h"
#include "cybergear.h"
#include "DDSM115.h"

#define TARGET_ANGLE_RAD   1.0472f   /* 60° in radians */
#define LOOP_DURATION_MS   5000      /* čas delovanja motorjev v ms */

extern CAN_HandleTypeDef hcan1;

static uint8_t  motor_ID[]    = {6, 5, 7, 8};
static uint32_t loop_start_ms = 0;

volatile uint8_t stop_request = 0;

volatile uint32_t dt_bno   = 0;
volatile uint32_t dt_cg    = 0;
volatile uint32_t dt_ddsm  = 0;
volatile uint32_t dt_total = 0;

/* ------------------------------------------------------------------ */

static void TIM2_Init(void)
{
    /* Enable TIM2 peripheral clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1  = 0;
    TIM2->PSC  = 8399;   /* 84 MHz / 8400 = 10 kHz */
    TIM2->ARR  = 199;    /* 10 kHz / 200  = 50 Hz  */
    TIM2->CR1 |= TIM_CR1_ARPE;   /* auto-reload preload */
    TIM2->EGR  = TIM_EGR_UG;     /* load PSC/ARR immediately */
    TIM2->SR   = ~TIM_SR_UIF;    /* clear update flag set by EGR */
    TIM2->DIER = TIM_DIER_UIE;   /* enable update interrupt */
    TIM2->CR1 |= TIM_CR1_CEN;    /* start counter */
}

void Control_Loop_Init(void)
{
    DWT_Init();
    loop_start_ms = HAL_GetTick();
    TIM2_Init();
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ------------------------------------------------------------------ */

static void Control_Loop_Run(void)
{
    /* --- Stop po preteku časa --- */
    if (HAL_GetTick() - loop_start_ms >= LOOP_DURATION_MS)
    {
        if (!stop_request)
        {
            sendVelocityCommand(0x10, 0.0f);
            sendVelocityCommand(0x30, 0.0f);
            stop_request = 1;   /* main() bo ustavil CG motorje */
        }
        BNO_App();
        return;
    }

    uint32_t t0, t1, t2, t3;

    /* --- BNO086 read --- */
    t0 = DWT_GetMicros();
    BNO_App();
    t1 = DWT_GetMicros();
    dt_bno = t1 - t0;

    /* --- CyberGear position commands --- */
    /* 300 µs DWT busy-wait between commands: enough for one CAN frame (~130 µs
       at 1 Mbps) to leave the TX mailbox. DWT is interrupt-independent — safe in ISR. */
    #define DWT_DELAY_US(us) do { uint32_t _t = DWT_GetMicros(); \
        while ((DWT_GetMicros() - _t) < (us)); } while(0)

    CG_SetPosition(&hcan1, motor_ID[0],  TARGET_ANGLE_RAD); DWT_DELAY_US(300);
    CG_SetPosition(&hcan1, motor_ID[1], -TARGET_ANGLE_RAD); DWT_DELAY_US(300);
    CG_SetPosition(&hcan1, motor_ID[2], -TARGET_ANGLE_RAD); DWT_DELAY_US(300);
    CG_SetPosition(&hcan1, motor_ID[3],  TARGET_ANGLE_RAD);
    t2 = DWT_GetMicros();
    dt_cg = t2 - t1;

    /* --- DDSM115 velocity commands --- */
    sendVelocityCommand(0x10,  0.0f);
    sendVelocityCommand(0x30, -0.0f);
    t3 = DWT_GetMicros();
    dt_ddsm = t3 - t2;

    dt_total = t3 - t0;
}

/* ------------------------------------------------------------------ */

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR = ~TIM_SR_UIF;   /* clear flag before processing */
        Control_Loop_Run();
    }
}
