/*
 * regulator.c
 *
 *  LQR balance regulator za bipedalni robot.
 *
 *  Theta (kot nagiba) prihaja iz obstoječega BNO086 I2C/SHTP sistema
 *  (bno_pitch v stopinjah, pretvorimo v radiane).
 *  RPM koles posodablja uart_app.c v HAL_UART_RxCpltCallback
 *  na podlagi ID bajta v RS485 odgovoru.
 */

#include "regulator.h"
#include "DDSM115.h"
#include "demo_app.h"    /* bno_pitch, DWT_DELAY_US */

#define MOTOR_LEFT   0x10
#define MOTOR_RIGHT  0x30

/* Desni motor je mehansko obrnjen — potrebuje nasproten predznak toka */
#define MOTOR_RIGHT_SIGN  (1.0f)
#define MOTOR_LEFT_SIGN   ( -1.0f)

/* Čas [ms] ko regulator čaka (robot se dvigne) preden začne regulirati */
#define STARTUP_DELAY_MS  3000

static float    wheel_pos   = 0.0f;
static float    theta_prev  = 0.0f;
static uint32_t start_tick  = 0;

/* RPM — polni ju uart_app.c ob vsakem RS485 odgovoru */
volatile float g_rpm_left  = 0.0f;
volatile float g_rpm_right = 0.0f;

/* ------------------------------------------------------------------ */

void controller_init(void)
{
    wheel_pos  = 0.0f;
    theta_prev = 0.0f;
    g_rpm_left  = 0.0f;
    g_rpm_right = 0.0f;
    start_tick  = HAL_GetTick();
}

void controller_step(void)
{
    /* ── 0. STARTUP DELAY — čaka da robot dvignejo ───────────── */

    if (HAL_GetTick() - start_tick < STARTUP_DELAY_MS)
    {
        sendCurrentCommand(MOTOR_LEFT,  0.0f);
        DWT_DELAY_US(300);
        sendCurrentCommand(MOTOR_RIGHT, 0.0f);
        theta_prev = bno_pitch * (3.14159f / 180.0f);  /* sinhronizacija odvoda */
        return;
    }

    /* ── 1. PREBERI SENZORJE ─────────────────────────────────── */

    /* bno_pitch je v stopinjah (posodablja BNO_App iz SHTP) */
    float theta = bno_pitch * (3.14159f / 180.0f);

    float theta_dot = (theta - theta_prev) / TS;
    theta_prev = theta;

    float vel_l = g_rpm_left  * (2.0f * 3.14159f / 60.0f) * WHEEL_RADIUS;
    float vel_r = g_rpm_right * (2.0f * 3.14159f / 60.0f) * WHEEL_RADIUS;
    float wheel_vel = 0.5f * (vel_l + vel_r);

    wheel_pos += wheel_vel * TS;

    /* ── 2. VARNOSTNA ZAUSTAVITEV ────────────────────────────── */

    if (fabsf(theta) > (THETA_MAX_DEG * 3.14159f / 180.0f))
    {
        sendCurrentCommand(MOTOR_LEFT,  0.0f);
        DWT_DELAY_US(300);
        sendCurrentCommand(MOTOR_RIGHT, 0.0f);
        start_tick = HAL_GetTick();  /* resetiraj delay ko robot pade */
        wheel_pos  = 0.0f;
        theta_prev = 0.0f;
        return;
    }

    /* ── 3. NAPAKA STANJ ─────────────────────────────────────── */

    float e_pos   = wheel_pos - POS_DESIRE;
    float e_vel   = wheel_vel;
    float e_theta = theta;
    float e_tdot  = theta_dot;

    /* ── 4. LQR REGULACIJSKO PRAVILO ─────────────────────────── */
    /*
     *  u = -K * e
     *    = -(K_WHEEL_POS * e_pos
     *       + K_WHEEL_VEL * e_vel
     *       + K_THETA     * e_theta
     *       + K_THETA_DOT * e_tdot)
     *
     *  POZOR: če se robot destabilizira namesto stabilizira,
     *  obrni predznak:  iq = +(...) ali sendCurrentCommand(x, -iq)
     */
    float iq = -(  K_WHEEL_POS * e_pos
                 + K_WHEEL_VEL * e_vel
                 + K_THETA     * e_theta
                 + K_THETA_DOT * e_tdot );

    /* ── 5. OMEJI TOK ────────────────────────────────────────── */

    if (iq >  IQ_MAX) iq =  IQ_MAX;
    if (iq < -IQ_MAX) iq = -IQ_MAX;

    /* ── 6. POŠLJI NA MOTORJE ────────────────────────────────── */

    sendCurrentCommand(MOTOR_LEFT,  MOTOR_LEFT_SIGN  * iq);
    DWT_DELAY_US(300);
    sendCurrentCommand(MOTOR_RIGHT, MOTOR_RIGHT_SIGN * iq);
}
