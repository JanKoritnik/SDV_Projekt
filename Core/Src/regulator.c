/*
 * regulator.c
 *
 *  LQR balance regulator za bipedalni robot.
 *
 *  Theta (kot nagiba) prihaja iz obstoječega BNO086 I2C/SHTP sistema
 *  (bno_roll v stopinjah, pretvorimo v radiane).
 *  RPM koles posodablja uart_app.c v HAL_UART_RxCpltCallback
 *  na podlagi ID bajta v RS485 odgovoru.
 */

#include "regulator.h"
#include "DDSM115.h"
#include "demo_app.h"    /* bno_roll, DWT_DELAY_US */

#define MOTOR_LEFT   0x10
#define MOTOR_RIGHT  0x30

/* Desni motor je mehansko obrnjen — potrebuje nasproten predznak toka */
#define MOTOR_RIGHT_SIGN  (-1.0f)
#define MOTOR_LEFT_SIGN   ( 1.0f)

/* Čas [ms] ko regulator čaka (robot se dvigne) preden začne regulirati */
#define STARTUP_DELAY_MS  500

volatile float    wheel_pos      = 0.0f;
static float    theta_prev     = 0.0f;
static uint32_t start_tick     = 0;
static float    theta_offset   = 0.0f;   /* auto-kalibracija med startup delay */
static float    calib_sum      = 0.0f;
static uint32_t calib_count    = 0;
static float    reg_active     = 1.0f;   /* 1=regulator deluje, 0=ustavljen */
static uint16_t pr_pos_l = 0;
static uint16_t pr_pos_r = 0;
/* RPM — polni ju uart_app.c ob vsakem RS485 odgovoru */
volatile int16_t g_rpm_left  = 0;
volatile int16_t g_rpm_right = 0;

volatile uint16_t g_pos_left  = 0;
volatile uint16_t g_pos_right = 0;

/* Izhod regulatorja — za Live Expression / debugger */
volatile float g_iq = 0.0f;

/* ------------------------------------------------------------------ */

void controller_init(void)
{
    wheel_pos    = 0.0f;
    theta_prev   = 0.0f;
    theta_offset = 0.0f;
    calib_sum    = 0.0f;
    calib_count  = 0;
    g_rpm_left   = 0;
    g_rpm_right  = 0;
    reg_active   = 1.0f;
    start_tick   = HAL_GetTick();
}

void controller_step(void)
{
    /* ── 0. STARTUP DELAY — čaka da robot dvignejo ───────────── */

    if (HAL_GetTick() - start_tick < STARTUP_DELAY_MS)
    {
        sendCurrentCommand(MOTOR_LEFT,  0.0f);
        DWT_DELAY_US(3500);
        sendCurrentCommand(MOTOR_RIGHT, 0.0f);

        /* Naberi vzorce za kalibracijo ravnovesnega kota */
        calib_sum   += bno_roll;
        calib_count += 1;
        theta_prev   = (bno_roll - theta_offset) * (3.14159f / 180.0f);
        return;
    }

    /* ── 0b. ENKRATNA KALIBRACIJA — izvede se samo ob prvem izteku delay ── */
    if (calib_count > 0)
    {
        theta_offset = calib_sum / (float)calib_count;
        calib_sum    = 0.0f;
        calib_count  = 0;   /* 0 = kalibracija končana, ne ponavljaj */
        theta_prev   = (bno_roll - theta_offset) * (3.14159f / 180.0f);


        sendCurrentCommand(MOTOR_LEFT,  0.0f);
        DWT_DELAY_US(3500);
        sendCurrentCommand(MOTOR_RIGHT, 0.0f);
        DWT_DELAY_US(3500);

        /* Nastavi izhodišče za pozicijo koles */
        pr_pos_l = g_pos_left;
        pr_pos_r = g_pos_right;
        wheel_pos = 0.0f;
    }

    /* ── 1. PREBERI SENZORJE ─────────────────────────────────── */

    /* bno_roll je v stopinjah; odštej auto-kalibrirani offset */
    float theta = (bno_roll - theta_offset) * (3.14159f / 180.0f);

    float theta_dot = (theta - theta_prev) / TS;
    theta_prev = theta;

    float vel_l = (float)g_rpm_left  * (2.0f * 3.14159f / 60.0f) * WHEEL_RADIUS;
    float vel_r = (float)g_rpm_right * (2.0f * 3.14159f / 60.0f) * WHEEL_RADIUS;
    float wheel_vel = 0.5f * (vel_l - vel_r);

    /* ── Pozicija koles z wrap-around korekcijo ─────────────────
     *  Obseg: 0–32767 (= 0°–360°). Ko kolo preseže 32767, skoči na 0.
     *  Korekcija: delta > +16383 → preskok naprej  → odštej 32767
     *             delta < -16383 → preskok nazaj   → prištej 32767
     *  Deluje za neomejeno število obratov v obe smeri.
     */
    int32_t delta_l = (int32_t)g_pos_left  - (int32_t)pr_pos_l;
    int32_t delta_r = (int32_t)g_pos_right - (int32_t)pr_pos_r;

    if (delta_l >  16383) delta_l -= 32767;
    if (delta_l < -16383) delta_l += 32767;
    if (delta_r >  16383) delta_r -= 32767;
    if (delta_r < -16383) delta_r += 32767;

    pr_pos_l = g_pos_left;
    pr_pos_r = g_pos_right;

    /* Desni motor obrnjen → minus; povprečje obeh koles → *0.5 */
    wheel_pos += (float)(delta_l - delta_r) * 0.5f * (0.314159f / 32767.0f);

    /* Anti-windup: omeji pozicijo da se integral ne nabere ko robot pade */
    if (wheel_pos >  100.0f) wheel_pos =  100.0f;
    if (wheel_pos < -100.0f) wheel_pos = -100.0f;

    /* ── 2. VARNOSTNA ZAUSTAVITEV ────────────────────────────── */

    if (bno_roll - theta_offset >  THETA_MAX_DEG ||
        bno_roll - theta_offset < -THETA_MAX_DEG)
    {
        reg_active = 0.0f;
    }

    /* ── 3. NAPAKA STANJ ─────────────────────────────────────── */

    float e_pos   = wheel_pos - POS_DESIRE;
    float e_vel   = wheel_vel;
    float e_theta = theta;
    float e_tdot  = theta_dot;

    /* ── 4. LQR REGULACIJSKO PRAVILO ─────────────────────────── */

    float iq = -(  K_WHEEL_POS * e_pos
                 - K_WHEEL_VEL * e_vel
                 + K_THETA     * e_theta
                 + K_THETA_DOT * e_tdot ) * 0.5f * 0.75f * reg_active;

    g_iq = iq;

    /* ── 5. OMEJI TOK ────────────────────────────────────────── */

    if (iq >  IQ_MAX) iq =  IQ_MAX;
    if (iq < -IQ_MAX) iq = -IQ_MAX;

    /* ── 6. POŠLJI NA MOTORJE ────────────────────────────────── */

    sendCurrentCommand(MOTOR_LEFT,  MOTOR_LEFT_SIGN  * iq);
    DWT_DELAY_US(3500);
    sendCurrentCommand(MOTOR_RIGHT, MOTOR_RIGHT_SIGN * iq);
}
