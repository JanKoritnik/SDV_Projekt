#ifndef REGULATOR_H
#define REGULATOR_H

#include <math.h>
#include <stdint.h>

// ═══════════════════════════════════════════════
// LQR OJAČITVE
// Izračunano iz: Q=diag([100,30,200,10]), R=0.01
// Model: HIGH lega robota, Ts=20ms
// ═══════════════════════════════════════════════
#define K_WHEEL_POS    90.0f
#define K_WHEEL_VEL    99.00f
#define K_THETA        260.0f
#define K_THETA_DOT    48.0f

// ═══════════════════════════════════════════════
// NASTAVITVE — TUKAJ NASTAVIŠ
// ═══════════════════════════════════════════════
#define TS              0.020f   // čas tipanja [s] — NE SPREMINJAJ
#define WHEEL_RADIUS    0.0501f   // polmer kolesa [m]
#define IQ_MAX          1.0f     // max tok [A] — povečuj postopoma: 1→1.5→2→3
#define POS_DESIRE      0.0f     // želena pozicija [m]

// Kalibracijski kot — auto-kalibriran med startup delay
// Ročno nastavi samo če auto-kalibracija ne deluje
#define THETA_OFFSET_DEG  0.0f

// ═══════════════════════════════════════════════
// VARNOSTNA MEJA
// ═══════════════════════════════════════════════
#define THETA_MAX_DEG   30.0f

// RPM iz DDSM115 odgovorov (posodablja uart_app.c ob vsakem RS485 prejemu)
extern volatile int16_t g_rpm_left;
extern volatile int16_t g_rpm_right;
extern volatile float g_iq;

void controller_init(void);
void controller_step(void);

#endif /* REGULATOR_H */
