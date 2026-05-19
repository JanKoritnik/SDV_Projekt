#ifndef REGULATOR_H
#define REGULATOR_H

#include <math.h>

// ═══════════════════════════════════════════════
// LQR OJAČITVE
// Izračunano iz: Q=diag([100,30,200,10]), R=0.01
// Model: HIGH lega robota, Ts=20ms
// ═══════════════════════════════════════════════
#define K_WHEEL_POS    65.036f   // ojačitev pozicije koles [Nm/m]
#define K_WHEEL_VEL    69.135f   // ojačitev hitrosti koles [Nm*s/m]
#define K_THETA       326.185f   // ojačitev kota nagiba    [Nm/rad]
#define K_THETA_DOT    51.248f   // ojačitev kotne hitrosti [Nm*s/rad]

// ═══════════════════════════════════════════════
// NASTAVITVE — TUKAJ NASTAVIŠ
// ═══════════════════════════════════════════════
#define TS              0.020f   // čas tipanja [s] — NE SPREMINJAJ
#define WHEEL_RADIUS    0.050f   // polmer kolesa [m]
#define IQ_MAX          1.0f     // max tok na začetku [A] — povečuj postopoma
#define POS_DESIRE      0.0f     // želena pozicija [m], 0 = stoji na mestu

// ═══════════════════════════════════════════════
// VARNOSTNA MEJA
// ═══════════════════════════════════════════════
#define THETA_MAX_DEG   45.0f    // pri večjem kotu izklopi motorje

// RPM iz DDSM115 odgovorov (posodablja uart_app.c ob vsakem RS485 prejemu)
extern volatile float g_rpm_left;
extern volatile float g_rpm_right;

void controller_init(void);
void controller_step(void);

#endif /* REGULATOR_H */
