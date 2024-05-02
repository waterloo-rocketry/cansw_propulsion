#ifndef SENSOR_GEN_H
#define	SENSOR_GEN_H


#define PRES_TIME_DIFF_ms 5

#include <stdint.h>

// Contains miscellaneous sensor board-specific code

#define LED_ON_G() (LATB4 = 0)
#define LED_OFF_G() (LATB4 = 1)
#define LED_ON_B() (LATB3 = 0)
#define LED_OFF_B() (LATB3 = 1)
#define LED_ON_W() (LATB2 = 0)
#define LED_OFF_W() (LATB2 = 1)

// Initialize LEDS
void LED_init(void);

// Blink the LEDs on or off each time it's called. Can be used to provide a visual
// heartbeat for the board.
void LED_heartbeat_G(void); //Green LED
void LED_heartbeat_B(void); //Blue LED
void LED_heartbeat_W(void); //White LED



// Read pressure sensor ADC and convert to PSI. Replace all negative values with
// zero since canlib and RLCS don't like it.
uint32_t get_pressure_4_20_psi(void);
uint32_t get_pressure_pneumatic_psi(void);
uint16_t update_pressure_psi_low_pass(void);
uint16_t get_temperature_c(void);

#endif	/* SENSOR_GEN_H */

