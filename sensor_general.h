#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#define PRES_TIME_DIFF_ms 16 // 64 Hz

#include "mcc_generated_files/adc/adcc.h"
#include <stdint.h>
// Contains miscellaneous sensor board-specific code

#define LED_ON_G() (LATA2 = 0)
#define LED_OFF_G() (LATA2 = 1)
#define LED_ON_B() (LATA3 = 0)
#define LED_OFF_B() (LATA3 = 1)
#define LED_ON_R() (LATA4 = 0)
#define LED_OFF_R() (LATA4 = 1)

// Initialize LEDS
void LED_init(void);

// Blink the LEDs on or off each time it's called. Can be used to provide a visual
// heartbeat for the board.
void LED_heartbeat_G(void); // Green LED
void LED_heartbeat_B(void); // Blue LED
void LED_heartbeat_R(void); // Red LED

// Read pressure sensor ADC and convert to PSI. Replace all negative values with
// zero since canlib and RLCS don't like it.
uint32_t get_pressure_4_20_psi(adcc_channel_t adc_channel);
uint32_t get_pressure_pneumatic_psi(adcc_channel_t adc_channel);
uint16_t update_pressure_psi_low_pass(adcc_channel_t adc_channel, double *low_pass_pressure_psi);
uint16_t get_temperature_c(adcc_channel_t adc_channel);
uint16_t get_hall_sensor_reading(adcc_channel_t adc_channel);
#endif /* SENSOR_GEN_H */
