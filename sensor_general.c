#include <stdbool.h>
#include <xc.h>

#include "mcc_generated_files/system/system.h"
#include <math.h>

#include "sensor_general.h"

#define PT_OFFSET 0

const float VREF = 3.3;

void LED_init(void) {
    TRISA4 = 0; // set B4, B3, B2 as output
    TRISA3 = 0;
    TRISA2 = 0;
    LATA4 = 1; // turn the leds off
    LATA3 = 1;
    LATA2 = 1;
}

// Green LED
void LED_heartbeat_G(void) {
    static bool led_on = false;
    if (led_on) {
        LED_OFF_G();
        led_on = false;
    } else {
        LED_ON_G();
        led_on = true;
    }
}

// Blue LED
void LED_heartbeat_B(void) {
    static bool led_on = false;
    if (led_on) {
        LED_OFF_B();
        led_on = false;
    } else {
        LED_ON_B();
        led_on = true;
    }
}

// Red LED
void LED_heartbeat_R(void) {
    static bool led_on = false;
    if (led_on) {
        LED_OFF_R();
        led_on = false;
    } else {
        LED_ON_R();
        led_on = true;
    }
}

// 4-20mA pressure transducer
uint32_t get_pressure_4_20_psi(enum adcc_channel_t) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(adcc_channel_t);

    float v = (voltage_raw + 0.5f) / 4096.0f * VREF;

    const double r = 100;
    const double pressure_range = 3000;

    double current = v / r;

    int32_t pressure_psi = (int32_t)(((current - 0.004) / (0.02 - 0.004)) * pressure_range);

    return (uint32_t)pressure_psi + PT_OFFSET;
}

uint32_t get_pressure_pneumatic_psi(enum adcc_channel_t) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(adcc_channel_t);

    float v = ((voltage_raw + 0.5f) / 4096.0f * VREF) * 2; // 10kohm and 10kohm resistor divider

    // for PSE530-R06, see "Analog Output" graph here: https://www.smcpneumatics.com/pdfs/PSE.pdf
    //  analog output[V] = ((5[V]-0.6[V])/(1[MPa] - (-0.1[MPa]))*pressure[MPa], aka y = 4x + 1
    //  calibrated based on the PSE540 and DAQ.
    int16_t pressure_psi = (int16_t)(((v - 1) / 4) * 165.34 + 5.2);

    // int32_t pressure_psi = (int32_t) (v * 39.2f*3.0f - 39.2f);

    return (uint16_t)pressure_psi;
}

// Low-pass filter for 4-20mA pressure transducer
#define SAMPLE_FREQ (1000.0 / PRES_TIME_DIFF_ms)
#define LOW_PASS_ALPHA(TR) ((SAMPLE_FREQ * TR / 5.0) / (1 + SAMPLE_FREQ * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 10.0 // seconds
double alpha_low = LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME);
double low_pass_pressure_psi = 0;

uint16_t update_pressure_psi_low_pass(void) {

    int16_t pressure_psi = get_pressure_4_20_psi();

    low_pass_pressure_psi = alpha_low * low_pass_pressure_psi + (1.0 - alpha_low) * pressure_psi;
    return (uint16_t)low_pass_pressure_psi;
}

// 10kR thermistor
uint16_t get_temperature_c(enum adcc_channel_t) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(adcc_channel_t);
    const float rdiv = 10000.0; // 10kohm divider resistor

    // beta, r0, t0 from
    // https://media.digikey.com/pdf/Data%20Sheets/Thermometrics%20Global%20Business%20PDFs/TG_Series.pdf
    const float beta = 3434.0;
    const float r0 = 10000.0;
    const float t0 = 298.15; // 25 C in Kelvin

    float v =
        (voltage_raw + 0.5f) / 4096.0f * VREF; // (raw + continuity correction) / 12bit adc * vref
    float r = ((VREF * rdiv) / v) - rdiv;

    float invk = 1 / t0 + 1 / beta * log(r / r0);

    return (uint16_t)(1 / invk - 273);
}
