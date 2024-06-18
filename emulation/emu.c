#include <stdio.h>

#include "adcc.h"
#include "can_common.h"
#include "xc.h"

uint8_t TRISA2;
uint8_t TRISA3;
uint8_t TRISA4;
uint8_t LATA2;
uint8_t LATA3;
uint8_t LATA4;
uint8_t TRISC1;
uint8_t RC1PPS;
uint8_t TRISC0;
uint8_t ANSELC0;
uint8_t CANRXPPS;
uint8_t PIR5;
struct INTCON0bits_t INTCON0bits;
struct PIE3bits_t PIE3bits;
struct PIR3bits_t PIR3bits;

void i2c_init(uint8_t clkdiv) {
    printf("i2c_init, clkdiv = %u\n", clkdiv);
}

bool i2c_write_reg8(uint8_t address, uint8_t reg, uint8_t val) {
    printf("i2c_write_reg8, addr=%u, reg=%u, val=%x\n", address, reg, val);
    return true;
}

bool i2c_read_reg8(uint8_t address, uint8_t reg, uint8_t *value) {
    *value = 0;
    return true;
}

void timer0_init(void) {
    puts("timer0_init");
}

void timer0_handle_interrupt(void) {
    puts("timer0_handle_interrupt");
}

uint32_t millis_count;
uint32_t millis(void) {
    millis_count++;
    return millis_count % 100;
}

adc_result_t ADCC_GetSingleConversion(adcc_channel_t channel) {
    return 1024;
}

void SYSTEM_Initialize(void) {
    puts("SYSTEM_initialize");
}

void CLRWDT() {
    puts("CLRWDT");
}

void RESET() {
    puts("RESET");
}

void can_init(const can_timing_t *timing, void (*receive_callback)(const can_msg_t *message)) {
    puts("can_init");
}

void can_send(const can_msg_t *message) {
    printf("can_send sid=%03x,", message->sid);
    for (uint8_t i = 0; i < message->data_len; i++) {
        printf(" %02x", message->data[i]);
    }
    putchar('\n');
}

bool can_send_rdy(void) {
    return true;
}

void can_handle_interrupt(void) {
    puts("can_handle_interrupt");
}
