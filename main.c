#include <stdio.h>
#include <stdlib.h>

#include "canlib/canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
#include "sensor_general.h"
#include "i2c.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000

// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 // 2 Hz
#define PRES_OX_CC_TIME_DIFF_ms 500 // 2 Hz
#define PRES_PNEUMATICS_TIME_DIFF_ms 500 // 2 Hz
#define TEMP_TIME_DIFF_ms 0 // Disabled

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);

volatile bool seen_can_message = false;

// memory pool for the CAN tx buffer
uint8_t tx_pool[200];

int main(int argc, char **argv) {
    // MCC generated initializer
    SYSTEM_Initialize();

    ADCC_DisableContinuousConversion();
    LED_init();

    // init our millisecond function
    timer0_init();

    // Enable global interrupts
    INTCON0bits.GIE = 1;

    // Set up CAN TX
    TRISC1 = 0;
    RC1PPS = 0x33;

    // Set up CAN RX
    TRISC0 = 1;
    ANSELC0 = 0;
    CANRXPPS = 0x10;

    // set up CAN module
    can_timing_t can_setup;
    can_generate_timing_params(_XTAL_FREQ, &can_setup);
    can_init(&can_setup, can_msg_handler);

    // set up CAN tx buffer
    txb_init(tx_pool, sizeof(tx_pool), can_send, can_send_rdy);

    i2c_init(0);

    uint32_t last_message_millis = 0; // last time we saw a can message
    // loop timers
    uint32_t last_status_millis = millis();
    uint32_t last_pres_ox_cc_millis = millis();
    uint32_t last_pres_pneumatics_millis = millis();
    uint32_t last_pres_low_millis = millis();
    uint32_t last_temp_millis = millis();
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }

        if (millis() - last_message_millis > MAX_BUS_DEAD_TIME_ms) {
            // We've got too long without seeing a valid CAN message (including one of ours)
            RESET();
        }

        if (millis() - last_status_millis > STATUS_TIME_DIFF_ms) {
            last_status_millis = millis();

            bool status_ok = true;
            status_ok &= check_bus_current_error();
            if (status_ok) {
                send_status_ok();
            }

            LED_heartbeat_G();
        }

#if PRES_OX_CC_TIME_DIFF_ms
        if (millis() - last_pres_ox_cc_millis > PRES_OX_CC_TIME_DIFF_ms) {
            last_pres_ox_cc_millis = millis();

            // No low-pass filter
            // uint16_t pressure_4_20_psi = get_pressure_4_20_psi();

            // With low-pass filter, uncomment if low-pass filtering required:
            uint16_t pressure_4_20_psi = update_pressure_psi_low_pass();

            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), PT_SENSOR_ID, pressure_4_20_psi, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif
#if PRES_TIME_DIFF_ms
        if (millis() - last_pres_low_millis > PRES_TIME_DIFF_ms) {
            last_pres_low_millis = millis();
            update_pressure_psi_low_pass();
        }
#endif
#if PRES_PNEUMATICS_TIME_DIFF_ms
        if (millis() - last_pres_pneumatics_millis > PRES_PNEUMATICS_TIME_DIFF_ms) {
            last_pres_pneumatics_millis = millis();

            uint16_t pressure_pneumatics_psi = get_pressure_pneumatic_psi();

            can_msg_t sensor_msg;
            build_analog_data_msg(
                millis(), SENSOR_PRESSURE_PNEUMATICS, pressure_pneumatics_psi, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif
#if TEMP_TIME_DIFF_ms
        if (millis() - last_temp_millis > TEMP_TIME_DIFF_ms) {
            last_temp_millis = millis();

            uint16_t temperature_c = get_temperature_c();

            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), SENSOR_VENT_TEMP, temperature_c, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif

        // send any queued CAN messages
        txb_heartbeat();
    }

    send_status_ok();

    // unreachable
    return (EXIT_SUCCESS);
}

static void __interrupt() interrupt_handler() {
    if (PIR5) {
        can_handle_interrupt();
    }

    // Timer0 has overflowed - update millis() function
    // This happens approximately every 500us
    if (PIE3bits.TMR0IE == 1 && PIR3bits.TMR0IF == 1) {
        timer0_handle_interrupt();
        PIR3bits.TMR0IF = 0;
    }
}

static void can_msg_handler(const can_msg_t *msg) {
    seen_can_message = true;
    uint16_t msg_type = get_message_type(msg);
    int dest_id = -1;
    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    switch (msg_type) {
        case MSG_GENERAL_CMD:
            // nothing right now
            break;

        case MSG_LEDS_ON:
            LED_ON_G();
            LED_ON_B();
            LED_ON_R();
            break;

        case MSG_LEDS_OFF:
            LED_OFF_G();
            LED_OFF_B();
            LED_OFF_R();
            break;

        case MSG_RESET_CMD:
            dest_id = get_reset_board_id(msg);
            if (dest_id == BOARD_UNIQUE_ID || dest_id == 0) {
                RESET();
            }
            break;
        // all the other ones - do nothing
        default:
            break;
    }
}

// Send a CAN message with nominal status
static void send_status_ok(void) {
    can_msg_t board_stat_msg;
    build_board_stat_msg(millis(), E_NOMINAL, NULL, 0, &board_stat_msg);

    txb_enqueue(&board_stat_msg);
}
