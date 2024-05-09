#include <stdio.h>
#include <stdlib.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"


#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
#include "i2c.h"
#include "sensor_general.h"
#include "actuator.h"
#include "board.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000

// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 // 2 Hz
#define PRES_OX_CC_TIME_DIFF_ms 500 // 2 Hz
#define PRES_PNEUMATICS_TIME_DIFF_ms 500 // 2 Hz
#define TEMP_TIME_DIFF_ms 0 // Disabled

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);

// Follows ACTUATOR_STATE in message_types.h
// SHOULD ONLY BE MODIFIED IN ISR
volatile enum ACTUATOR_STATE requested_actuator_state = SAFE_STATE;

volatile bool seen_can_message = false;
volatile bool seen_can_command = false;

// memory pool for the CAN tx buffer
uint8_t tx_pool[200];

#define IOEXP_I2C_ADDR 0x41

int main(int argc, char **argv) {
    // MCC generated initializer
    SYSTEM_Initialize();
    OSCILLATOR_Initialize();

    FVR_Initialize();
    ADCC_Initialize();
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

    // Set up actuator
    actuator_init();

    uint32_t last_message_millis = 0; // last time we saw a can message
    // loop timers
    uint32_t last_status_millis = millis();
    uint32_t last_pres_ox_cc_millis = millis();
    uint32_t last_pres_pneumatics_millis = millis();
    uint32_t last_pres_low_millis = millis();
    uint32_t last_temp_millis = millis();
    uint32_t last_millis = millis();
    uint32_t last_command_millis = millis();

    // Test the IO Expander
    i2c_write_reg8(IOEXP_I2C_ADDR, 3, 0);
    i2c_write_reg8(IOEXP_I2C_ADDR, 1, 0b101);

    bool blue_led_on = false;   // visual heartbeat
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }

        if (millis() - last_message_millis > MAX_BUS_DEAD_TIME_ms &&
            (SAFE_STATE_ENABLED || requested_actuator_state == SAFE_STATE)) {
            // Only reset if safe state is enabled (aka this isn't injector valve)
            // OR this is injector valve and the currently requested state is the safe
            // state (closed)
            
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
        
        if (millis() - last_millis > MAX_LOOP_TIME_DIFF_ms) {

            // check for general board status
            bool status_ok = true;
            status_ok &= check_battery_voltage_error();
            status_ok &= check_bus_current_error();
            status_ok &= check_actuator_pin_error(requested_actuator_state);

            // if there was an issue, a message would already have been sent out
            if (status_ok) { send_status_ok(); }

            // Set safe state if:
            // 1. We haven't heard CAN traffic in a while
            // 2. We're low on battery voltage
            // "thread safe" because main loop should never write to requested_actuator_state
            if (SAFE_STATE_ENABLED && (
                    (millis() - last_command_millis > MAX_CAN_IDLE_TIME_MS)
                    || is_batt_voltage_critical())) {
                actuator_send_status(SAFE_STATE);
                actuator_set(SAFE_STATE);
            } else {
                actuator_send_status(requested_actuator_state);
                actuator_set(requested_actuator_state);
            }

            // visual heartbeat indicator
            if (blue_led_on) {
                BLUE_LED_OFF();
                blue_led_on = false;
            } else {
                BLUE_LED_ON();
                blue_led_on = true;
            }

            // update our loop counter
            last_millis = millis();
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
    int cmd_type = -1;
    // ignore messages that were sent from this board
    if (get_board_unique_id(msg) == BOARD_UNIQUE_ID) {
        return;
    }

    switch (msg_type) {
        case MSG_GENERAL_CMD:
            cmd_type = get_general_cmd_type(msg);
            if (cmd_type == BUS_DOWN_WARNING) {
                requested_actuator_state = SAFE_STATE;
            }
            break;
            
        case MSG_ACTUATOR_CMD:
            // see message_types.h for message format
            if (get_actuator_id(msg) == ACTUATOR_ID) {
                // vent position will be updated synchronously
                requested_actuator_state = get_req_actuator_state(msg);
                // keep track of heartbeat here
                seen_can_command = true;
            }

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