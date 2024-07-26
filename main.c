
#include <stdio.h>
#include <stdlib.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"

#include "mcc_generated_files/system/system.h"

#include "IOExpanderDriver.h"
#include "actuator.h"
#include "error_checks.h"
#include "i2c.h"
#include "sensor_general.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000 

// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 // 2 Hz

#define MAX_CAN_IDLE_TIME_MS 20000 

#define SAFE_STATE_ENABLED 1
adcc_channel_t current_sense_5v = channel_ANA0; 
adcc_channel_t current_sense_12v = channel_ANA1; 
adcc_channel_t batt_vol_sense = channel_ANC2;

// ADD more actuator ID's if propulsion wants more stuff
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)

#define SAFE_STATE_FILL ACTUATOR_OFF
#define SAFE_STATE_INJ ACTUATOR_OFF

#define FILL_DUMP_PIN 2
#define INJECTOR_PIN 0 

#define PRES_PNEUMATICS_TIME_DIFF_ms 250 // 4 Hz
#define PRES_FUEL_TIME_DIFF_ms 62 // 16 Hz
#define PRES_CC_TIME_DIFF_ms 62 // 16 Hz
#define HALLSENSE_FUEL_TIME_DIFF_ms 250 // 4 Hz
#define HALLSENSE_OX_TIME_DIFF_ms 250 // 4 Hz

adcc_channel_t pres_fuel = channel_ANB1;
adcc_channel_t pres_pneumatics = channel_ANB2;
adcc_channel_t pres_cc = channel_ANB0;
adcc_channel_t hallsense_fuel = channel_ANB4;
adcc_channel_t hallsense_ox = channel_ANB5;

double fuel_pres_low_pass = 0;
double cc_pres_low_pass = 0;

uint8_t fuel_pres_count = 0;
uint8_t cc_pres_count = 0;

#define HALLSENSE_FUEL_THRESHOLD 1400
#define HALLSENSE_OX_THRESHOLD 1400

#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
#define SAFE_STATE_VENT ACTUATOR_OFF
#define VENT_VALVE_PIN 0 
#define VENT_TEMP_TIME_DIFF_ms 250 // 4 Hz
#define PRES_OX_TIME_DIFF_ms 62 // 16 Hz

adcc_channel_t pres_ox = channel_ANB0;
adcc_channel_t temp_vent = channel_ANB1;

double ox_pres_low_pass = 0;

uint8_t ox_pres_count = 0;

#else
#error "INVALID_BOARD_UNIQUE_ID"

#endif

static void can_msg_handler(const can_msg_t *msg);
static void send_status_ok(void);

// Follows ACTUATOR_STATE in message_types.h
// SHOULD ONLY BE MODIFIED IN ISR
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
volatile enum ACTUATOR_STATE requested_actuator_state_fill = SAFE_STATE_FILL;
volatile enum ACTUATOR_STATE requested_actuator_state_inj = SAFE_STATE_INJ;
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
volatile enum ACTUATOR_STATE requested_actuator_state_vent = SAFE_STATE_VENT;

#endif

volatile bool seen_can_message = false;
volatile bool seen_can_command = false;

// memory pool for the CAN tx buffer
uint8_t tx_pool[200];

#define IOEXP_I2C_ADDR 0x41

int main(int argc, char **argv) {
    // MCC generated initializer
    SYSTEM_Initialize();

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
    // pca_init(); FIXME find way to init pca without having staes default to high 
    actuator_init();

    uint32_t last_message_millis = 0; // last time we saw a can message 
    // loop timers
    uint32_t last_status_millis = millis(); // unused?
    uint32_t last_pres_low_millis = millis();

    uint32_t last_millis = millis();
    uint32_t last_command_millis = millis();

#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
    uint32_t last_pres_fuel_millis = millis();
    uint32_t last_pres_pneumatics_millis = millis();
    uint32_t last_pres_cc_millis = millis();
    uint32_t last_hallsense_fuel_millis = millis();
    uint32_t last_hallsense_fill_millis = millis();
    uint32_t last_hallsense_ox_millis = millis();
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
    uint32_t last_pres_ox_millis = millis();
    uint32_t last_vent_temp_millis = millis();
#endif

    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }
        if (seen_can_command) {
            seen_can_command = false;
            last_command_millis = millis();
        }

#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
        if (((millis() - last_message_millis) > MAX_BUS_DEAD_TIME_ms) &&
            (requested_actuator_state_inj == SAFE_STATE_INJ)) {
            // Only reset if safe state is enabled (aka this isn't injector valve)
            // OR this is injector valve and the currently requested state is the safe
            // state (closed)

            // We've got too long without seeing a valid CAN message (including one of ours)
            RESET();
        }
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
        if ((millis() - last_message_millis) > MAX_BUS_DEAD_TIME_ms) {
            RESET();
        }
#endif
        if (millis() - last_millis > STATUS_TIME_DIFF_ms) {

            // check for general board status
            bool status_ok = true;
            status_ok &= check_battery_voltage_error(batt_vol_sense);

            status_ok &= check_5v_current_error(current_sense_5v); 
            status_ok &= check_12v_current_error(current_sense_12v);

            // if there was an issue, a message would already have been sent out
            if (status_ok) {
                send_status_ok();
            }
            
            // Set safe state if:
            // 1. We haven't heard CAN traffic in a while
            // 2. We're low on battery voltage
            // "thread safe" because main loop should never write to requested_actuator_state
            if (SAFE_STATE_ENABLED && (((millis() - last_command_millis) > MAX_CAN_IDLE_TIME_MS) ||
                                       is_batt_voltage_critical())) {
                
                // Red LED flashes during safe state.
                LED_heartbeat_R();
                LED_OFF_B();

#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
                actuator_set(SAFE_STATE_FILL, FILL_DUMP_PIN);
                
                // Injector does not have electrical safe state, just keep state
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                actuator_set(SAFE_STATE_VENT, VENT_VALVE_PIN);
#endif
                           
            } else {
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
                actuator_set(requested_actuator_state_inj, INJECTOR_PIN);
                set_actuator_LED(requested_actuator_state_inj, ACTUATOR_INJECTOR_VALVE);
                
                actuator_set(requested_actuator_state_fill, FILL_DUMP_PIN);
                set_actuator_LED(requested_actuator_state_fill, ACTUATOR_FILL_DUMP_VALVE);
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                actuator_set(requested_actuator_state_vent, VENT_VALVE_PIN);
                set_actuator_LED(requested_actuator_state_vent, ACTUATOR_VENT_VALVE);
                LED_OFF_R();
#endif
            } 
            
            // send ACTUATOR_STATUS can messages for non-hall sense valves
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
            can_msg_t stat_msg2;
            build_actuator_stat_msg(millis(), ACTUATOR_FILL_DUMP_VALVE, ACTUATOR_UNK, requested_actuator_state_fill, &stat_msg2);
            txb_enqueue(&stat_msg2);
 #elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
            can_msg_t stat_msg;
            build_actuator_stat_msg(millis(), ACTUATOR_VENT_VALVE, ACTUATOR_UNK, requested_actuator_state_vent, &stat_msg);
            txb_enqueue(&stat_msg);
#endif

            // Visual heartbeat indicator 
            LED_heartbeat_G();

            // update our loop counter
            last_millis = millis();
        }

        
#if PRES_PNEUMATICS_TIME_DIFF_ms
        if (millis() - last_pres_pneumatics_millis > PRES_PNEUMATICS_TIME_DIFF_ms) {
            last_pres_pneumatics_millis = millis();

            uint16_t pressure_pneumatics_psi = get_pressure_pneumatic_psi(pres_pneumatics);

            can_msg_t sensor_msg;
            build_analog_data_msg(
            millis(), SENSOR_PRESSURE_PNEUMATICS, pressure_pneumatics_psi, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif

#if PRES_FUEL_TIME_DIFF_ms
        if (millis() - last_pres_fuel_millis > PRES_FUEL_TIME_DIFF_ms) {
            last_pres_fuel_millis = millis();
            fuel_pres_low_pass = update_pressure_psi_low_pass(pres_fuel, fuel_pres_low_pass);
            if((fuel_pres_count & 0x3) == 0){
                can_msg_t sensor_msg;
                build_analog_data_msg(millis(), SENSOR_PRESSURE_FUEL, fuel_pres_low_pass, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            fuel_pres_count++;
        }
#endif

#if PRES_CC_TIME_DIFF_ms
        if (millis() - last_pres_cc_millis > PRES_CC_TIME_DIFF_ms) {
            last_pres_cc_millis = millis();
            cc_pres_low_pass = update_pressure_psi_low_pass(pres_cc, cc_pres_low_pass);
            if((cc_pres_count & 0x3) == 0){
                can_msg_t sensor_msg;
                build_analog_data_msg(millis(), SENSOR_PRESSURE_CC, cc_pres_low_pass, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            cc_pres_count++;
        }
#endif

#if HALLSENSE_FUEL_TIME_DIFF_ms
        if (millis() - last_hallsense_fuel_millis > HALLSENSE_FUEL_TIME_DIFF_ms) {
            last_hallsense_fuel_millis = millis();
            uint16_t hallsense_fuel_flux = get_hall_sensor_reading(hallsense_fuel);
            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), SENSOR_HALL_FUEL_INJ, hallsense_fuel_flux, &sensor_msg);
            txb_enqueue(&sensor_msg);
            
            can_msg_t stat_msg3;
            build_actuator_stat_msg(millis(), ACTUATOR_FUEL_INJECTOR, ((hallsense_fuel_flux > HALLSENSE_FUEL_THRESHOLD) ? ACTUATOR_ON : ACTUATOR_OFF), requested_actuator_state_inj, &stat_msg3);
            txb_enqueue(&stat_msg3);
        }
#endif

#if HALLSENSE_OX_TIME_DIFF_ms
        if (millis() - last_hallsense_ox_millis > HALLSENSE_OX_TIME_DIFF_ms) {
            last_hallsense_ox_millis = millis();
            uint16_t hallsense_ox_flux = get_hall_sensor_reading(hallsense_ox);
            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), SENSOR_HALL_OX_INJ, hallsense_ox_flux, &sensor_msg);
            txb_enqueue(&sensor_msg);
            
            can_msg_t stat_msg1;
            build_actuator_stat_msg(millis(), ACTUATOR_OX_INJECTOR, ((hallsense_ox_flux > HALLSENSE_OX_THRESHOLD) ? ACTUATOR_ON : ACTUATOR_OFF), requested_actuator_state_inj, &stat_msg1);
            txb_enqueue(&stat_msg1);
        }
#endif

#if VENT_TEMP_TIME_DIFF_ms
        if (millis() - last_vent_temp_millis > VENT_TEMP_TIME_DIFF_ms) {
            last_vent_temp_millis = millis();

            uint16_t temperature_c = get_temperature_c(temp_vent);

            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), SENSOR_VENT_TEMP, temperature_c, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif

#if PRES_OX_TIME_DIFF_ms
        if (millis() - last_pres_ox_millis > PRES_OX_TIME_DIFF_ms) {
            last_pres_ox_millis = millis();
            ox_pres_low_pass = update_pressure_psi_low_pass(pres_ox, ox_pres_low_pass);
            if((ox_pres_count & 0x3) == 0){
                can_msg_t sensor_msg;
                build_analog_data_msg(millis(), SENSOR_PRESSURE_OX, ox_pres_low_pass, &sensor_msg);
                txb_enqueue(&sensor_msg);
            }
            ox_pres_count++;
        }
#endif
        // send any queued CAN messages
        txb_heartbeat();
    }

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

    // make able to handle multiple actuator
    switch (msg_type) {
        // Make it handle multiple actuator
        case MSG_ACTUATOR_CMD:
            // see message_types.h for message format
            // vent position will be updated synchronously

#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
            if (get_actuator_id(msg) == ACTUATOR_INJECTOR_VALVE) {
                requested_actuator_state_inj = get_req_actuator_state(msg);
                seen_can_command = true;  
            } else if (get_actuator_id(msg) == ACTUATOR_FILL_DUMP_VALVE) {
                requested_actuator_state_fill = get_req_actuator_state(msg);
                seen_can_command = true;
            }
#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
            if (get_actuator_id(msg) == ACTUATOR_VENT_VALVE) {
                requested_actuator_state_vent = get_req_actuator_state(msg);
                seen_can_command = true;
            }
#endif

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
