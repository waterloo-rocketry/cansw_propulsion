#include <stdio.h>
#include <stdlib.h>

#include "canlib/canlib.h"
#include "canlib/message_types.h"


#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
#include "i2c.h"
#include "sensor_general.h"
#include "actuator.h"
#include "IOExpanderDriver.h"

#include <xc.h>

#define MAX_BUS_DEAD_TIME_ms 1000

// Set any of these to zero to disable
#define STATUS_TIME_DIFF_ms 500 // 2 Hz


#define MAX_LOOP_TIME_DIFF_ms 20
#define MAX_CAN_IDLE_TIME_MS 1000

#define SAFE_STATE_ENABLED 1

//ADD more actuator ID's if propulsion wants more stuff
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
#define SAFE_STATE_FILL 1
#define SAFE_STATE_INJ 1
#define FILL_DUMP_PIN 1
#define INJECTOR_PIN 2
#define PRES_PNEUMATICS_TIME_DIFF_ms 500 // 2 Hz
#define PRES_FUEL_TIME_DIFF_ms 500
#define PRES_CC_TIME_DIFF_ms 500
#define adcc_channel_t

#elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
#define SAFE_STATE_VENT 1
#define VENT_VALVE_PIN 1
#define VENT_TEMP_TIME_DIFF_ms 0 

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
    actuator_init();

    uint32_t last_message_millis = 0; // last time we saw a can message
    // loop timers
    uint32_t last_status_millis = millis();
    uint32_t last_pres_low_millis = millis();

    uint32_t last_millis = millis();
    uint32_t last_command_millis = millis();
#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
    uint32_t last_pres_fuel_millis = millis();
    uint32_t last_pres_pneumatics_millis = millis();
    uint32_t last_pres_cc_millis = millis();
    adcc_channel_t pres_fuel;
    adcc_channel_t pres_pneumatics;
    adcc_channel_t pres_cc;
#endif

#if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
    uint32_t last_pres_ox_millis=millis();
    uint32_t last_vent_temp_millis = millis();
    adcc_channel_t pres_ox;
    adcc_channel_t temp_vent
#endif
    
    // Test the IO Expander
    pca_init();

    bool blue_led_on = false;   // visual heartbeat
    while (1) {
        CLRWDT(); // feed the watchdog, which is set for 256ms

        if (seen_can_message) {
            seen_can_message = false;
            last_message_millis = millis();
        }
        
    #if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
        if (millis() - last_message_millis > MAX_BUS_DEAD_TIME_ms &&
            (SAFE_STATE_ENABLED || requested_actuator_state_inj == SAFE_STATE)) {
            // Only reset if safe state is enabled (aka this isn't injector valve)
            // OR this is injector valve and the currently requested state is the safe
            // state (closed)
            
            // We've got too long without seeing a valid CAN message (including one of ours)
            RESET();
        }
    #endif
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

            // if there was an issue, a message would already have been sent out
            if (status_ok) { send_status_ok(); }

            // Set safe state if:
            // 1. We haven't heard CAN traffic in a while
            // 2. We're low on battery voltage
            // "thread safe" because main loop should never write to requested_actuator_state
            if (SAFE_STATE_ENABLED && (
                    (millis() - last_command_millis > MAX_CAN_IDLE_TIME_MS)
                    || is_batt_voltage_critical())) {
                //actuator_send_status(SAFE_STATE);
                #if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ) 
                    actuator_set(SAFE_STATE_FILL, FILL_DUMP_PIN);  
                    actuator_set(SAFE_STATE_INJ, INJECTOR_PIN); 
                #elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                    actuator_set(SAFE_STATE_VENT, VENT_VALVE_PIN);
                #endif
                
            } else {
                #if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
                    actuator_set(requested_actuator_state_inj, INJECTOR_PIN);
                    actuator_set(requested_actuator_state_fill, FILL_DUMP_PIN);
                #elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                    actuator_set(requested_actuator_state_vent, VENT_VALVE_PIN);
                #endif
            }

            // visual heartbeat indicator
            if (blue_led_on) {
                //BLUE_LED_OFF();
                blue_led_on = false;
            } else {
                //BLUE_LED_ON();
                blue_led_on = true;
            }

            // update our loop counter
            last_millis = millis();
        }
        
#if PRES_TIME_DIFF_ms
        if (millis() - last_pres_low_millis > PRES_TIME_DIFF_ms) {
            last_pres_low_millis = millis();
            update_pressure_psi_low_pass(channel_ANA0);
        }
#endif
        
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
        if(millis()-last_pres_fuel_millis > PRES_FUEL_TIME_DIFF_ms)
        {
            last_pres_fuel_millis = millis();
            uint16_t pressure_fuel_psi = update_pressure_psi_low_pass(pres_fuel);
            can_msg_t sensor_msg;
            build_analog_data_msg(
                millis(), SENSOR_PRESSURE_FUEL, pressure_fuel_psi, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif

#if PRES_CC_TIME_DIFF_ms 
        if(millis()-last_pres_cc_millis > PRES_CC_TIME_DIFF_ms)
        {
            last_pres_cc_millis = millis();
            uint16_t pressure_cc_psi = update_pressure_psi_low_pass(pres_cc);
            can_msg_t sensor_msg;
            build_analog_data_msg(
                millis(), SENSOR_PRESSURE_CC, pressure_cc_psi, &sensor_msg);
            txb_enqueue(&sensor_msg);  
        }
#endif
        
#if VENT_TEMP_TIME_DIFF_ms
        if (millis() - last_temp_millis > VENT_TEMP_TIME_DIFF_ms) {
            last_vent_temp_millis = millis();

            uint16_t temperature_c = get_temperature_c(temp_vent);

            can_msg_t sensor_msg;
            build_analog_data_msg(millis(), SENSOR_VENT_TEMP, temperature_c, &sensor_msg);
            txb_enqueue(&sensor_msg);
        }
#endif
        
#if PRES_OX_TIME_DIFF_ms 
        if(millis()-last_pres_cc_millis > PRES_OX_TIME_DIFF_ms)
        {
            last_pres_ox_millis = millis();
            uint16_t pressure_ox_psi = update_pressure_psi_low_pass(pres_ox);
            can_msg_t sensor_msg;
            build_analog_data_msg(
                millis(), SENSOR_PRESSURE_OX, pressure_ox_psi, &sensor_msg);
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

#define ACTUATOR_ID 2

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

    //make able to handle multiple actuator
    switch (msg_type) {
        case MSG_GENERAL_CMD:
            cmd_type = get_general_cmd_type(msg);
            if (cmd_type == BUS_DOWN_WARNING) {
            #if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
                requested_actuator_state_fill = SAFE_STATE_FILL;
                requested_actuator_state_inj = SAFE_STATE_INJ;
            #elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                requested_actuator_state_vent=SAFE_STATE_VENT;
            #endif
            }
            break;
            
        //Make it handle multiple actuator    
        case MSG_ACTUATOR_CMD:
            // see message_types.h for message format
            if (get_actuator_id(msg) == ACTUATOR_ID) {
                // vent position will be updated synchronously
                
                #if (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_INJ)
                    if(get_actuator_id(msg) == ACTUATOR_INJECTOR_VALVE)
                        requested_actuator_state_inj=get_req_actuator_state(msg);
                    else if(get_actuator_id(msg) == ACTUATOR_FILL_DUMP_VALVE)
                        requested_actuator_state_fill=get_req_actuator_state(msg);
                #elif (BOARD_UNIQUE_ID == BOARD_ID_PROPULSION_VENT)
                        if(get_actuator_id(msg) == ACTUATOR_VENT_VALVE)
                        requested_actuator_state_vent = get_req_actuator_state(msg);
                #endif
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
