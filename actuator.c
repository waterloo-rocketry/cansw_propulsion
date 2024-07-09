#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#include "IOExpanderDriver.h"
#include "actuator.h"
#include "canlib/canlib.h"

uint8_t actuator_states = 0;

void actuator_init() {
    actuator_states = pca_get_output();
}

void actuator_set(enum ACTUATOR_STATE state, uint8_t pin_num) {
    if (state == ACTUATOR_OFF) {
        actuator_states &= ~(1 << pin_num);
    } else if (state == ACTUATOR_ON)  {
        actuator_states |= (1 << pin_num);
    }
    pca_set_output(actuator_states);
    
    // add "enum ACTUATOR_ID id" as parameter 
    // build_actuator_stat_msg(millis(), id, state, enum ACTUATOR_STATE req_actuator_state, can_msg_t *output);
}


enum ACTUATOR_STATE get_actuator_state(void) {
#if !HAS_LIMS
    return ACTUATOR_UNK;
#else
// define in board.h
#ifdef INJECTOR
    adc_result_t hall_raw = ADCC_GetSingleConversion(channel_HALL);
    
    if (hall_raw > HALL_ERR_THRESHOLD) { return ACTUATOR_ILLEGAL; }
    // HIGH_STATE is defined in board.h as the ACTUATOR_STATE that a high
    // hall sensor reading maps to.
    if (hall_raw > HALL_THRESHOLD) { return HIGH_STATE; }
    return 1 - HIGH_STATE; // First two ACTUATOR_STATEs are on and off so this works to invert
#else
    // read limit switch values
    bool actuator_open = PORTBbits.RB4;
    bool actuator_closed = PORTBbits.RB3;
    
    if (actuator_open && !actuator_closed) { return ACTUATOR_OFF; }
    if (!actuator_open && actuator_closed) { return ACTUATOR_ON; }
    if (!actuator_open && !actuator_closed) { return ACTUATOR_UNK; }
    return ACTUATOR_ILLEGAL; // both limit switches at same time
#endif
#endif
}


void actuator_send_status(enum ACTUATOR_STATE req_state) {
// define in board.h
#ifdef INJECTOR
    adc_result_t hall_raw = ADCC_GetSingleConversion(channel_HALL);
    
    // Send a CAN message with the raw hall value
    can_msg_t hall_msg;
    build_analog_data_msg(millis(), SENSOR_MAG_1, hall_raw, &hall_msg);
    txb_enqueue(&hall_msg);
#endif
    enum ACTUATOR_STATE curr_state = get_actuator_state();

    can_msg_t stat_msg;
    build_actuator_stat_msg(millis(), ACTUATOR_ID, curr_state, req_state, &stat_msg);
    txb_enqueue(&stat_msg);
}