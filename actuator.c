#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#include "IOExpanderDriver.h"
#include "actuator.h"
#include "canlib/canlib.h"
#include "sensor_general.h"

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
}

void set_actuator_LED(enum ACTUATOR_STATE state, enum ACTUATOR_ID actuator) {
    if (actuator == ACTUATOR_INJECTOR_VALVE) {
        (state == ACTUATOR_ON) ? LED_ON_R() : LED_OFF_R(); 
    }
    else { // Vent/Fill
        (state == ACTUATOR_ON) ? LED_ON_B() : LED_OFF_B(); 
    }
}

// TODO update this
enum ACTUATOR_STATE get_actuator_state(uint8_t pin_num) {
    
    // temp code
    uint8_t state = (actuator_states >> pin_num) & 0x01;
    
    if (state == 1)
        return ACTUATOR_ON;
    
    return ACTUATOR_OFF;
    
    
    
    /**
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
     */
}
