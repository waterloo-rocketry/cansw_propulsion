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
    } else if (state == ACTUATOR_ON) {
        actuator_states |= (1 << pin_num);
    }
    pca_set_output(actuator_states);
}

void set_actuator_LED(enum ACTUATOR_STATE state, enum ACTUATOR_ID actuator) {
    if (actuator == ACTUATOR_INJECTOR_VALVE) {
        (state == ACTUATOR_ON) ? LED_ON_R() : LED_OFF_R();
    } else { // Vent/Fill
        (state == ACTUATOR_ON) ? LED_ON_B() : LED_OFF_B();
    }
}

// TODO update this
enum ACTUATOR_STATE get_actuator_state(uint8_t pin_num) {

    // temp code
    uint8_t state = (actuator_states >> pin_num) & 0x01;

    if (state == 1) {
        return ACTUATOR_ON;
    }

    return ACTUATOR_OFF;
}
