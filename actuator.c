#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#include "IOExpanderDriver.h"
#include "actuator.h"
#include "canlib/canlib.h"

static uint8_t actuator_states;

void actuator_init(uint8_t polarity) {
    pca_set_polarity(polarity);
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
