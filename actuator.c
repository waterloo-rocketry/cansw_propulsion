#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "canlib/canlib.h"
#include "IOExpanderDriver.h"
#include "actuator.h"

// FIXME

static uint8_t actuator_states;

void actuator_init(){
    actuator_states=get_output();
}

void actuator_set(enum ACTUATOR_STATE state, uint8_t pin_num) {
    if (state == ACTUATOR_OFF) {
          actuator_states &= ~(1<<pin_num);
    } else if (state == ACTUATOR_ON) {
          actuator_states |= (1<<pin_num);
    }
     set_output(actuator_states);
}
