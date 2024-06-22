#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#include "IOExpanderDriver.h"
#include "actuator.h"
#include "canlib/canlib.h"

// FIXME

static uint8_t actuator_states;
#define PCA_ADDRESS 0x41
#define POLARITY_REG 0x02
void actuator_init(uint8_t polarity) {
    i2c_write_reg8(PCA_ADDRESS, POLARITY_REG, polarity);
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
