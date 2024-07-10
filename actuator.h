#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "canlib/message_types.h"
#include <stdbool.h>

void actuator_init();
void actuator_set(enum ACTUATOR_STATE state, uint8_t pin_num);
void set_actuator_LED(enum ACTUATOR_STATE state, enum ACTUATOR_ID actuator);
enum ACTUATOR_STATE get_actuator_state(uint8_t pin_num);

#endif /*ACTUATOR_H*/
