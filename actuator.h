#ifndef ACTUATOR_H
#define	ACTUATOR_H


#include <stdbool.h>
#include "canlib/message_types.h"

#define ACTUACTOR_ID 0
#define SAFE_STATE ACTUATOR_OFF


void actuator_init();

void actuator_set(enum ACTUATOR_STATE state, uint8_t pin_num);

#endif /*ACTUATOR_H*/
