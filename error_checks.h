#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include "canlib/message_types.h"

#include <stdbool.h>

// at this voltage, the actuator will revert to its safe state
#define ACTUATOR_BATT_UNDERVOLTAGE_PANIC_THRESHOLD_mV 9000

// at this voltage, a warning will be sent out over CAN
#define ACTUATOR_BATT_UNDERVOLTAGE_THRESHOLD_mV 9500

// at this voltage, a warning will be sent out over CAN
#define ACTUATOR_BATT_OVERVOLTAGE_THRESHOLD_mV 14000

// From 5V bus line. At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_mA 50

// General board status checkers
bool check_battery_voltage_error(void);
bool is_batt_voltage_critical(void);
bool check_actuator_pin_error(enum ACTUATOR_STATE req_state);

// General board status checkers
bool check_bus_current_error(void);

#endif /* ERROR_CHECKS_H */
