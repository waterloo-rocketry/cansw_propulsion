#ifndef ERROR_CHECKS_H
#define ERROR_CHECKS_H

#include <stdbool.h>

// From 5V bus line. At this current, a warning will be sent out over CAN
#define OVERCURRENT_THRESHOLD_mA 50

// General board status checkers
bool check_bus_current_error(void);

#endif /* ERROR_CHECKS_H */
