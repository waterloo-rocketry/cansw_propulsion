#!/bin/bash
gcc ../actuator.c ../error_checks.c ../IOExpanderDriver.c ../main.c ../sensor_general.c -I. -I../rocketlib/include -DBOARD_UNIQUE_ID=BOARD_ID_PROPULSION_INJ ../canlib/can_common.c emu.c -I../canlib/ ../canlib/util/can_tx_buffer.c ../canlib/util/safe_ring_buffer.c -I../mcc_generated_files/adc -D__uint24=uint32_t ../canlib/util/timing_util.c -lm -o cansw_propulsion

