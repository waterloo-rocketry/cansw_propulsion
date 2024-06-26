#include <xc.h>

#include "canlib/canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
#include "board.h"
#include "actuator.h"

//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

static bool battery_voltage_critical = false;

bool check_battery_voltage_error(void){    //returns mV
    adc_result_t batt_raw = ADCC_GetSingleConversion(channel_VBAT);

    // Vref: 4.096V, Resolution: 12 bits -> raw ADC value is precisely in mV
    uint16_t batt_voltage_mV = (uint16_t)batt_raw;

    // get the un-scaled battery voltage (voltage divider)
    // we don't care too much about precision - some truncation is fine
    batt_voltage_mV = batt_voltage_mV * 3.7;

    if (batt_voltage_mV < ACTUATOR_BATT_UNDERVOLTAGE_THRESHOLD_mV
            || batt_voltage_mV > ACTUATOR_BATT_OVERVOLTAGE_THRESHOLD_mV) {

        uint32_t timestamp = millis();
        uint8_t batt_data[2] = {0};
        batt_data[0] = (batt_voltage_mV >> 8) & 0xff;
        batt_data[1] = (batt_voltage_mV >> 0) & 0xff;
        enum BOARD_STATUS error_code = batt_voltage_mV < ACTUATOR_BATT_UNDERVOLTAGE_THRESHOLD_mV
                ? E_BATT_UNDER_VOLTAGE
                : E_BATT_OVER_VOLTAGE;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, error_code, batt_data, 2, &error_msg);
        txb_enqueue(&error_msg);

        // main loop should check this and go to safe state if needed
        if (batt_voltage_mV < ACTUATOR_BATT_UNDERVOLTAGE_PANIC_THRESHOLD_mV) {
            // need to go to safe state
            battery_voltage_critical = true;
        } else {
            // low on battery but still ok
            battery_voltage_critical = false;
        }

        // shit's bad yo
        return false;
    }

    // also send the battery voltage as a sensor data message
    // this may or may not be the best place to put this
    can_msg_t batt_msg;
    build_analog_data_msg(millis(), SENSOR_BATT_VOLT, batt_voltage_mV, &batt_msg);
    txb_enqueue(&batt_msg);

    // things look ok
    battery_voltage_critical = false;
    return true;
}

bool is_batt_voltage_critical(void) {
    return battery_voltage_critical;
}

bool check_actuator_pin_error(enum ACTUATOR_STATE req_state) {
    enum ACTUATOR_STATE cur_state = get_actuator_state();
    bool valid = true;
    if (cur_state == ACTUATOR_ILLEGAL) { valid = false; }

    if (!valid) {
        uint8_t state_data[2] = {0};
        state_data[0] = req_state;
        state_data[1] = cur_state;
        can_msg_t error_msg;
        build_board_stat_msg(millis(), E_ACTUATOR_STATE, state_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    return true;
}

bool check_bus_current_error(void){
    // ADC is using FVR of 1.024V
    adc_result_t sense_raw_mV = ADCC_GetSingleConversion(channel_ANA0) / 4; // FIXME ADC Channel
    int curr_draw_mA = (sense_raw_mV) / 20;

    if (curr_draw_mA > OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_5V_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}