#include <xc.h>

#include "canlib/canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"
// #include "board.h"
#include "actuator.h"

const float mA_SENSE_CONVERT_FACTOR =
    10000 * 3.3 / 4096.0f; //  uV conversion / 100 V/V multiplier * vref / 12bit adc
const float BATT_CONVERT_FACTOR = 1000 * 3.3 / 4096.0f; // mV conversion * vref / 12bit adc
//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

static bool battery_voltage_critical = false;

bool check_battery_voltage_error(adcc_channel_t battery_channel) { // returns mV
    adc_result_t batt_raw = ADCC_GetSingleConversion(battery_channel);
    // adc_result_t batt_raw = 0;

    // Vref: 3.3V, Resolution: 12 bits -> raw ADC value is precisely in mV
    uint16_t batt_voltage_mV = (float)batt_raw * BATT_CONVERT_FACTOR;

    // get the un-scaled battery voltage (voltage divider)
    // we don't care too much about precision - some truncation is fine
    batt_voltage_mV = batt_voltage_mV * 4;

    if (batt_voltage_mV < ACTUATOR_BATT_UNDERVOLTAGE_THRESHOLD_mV ||
        batt_voltage_mV > ACTUATOR_BATT_OVERVOLTAGE_THRESHOLD_mV) {

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

bool check_5v_current_error(adcc_channel_t current_channel) { // Check bus current error

    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 62; // 62 is R8 rating in mR

    if (curr_draw_mA > BUS_OVERCURRENT_THRESHOLD_mA) {
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

bool check_12v_current_error(adcc_channel_t current_channel) { // check battery current error
    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 15; // 15 is R7 rating in mR

    if (curr_draw_mA > BAT_OVERCURRENT_THRESHOLD_mA) {
        uint32_t timestamp = millis();
        uint8_t curr_data[2] = {0};
        curr_data[0] = (curr_draw_mA >> 8) & 0xff;
        curr_data[1] = (curr_draw_mA >> 0) & 0xff;

        can_msg_t error_msg;
        build_board_stat_msg(timestamp, E_BATT_OVER_CURRENT, curr_data, 2, &error_msg);
        txb_enqueue(&error_msg);
        return false;
    }

    // things look ok
    return true;
}
