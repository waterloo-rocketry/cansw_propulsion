// Pickle-BARO: PIC18F Driver for the MS5607-02BA03
//
// Copyright (c) 2012 Roman Schmitz
// Copyright (c) 2019 Alex Mihaila
//
// The following code is a derivative work of the code from the arduino-ms5xxx project,
// which is licensed GPLv3. This code therefore is also licensed under the terms
// of the GNU Public License, version 3.
//
// Pickle-BARO is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Pickle-BARO is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Pickle-BARO.  If not, see <http://www.gnu.org/licenses/>.

#include "mcc_generated_files/device_config.h"
#include <xc.h>
#include <math.h>
#include "my2c.h"

#include "baro.h"

// i2c commands
#define BARO_RESET    0x1E    // perform reset
#define ADC_READ 0x00    // initiate read sequence
// The actual conversion command sent needs to be the result of or-ing ADC_CONV
// D1 or D2, and an oversampling ratio.
#define ADC_CONV 0x40    // start conversion
#define ADC_D1   0x00    // read ADC 1
#define ADC_D2   0x10    // read ADC 2
#define ADC_256  0x00    // set ADC oversampling ratio to 256
#define ADC_512  0x02    // set ADC oversampling ratio to 512
#define ADC_1024 0x04    // set ADC oversampling ratio to 1024
#define ADC_2048 0x06    // set ADC oversampling ratio to 2048
#define ADC_4096 0x08    // set ADC oversampling ratio to 4096

// PROM addresses
#define PROM_INFO  0xA0    // read factory info
#define PROM_C1    0xA2    // read correction factor 1
#define PROM_C2    0xA4    // read correction factor 2
#define PROM_C3    0xA6    // read correction factor 3
#define PROM_C4    0xA8    // read correction factor 4
#define PROM_C5    0xAA    // read correction factor 5
#define PROM_C6    0xAC    // read correction factor 6
#define PROM_CRC   0xAE    // read CRC (unused)

uint8_t prom_cmds[8] = {PROM_INFO, PROM_C1, PROM_C2, PROM_C3, PROM_C4,
        PROM_C5, PROM_C6, PROM_CRC};

// Correction factors
static uint16_t c[8] = {0};

// Sensor I2C address
static uint8_t sensor_addr = 0x0;

void baro_init(uint8_t i2c_addr) {
    sensor_addr = i2c_addr;

    bool err = MY2C_writeCmd(sensor_addr, BARO_RESET);
    if (err) {
        __delay_ms(5);
        MY2C_writeCmd(sensor_addr, BARO_RESET);
    }
    __delay_ms(5);

    // read PROM coefficients
    for (uint8_t i = 0; i < 7; ++i) {
        c[i] = MY2C_read2ByteRegister(sensor_addr, prom_cmds[i]);
    }
}

void baro_start_conversion(uint8_t cmd) {
    MY2C_writeCmd(sensor_addr, ADC_CONV | cmd);
}

static uint32_t read_adc_result(void) {
    uint8_t data[3];
    MY2C_readNByteRegister(sensor_addr, ADC_READ, data, 3);

    uint32_t result = (uint32_t)(data[0]) << 16
                    | (uint32_t)(data[1]) << 8
                    | (uint32_t)(data[2]) << 0;
    return result;
}

void baro_read(double *temperature, double *pressure) {
    baro_start_conversion(ADC_D1 | ADC_256);
    __delay_ms(1);
    uint32_t d1 = read_adc_result();

    baro_start_conversion(ADC_D2 | ADC_256);
    __delay_ms(1);
    uint32_t d2 = read_adc_result();

    double dT;
    double OFF;
    double SENS;
    double TEMP;
    double P;

    // calculate 1st order pressure and temperature (MS5607 1st order algorithm)
    dT = d2 - c[5] * POW2(8);
    OFF = c[2] * POW2(17) + dT * c[4] / POW2(6);
    SENS = c[1] * POW2(16) + dT * c[3] / POW2(7);
    TEMP = (2000 + (dT * c[6]) / POW2(23));
    P = (((d1 * SENS) / POW2(21) - OFF) / POW2(15));

    // perform higher order corrections
    double T2 = 0., OFF2 = 0., SENS2 = 0.;
    if (TEMP < 2000) {
      T2 = dT * dT / POW2(31);
      OFF2 = 61 * (TEMP - 2000) * (TEMP - 2000) / POW2(4);
      SENS2 = 2 * (TEMP - 2000) * (TEMP - 2000);
      if (TEMP < -1500) {
        OFF2 += 15 * (TEMP + 1500) * (TEMP + 1500);
        SENS2 += 8 * (TEMP + 1500) * (TEMP + 1500);
      }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    P = (((d1 * SENS) / POW2(21) - OFF) / POW2(15));

    *temperature = TEMP;
    *pressure = P;
}
