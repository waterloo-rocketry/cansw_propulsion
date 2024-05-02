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

#ifndef BARO_H
#define BARO_H

#include <stdint.h>
#include <stdbool.h>

#define BARO_ADDR 0b1110110
#define BARO_LATENCY_MS 5
#define BARO_RESOLUTION_MBAR 0.036f

#define POW2(n) (double)((uint32_t)1 << n)

void baro_init(uint8_t i2c_addr);

// Starts a new ADC conversion, waits for it to complete, and
// computes pressure and temperature from the results.
void baro_read(double *temperature, double *pressure);

#endif
