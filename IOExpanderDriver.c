/*
 * File:   IOExpanderDriver.c
 * Author: Pranav Mahabal
 *
 * Created on June 13, 2024, 8:47 PM
 */

#include "rocketlib/include/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define PCA_ADDRESS 0x41

#define INPUT 0x00
#define OUTPUT 0x01
#define POLARITY_REG 0x02
#define CONFIGURATION 0x03

void pca_init() {
    
}

//
void pca_set_output(uint8_t states) {
    i2c_write_reg8(PCA_ADDRESS, OUTPUT, states);
    i2c_write_reg8(PCA_ADDRESS, CONFIGURATION, 0); // temp fix so that we can set states before default pcb config causes them to open 
}

uint8_t pca_get_output() {
    uint8_t current_state;
    i2c_read_reg8(PCA_ADDRESS, OUTPUT, &current_state);
    return current_state;
}
