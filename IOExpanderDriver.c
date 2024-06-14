/* 
 * File:   IOExpanderDriver.c
 * Author: Pranav Mahabal
 *
 * Created on June 13, 2024, 8:47 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "rocketlib/include/i2c.h"
#include <stdint.h>

#define PCA_ADDRESS 0x41
#define CONFIGURATION 0x3
#define OUTPUT 0x0
void pca_init()
{
    i2c_write_reg8(PCA_ADDRESS, CONFIGURATION, OUTPUT);
    
}
void set_output(uint8_t states)
{
    i2c_write_reg8(PCA_ADDRESS, 1, states);
    
}
