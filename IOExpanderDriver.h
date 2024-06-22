/*
 * File:   IOExpanderDriver.h
 * Author: Pranav Mahabal
 *
 * Created on June 13, 2024, 8:47 PM
 */

#ifndef IOEXPANDERDRIVER_H
#define IOEXPANDERDRIVER_H
#include <stdint.h>

void pca_init();
void pca_set_output(uint8_t states);
uint8_t pca_get_output();

#endif /* IOEXPANDERDRIVER_H */
