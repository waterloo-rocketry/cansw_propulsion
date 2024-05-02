#ifndef MY2C_H
#define	MY2C_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdbool.h>

void MY2C_init(void);

bool MY2C_write(uint8_t address, uint8_t *data, uint8_t len);

bool MY2C_read(uint8_t address, uint8_t *data, uint8_t len);

bool MY2C_writeCmd(uint8_t address, uint8_t cmd);

void MY2C_readNByteRegister(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

bool MY2C_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t val);

uint8_t MY2C_read1ByteRegister(uint8_t address, uint8_t reg);

uint16_t MY2C_read2ByteRegister(uint8_t address, uint8_t reg);

#ifdef	__cplusplus
}
#endif

#endif	/* MY2C_H */

