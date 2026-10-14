#ifndef PICO_I2C_BRIDGE_H
#define PICO_I2C_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int  pico_i2c_bridge_init(uint32_t clock_hz);
int  pico_i2c_bridge_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint32_t len);
int  pico_i2c_bridge_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
void pico_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif