#include "pico_i2c_bridge.h"
#include <Arduino.h>
#include <Wire.h>

extern "C" int pico_i2c_bridge_init(uint32_t clock_hz)
{
    Wire.begin();
    Wire.setClock(clock_hz);
    return 0;
}

extern "C" int pico_i2c_bridge_write(uint8_t dev_addr, uint8_t reg_addr,
                                      const uint8_t *data, uint32_t len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (len > 0 && data != nullptr) {
        // 关键修复：显式转成 size_t，消除重载歧义
        Wire.write(data, (size_t)len);
    }
    return (Wire.endTransmission(true) == 0) ? 0 : -1;
}

extern "C" int pico_i2c_bridge_read(uint8_t dev_addr, uint8_t reg_addr,
                                     uint8_t *data, uint32_t len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        return -1;
    }
    uint32_t received = Wire.requestFrom((int)dev_addr, (int)len);
    if (received != len) {
        return -1;
    }
    for (uint32_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return 0;
}

extern "C" void pico_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}