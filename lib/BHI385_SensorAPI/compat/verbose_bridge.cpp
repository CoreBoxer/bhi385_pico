#include <Arduino.h>

extern "C" void compat_serial_write(const char *buf, int len)
{
    Serial.write((const uint8_t *)buf, len);
}