#include "verbose.h"
#include <stdarg.h>
#include <stdio.h>

/* 真正的串口输出由C++桥接层(verbose_bridge.cpp)实现，
   因为Arduino的Serial对象是C++类，无法在纯C文件里直接调用 */
extern void compat_serial_write(const char *buf, int len);

static void verbose_vprintf(const char *prefix, const char *format, va_list args)
{
    char buffer[VERBOSE_BUFFER_SIZE];
    int len = 0;

    if (prefix)
    {
        len = snprintf(buffer, sizeof(buffer), "%s", prefix);
        if (len < 0 || len >= (int)sizeof(buffer))
        {
            len = 0;
        }
    }

    int n = vsnprintf(buffer + len, sizeof(buffer) - len, format, args);
    if (n > 0)
    {
        len += n;
        if (len > (int)sizeof(buffer))
        {
            len = sizeof(buffer);
        }
    }

    compat_serial_write(buffer, len);
}

void verbose(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    verbose_vprintf(NULL, format, args);
    va_end(args);
}

void verbose_error(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    verbose_vprintf(NULL, format, args);
    va_end(args);
}

void verbose_warning(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    verbose_vprintf(NULL, format, args);
    va_end(args);
}

void verbose_info(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    verbose_vprintf(NULL, format, args);
    va_end(args);
}

void verbose_write(uint8_t *buffer, uint16_t length)
{
    compat_serial_write((const char *)buffer, length);
}