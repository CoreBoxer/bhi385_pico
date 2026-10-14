#ifndef _VERBOSE_H_
#define _VERBOSE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VERBOSE_BUFFER_SIZE  256

void verbose(const char *format, ...);
void verbose_error(const char *format, ...);
void verbose_warning(const char *format, ...);
void verbose_info(const char *format, ...);
void verbose_write(uint8_t *buffer, uint16_t length);

#define PRINT(format, ...)    verbose(format,##__VA_ARGS__)
#define INFO(format, ...)     verbose_info("[I]"format,##__VA_ARGS__)
#define PRINT_I(format, ...)  verbose_info(format,##__VA_ARGS__)
#define WARNING(format, ...)  verbose_warning("[W]"format,##__VA_ARGS__)
#define PRINT_W(format, ...)  verbose_warning(format,##__VA_ARGS__)
#define ERROR(format, ...)    verbose_error("[E]"format,##__VA_ARGS__)
#define PRINT_E(format, ...)  verbose_error(format,##__VA_ARGS__)
#define DATA(format, ...)     verbose("[D]"format,##__VA_ARGS__)
#define PRINT_D(format, ...)  verbose(format,##__VA_ARGS__)
#define HEX(format, ...)      verbose("[H]"format,##__VA_ARGS__)
#define PRINT_H(format, ...)  verbose(format,##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* _VERBOSE_H_ */