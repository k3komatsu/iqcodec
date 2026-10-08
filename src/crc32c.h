#ifndef CRC32C_H
#define CRC32C_H
#include <stddef.h>
#include <stdint.h>
// CRC-32C of the bytes in buf, continuing from crc (start with 0).
uint32_t crc32c(uint32_t crc, const void *buf, size_t n);
#endif
