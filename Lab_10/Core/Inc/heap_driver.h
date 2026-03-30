#ifndef HEAP_DRIVER_H
#define HEAP_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#define HEAP_START_ADDR ((uint8_t*)0x20001000)
#define HEAP_SIZE       (4*1024)   // 4 KB heap
#define BLOCK_SIZE      16          // 16-byte blocks
#define BLOCK_COUNT     (HEAP_SIZE / BLOCK_SIZE)

void heap_init(void);
void* heap_alloc(size_t size);
void heap_free(void* ptr);

#endif // HEAP_DRIVER_HS