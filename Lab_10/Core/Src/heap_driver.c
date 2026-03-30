#include "heap_driver.h"

static uint8_t block_map[BLOCK_COUNT];
static uint8_t *const heap_base = HEAP_START_ADDR;

/* Helper: compute required blocks for a given size */
static size_t get_required_blocks(size_t size)
{
    if (size == 0)
        return 0;
    return (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
}

/* Initialize heap */
void heap_init(void)
{
    for (size_t i = 0; i < BLOCK_COUNT; i++)
        block_map[i] = 0;
}

/* Allocate memory */
void* heap_alloc(size_t size)
{
    if (size == 0 || size > HEAP_SIZE)
        return NULL;

    size_t needed_blocks = get_required_blocks(size);
    size_t free_run = 0;
    size_t start_index = 0;

    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 0)
        {
            if (free_run == 0)
                start_index = i;
            free_run++;
            if (free_run == needed_blocks)
            {
                for (size_t j = start_index; j < start_index + needed_blocks; j++)
                    block_map[j] = 1;
                return (void *)(heap_base + start_index * BLOCK_SIZE);
            }
        }
        else
        {
            free_run = 0;
        }
    }
    return NULL;
}

/* Free memory */
void heap_free(void* ptr)
{
    if (ptr == NULL)
        return;

    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t heap_start = (uintptr_t)HEAP_START_ADDR;
    uintptr_t heap_end = heap_start + HEAP_SIZE;

    if (addr < heap_start || addr >= heap_end)
        return;

    if ((addr - heap_start) % BLOCK_SIZE != 0)
        return;

    size_t index = (addr - heap_start) / BLOCK_SIZE;
    while (index < BLOCK_COUNT && block_map[index] == 1)
    {
        block_map[index] = 0;
        index++;
    }
}