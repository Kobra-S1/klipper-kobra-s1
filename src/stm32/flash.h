#ifndef __STM32_FLASH_H
#define __STM32_FLASH_H

#include <stdint.h>

struct flash_config {
    int force_erase;
};

// Setup flash configuration
struct flash_config flash_setup(int force_erase);

// Flash operations
int flash_write_byte(struct flash_config *info, uint32_t addr, uint32_t data);
uint32_t flash_read_byte(struct flash_config *info, uint32_t addr);
int flash_erase(struct flash_config *info, uint32_t addr);
int flash_write_page(struct flash_config *info, uint32_t addr, uint32_t *data, uint32_t len);
int flash_read_page(struct flash_config *info, uint32_t addr, uint8_t *data, uint32_t len);

#endif // __STM32_FLASH_H
