#ifndef __GENERIC_DEBUG_SERIAL_IRQ_H
#define __GENERIC_DEBUG_SERIAL_IRQ_H

#include <stdint.h> // uint32_t
#include "autoconf.h" // CONFIG_DEBUG

#ifdef CONFIG_DEBUG
// callback provided by board specific code
void Debug_serial_enable_tx_irq(void);

// serial_irq.c
void Debug_serial_rx_byte(uint_fast8_t data);
int Debug_serial_get_tx_byte(uint8_t *pdata);
void Debug_sendf(char *format, ...);
#else
// No-op stubs when debug is disabled
#define Debug_serial_enable_tx_irq() do {} while(0)
#define Debug_serial_rx_byte(data) do {} while(0)
#define Debug_serial_get_tx_byte(pdata) (0)
#define Debug_sendf(...) do {} while(0)
#endif

#endif // serial_irq.h
