#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

// void (*foo)(int);
typedef void (*watchdog_irq_handler)(void);

void watchdog_start(watchdog_irq_handler handler,uint8_t timeoutInSeconds);
void watchdog_stop();
void watchdog_reset();

#endif
