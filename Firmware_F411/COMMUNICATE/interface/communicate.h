#ifndef COMMUNICATE_WITH_STABILIZER_H
#define COMMUNICATE_WITH_STABILIZER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "atkp.h"

void communicateInit(void);

BaseType_t atkp_write(atkp_t *p);

BaseType_t atkp_read(atkp_t *p);

#endif // COMMUNICATE_WITH_STABILIZER_H
