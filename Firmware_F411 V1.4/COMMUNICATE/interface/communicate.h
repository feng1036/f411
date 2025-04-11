#ifndef COMMUNICATE_WITH_STABILIZER_H
#define COMMUNICATE_WITH_STABILIZER_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "config.h"
#include "comm.h"
#include "atkp.h"
#include "remoter_ctrl.h"

#include "assert.h"

void communicateInit(void);

bool atkp_write(atkp_t *p);

bool atkp_read(atkp_t *p);

#endif // COMMUNICATE_WITH_STABILIZER_H