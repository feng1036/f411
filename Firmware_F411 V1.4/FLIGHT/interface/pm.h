#ifndef __PM_H
#define __PM_H
#include <stdbool.h>

void pmInit(void);
bool pmTest(void);
float pmGetBatteryVoltage(void);

#endif /* __PM_H */
