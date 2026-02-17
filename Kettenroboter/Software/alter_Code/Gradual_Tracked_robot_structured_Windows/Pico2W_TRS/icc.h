#ifndef ICC_H
#define ICC_H

#include <FreeRTOS.h>
#include "semphr.h"

SemaphoreHandle_t createMutex();

bool mutexTake(SemaphoreHandle_t mutex);

void mutexGive(SemaphoreHandle_t mutex);

#endif
