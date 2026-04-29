#ifndef ICC_H
#define ICC_H

#include "pico/mutex.h"
/*
#include <FreeRTOS.h>
#include "semphr.h"

SemaphoreHandle_t createMutex();

bool mutexTake(SemaphoreHandle_t mutex);

void mutexGive(SemaphoreHandle_t mutex);
*/

void initMutex(mutex_t *mutex);

bool mutexTake(mutex_t *mutex, uint32_t *owner);

void mutexGive(mutex_t *mutex);

#endif
