#ifndef ICC_H
#define ICC_H
/*
#include <FreeRTOS.h>
#include "semphr.h"

SemaphoreHandle_t createMutex();

bool mutexTake(SemaphoreHandle_t mutex);

void mutexGive(SemaphoreHandle_t mutex);
*/
#include "pico/mutex.h"

void initMutex(mutex_t *mutex);

bool mutexTake(mutex_t *mutex, uint32_t *owner);

void mutexGive(mutex_t *mutex);

#endif
