#include "icc.h"
#include "Arduino.h"
#include "pico/mutex.h"
/*
#include <FreeRTOS.h>
#include "semphr.h"

SemaphoreHandle_t createMutex(){
	SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
	return mutex;
}

bool mutexTake(SemaphoreHandle_t mutex){
	if(xSemaphoreTake(mutex, portMAX_DELAY)){
		return true;
	}
	else{
		return false;
	}
}

void mutexGive(SemaphoreHandle_t mutex){
	xSemaphoreGive(mutex);
}
*/


void initMutex(mutex_t *mutex){
	mutex_init(mutex);
}

bool mutexTake(mutex_t  *mutex, uint32_t *owner){
	if(mutex_try_enter(mutex, owner)){
		return true;
	}
	else{
		return false;
	}
}

void mutexGive(mutex_t *mutex){
	mutex_exit(mutex);
}