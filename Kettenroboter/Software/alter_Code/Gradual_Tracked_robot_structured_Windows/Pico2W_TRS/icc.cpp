#include "icc.h"
#include "Arduino.h"
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