#ifndef __COM__QUEUE_H
#define __COM__QUEUE_H
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef struct
{
	Axis3f acc;
	Axis3f gyro;
	baro_t baro;
} sensorData_t;

static xQueueHandle sensor_data_queue;
static xQueueHandle sensor_gyroBias_queue;
BaseType_t sensor_data_Read(sensorData_t* sensordata);
void sensor_data_Write(sensorData_t* sensordata);
BaseType_t sensor_gyroBias_Read(bool* gyroBias);
void sensor_gyroBias_Write(bool* gyroBias);
void queue_init(bool init);

#endif
