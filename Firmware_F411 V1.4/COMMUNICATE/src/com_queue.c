#include "com_queue.h"

void queue_init(bool init)
{
	sensor_data_queue=xQueueCreate(1,12*sizeof(float));
	sensor_gyroBias_queue=xQueueCreate(1,sizeof(bool));
}

bool sensor_data_Read(sensorData_t* sensordata)
{
    return(pdTRUE==xQueueReceive(sensor_data_queue,sensordata,0));
}

void sensor_data_Write(sensorData_t* sensordata)
{
    xQueueOverwrite(sensor_data_queue,sensordata);
}

bool sensor_gyroBias_Read(bool* gyroBias)
{
    return(pdTRUE==xQueueReceive(sensor_gyroBias_queue,gyroBias,0));
}

void sensor_gyroBias_Write(bool* gyroBias)
{
    xQueueOverwrite(sensor_gyroBias_queue,gyroBias);
}
