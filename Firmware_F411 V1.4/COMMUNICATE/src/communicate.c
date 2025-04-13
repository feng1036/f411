//和stablizier进行通信的代码
// Created by zhaoxinyu on 2025/4/1.

/********************************************************************************	 
 *文件名：	communicate_with_stablilizer.cpp
 *作者：	赵薪宇
 *描述：	和stablilizer进行通信的代码,定义了如何与stablilizer通信,包括收发信息的队列,收发函数等
********************************************************************************/
#include "communicate.h"

static xQueueHandle atkp_stabilizer_queue;	/*和stablilizer通信的队列*/

static bool isInit = false;

void communicateInit(void)
{
    if(isInit) return;
    atkp_stabilizer_queue = xQueueCreate(1, sizeof(atkp_t));	/*创建和stablilizer通信的队列*/
    ASSERT(atkp_stabilizer_queue);
    isInit = true;
}

bool atkp_write(atkp_t *p)
{
    return (pdTRUE==xQueueSend(atkp_stabilizer_queue, p, 0));	/*发送遥控数据到stablilizer通信的队列*/
}

bool atkp_read(atkp_t *p)
{
    return (pdTRUE==xQueueReceive(atkp_stabilizer_queue, p, 0));	/*接收数据从stablilizer通信的队列*/
}

bool communicateTest(void)
{
    return isInit;
}



