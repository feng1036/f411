#ifndef COMMUNICATE_WITH_STABILIZER_H
#define COMMUNICATE_WITH_STABILIZER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#include "stdint.h"
#include "semphr.h"

#define SYSLINK_MTU 32

#define CRTP_START_BYTE 0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

// Defined packet types
#define SYSLINK_GROUP_MASK 0xF0

#define SYSLINK_RADIO_GROUP 0x00
#define SYSLINK_RADIO_RAW 0x00
#define SYSLINK_RADIO_CHANNEL 0x01
#define SYSLINK_RADIO_DATARATE 0x02
#define SYSLINK_RADIO_CONTWAVE 0x03
#define SYSLINK_RADIO_RSSI 0x04
#define SYSLINK_RADIO_ADDRESS 0x05

#define SYSLINK_PM_GROUP 0x10
#define SYSLINK_PM_SOURCE 0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF 0x11
#define SYSLINK_PM_BATTERY_VOLTAGE 0x12
#define SYSLINK_PM_BATTERY_STATE 0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14

#define SYSLINK_OW_GROUP 0x20
#define SYSLINK_OW_SCAN 0x20
#define SYSLINK_OW_GETINFO 0x21
#define SYSLINK_OW_READ 0x22
#define SYSLINK_OW_WRITE 0x23

#define UARTSLK_TYPE USART2
#define UARTSLK_PERIF ((uint32_t)0x00020000)
#define ENABLE_UARTSLK_RCC RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ USART2_IRQn

#define UARTSLK_GPIO_PERIF ((uint32_t)0x00000001)
#define UARTSLK_GPIO_PORT GPIOA
#define UARTSLK_GPIO_TX_PIN ((uint16_t)0x0004)
#define UARTSLK_GPIO_RX_PIN ((uint16_t)0x0008)
#define UARTSLK_GPIO_AF_TX_PIN ((uint8_t)0x02)
#define UARTSLK_GPIO_AF_RX_PIN ((uint8_t)0x03)
#define UARTSLK_GPIO_AF_TX ((uint8_t)0x07)
#define UARTSLK_GPIO_AF_RX ((uint8_t)0x07)

#define UARTSLK_TXEN_PERIF ((uint32_t)0x00000001)
#define UARTSLK_TXEN_PORT GPIOA
#define UARTSLK_TXEN_PIN ((uint16_t)0x0001)
#define UARTSLK_TXEN_EXTI ((uint32_t)0x00001)

/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define REMOTE_MAX_DATA_SIZE 30

/*通讯数据结构*/
typedef struct
{
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[REMOTE_MAX_DATA_SIZE];
} RemoteData_t;

/*上行指令ID*/
typedef enum
{
    UP_VERSION = 0x00,
    UP_STATUS = 0x01,
    UP_SENSER = 0x02,
    UP_RCDATA = 0x03,
    UP_GPSDATA = 0x04,
    UP_POWER = 0x05,
    UP_MOTOR = 0x06,
    UP_SENSER2 = 0x07,
    UP_FLYMODE = 0x0A,
    UP_SPEED = 0x0B,
    UP_PID1 = 0x10,
    UP_PID2 = 0x11,
    UP_PID3 = 0x12,
    UP_PID4 = 0x13,
    UP_PID5 = 0x14,
    UP_PID6 = 0x15,
    UP_RADIO = 0x40,
    UP_MSG = 0xEE,
    UP_CHECK = 0xEF,

    UP_REMOTER = 0x50,
    UP_PRINTF = 0x51,

    UP_USER_DATA1 = 0xF1,
    UP_USER_DATA2 = 0xF2,
    UP_USER_DATA3 = 0xF3,
    UP_USER_DATA4 = 0xF4,
    UP_USER_DATA5 = 0xF5,
    UP_USER_DATA6 = 0xF6,
    UP_USER_DATA7 = 0xF7,
    UP_USER_DATA8 = 0xF8,
    UP_USER_DATA9 = 0xF9,
    UP_USER_DATA10 = 0xFA,
} upmsgID_e;

/*下行指令*/
#define D_COMMAND_ACC_CALIB 0x01
#define D_COMMAND_GYRO_CALIB 0x02
#define D_COMMAND_MAG_CALIB 0x04
#define D_COMMAND_BARO_CALIB 0x05
#define D_COMMAND_ACC_CALIB_EXIT 0x20
#define D_COMMAND_ACC_CALIB_STEP1 0x21
#define D_COMMAND_ACC_CALIB_STEP2 0x22
#define D_COMMAND_ACC_CALIB_STEP3 0x23
#define D_COMMAND_ACC_CALIB_STEP4 0x24
#define D_COMMAND_ACC_CALIB_STEP5 0x25
#define D_COMMAND_ACC_CALIB_STEP6 0x26
#define D_COMMAND_FLIGHT_LOCK 0xA0
#define D_COMMAND_FLIGHT_ULOCK 0xA1

#define D_ACK_READ_PID 0x01
#define D_ACK_READ_VERSION 0xA0
#define D_ACK_RESET_PARAM 0xA1
/*下行指令ID*/
typedef enum
{
    DOWN_COMMAND = 0x01,
    DOWN_ACK = 0x02,
    DOWN_RCDATA = 0x03,
    DOWN_POWER = 0x05,
    DOWN_FLYMODE = 0x0A,
    DOWN_PID1 = 0x10,
    DOWN_PID2 = 0x11,
    DOWN_PID3 = 0x12,
    DOWN_PID4 = 0x13,
    DOWN_PID5 = 0x14,
    DOWN_PID6 = 0x15,
    DOWN_RADIO = 0x40,

    DOWN_REMOTER = 0x50,
} downmsgID_e;

void communicateInit(void);
BaseType_t RemoteData_write(RemoteData_t *p);
BaseType_t RemoteData_read(RemoteData_t *p);

void radiolinkTask(void *param);
void uartslkInit(void); /*串口初始化*/
bool uartslkTest(void);
bool uartslkGetDataWithTimout(u8 *c);                /*从接收队列读取数据(带超时处理)*/
void uartslkSendData(u32 size, u8 *data);            /*发送原始数据*/
void uartslkSendDataIsrBlocking(u32 size, u8 *data); /*中断方式发送原始数据*/
int uartslkPutchar(int ch);                          /*发送一个字符到串口*/
void uartslkIsr(void);                               /*串口中断服务函数*/

#endif // COMMUNICATE_WITH_STABILIZER_H
