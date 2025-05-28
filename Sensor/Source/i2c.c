#include <string.h> 
#include "stm32f4xx.h" 
#include "i2c.h" 

// Static function declarations
static void I2Cdev_Init_Bus(struct I2C_Dev* I2c);
static void I2Cdev_Try_To_Restart_Bus(struct I2C_Dev* I2c);
static inline void I2Cdev_Rough_Loop_Delay(uint32_t Us);
static void I2Cdev_Unlock_Bus(GPIO_TypeDef* Port_SCL, GPIO_TypeDef* Port_SDA,uint16_t Pin_SCL,uint16_t Pin_SDA);

// Bus definition
static const struct I2cDef Sensor_Bus_Def=
{
    .I2C_Port=I2C1,
    .I2C_Perif=RCC_APB1PERIPH_I2C1,
    .I2C_EVIRQn=I2C1_EV_IRQn,
    .I2C_ERIRQn=I2C1_ER_IRQn,
    .I2C_Clock_Speed=I2C_SENSORS_CLOCK_SPEED,
    .GPIO_SCL_Perif=RCC_AHB1PERIPH_GPIOB,
    .GPIO_SCL_Port=GPIOB,
    .GPIO_SCL_Pin=GPIO_PIN_8,
    .GPIO_SCL_Pin_Source=GPIO_PINSOURCE8,
    .GPIO_SDA_Perif=RCC_AHB1PERIPH_GPIOB,
    .GPIO_SDA_Port=GPIOB,
    .GPIO_SDA_Pin=GPIO_PIN_9,
    .GPIO_SDA_Pin_Source=GPIO_PINSOURCE9,
    .GPIO_AF=GPIO_AF_I2C1,
};

// Sensor bus instance
struct I2C_Dev Sensor_Bus=
{
    .Def=&Sensor_Bus_Def,
};

// Rough loop delay
static inline void I2Cdev_Rough_Loop_Delay(uint32_t Us)
{
    volatile uint32_t Delay=0;
    for(Delay=0;Delay<I2CDEV_LOOPS_PER_US*Us;++Delay){}
}

// Try to restart bus
static void I2Cdev_Try_To_Restart_Bus(struct I2C_Dev* I2c)
{
    I2Cdev_Init_Bus(I2c);
}

// Initialize I2C bus
static void I2Cdev_Init_Bus(struct I2C_Dev* I2c)
{
    RCC->AHB1ENR|=((uint32_t)0x00000002);
    RCC->AHB1ENR|=((uint32_t)0x00000002);
    RCC->APB1ENR|=((uint32_t)0x00200000);
    GPIOB->MODER&=~(GPIO_MODER_MODER0<<(16U));
    GPIOB->MODER|=(((uint32_t)(0x01))<<(16U));
    GPIOB->OSPEEDR&=~(GPIO_OSPEEDER_OSPEEDR0<<(16U));
    GPIOB->OSPEEDR|=((uint32_t)(0x02)<<(16U));
    /* Output mode configuration*/
    GPIOB->OTYPER&=~((GPIO_OTYPER_OT_0)<<(8U));
    GPIOB->OTYPER|=(((uint16_t)(0x01))<<(8U));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR&=~(GPIO_PUPDR_PUPDR0<<(16U));
    GPIOB->PUPDR|=(((uint32_t)(0x00))<<(16U));
    GPIOB->MODER&=~(GPIO_MODER_MODER0<<(18U));
    GPIOB->MODER|=(((uint32_t)(0x01))<<(18U));
    GPIOB->OSPEEDR&=~(GPIO_OSPEEDER_OSPEEDR0<<(18U));
    GPIOB->OSPEEDR|=((uint32_t)(0x02)<<(18U));
    /* Output mode configuration*/
    GPIOB->OTYPER&=~((GPIO_OTYPER_OT_0)<<(9U));
    GPIOB->OTYPER|=(((uint16_t)(0x01))<<(9U));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR&=~(GPIO_PUPDR_PUPDR0<<(18U));
    GPIOB->PUPDR|=(((uint32_t)(0x00))<<(18U));
    I2Cdev_Unlock_Bus(I2c->def->GPIO_SCL_Port,I2c->def->GPIO_SDA_Port,I2c->def->GPIO_SCL_Pin,I2c->def->GPIO_SDA_Pin);
    GPIOB->MODER&=~(GPIO_MODER_MODER0<<(16U));
    GPIOB->MODER|=(((uint32_t)(0x02))<<(16U));
    GPIOB->OSPEEDR&=~(GPIO_OSPEEDER_OSPEEDR0<<(16U));
    GPIOB->OSPEEDR|=((uint32_t)(0x02)<<(16U));
    /* Output mode configuration*/
    GPIOB->OTYPER&=~((GPIO_OTYPER_OT_0)<<(8U));
    GPIOB->OTYPER|=(((uint16_t)(0x01))<<(8U));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR&=~(GPIO_PUPDR_PUPDR0<<(16U));
    GPIOB->PUPDR|=(((uint32_t)(0x00))<<(16U));
    GPIOB->MODER&=~(GPIO_MODER_MODER0<<(18U));
    GPIOB->MODER|=(((uint32_t)(0x02))<<(18U));
    GPIOB->OSPEEDR&=~(GPIO_OSPEEDER_OSPEEDR0<<(18U));
    GPIOB->OSPEEDR|=((uint32_t)(0x02)<<(18U));
    /* Output mode configuration*/
    GPIOB->OTYPER &=~((GPIO_OTYPER_OT_0)<<(9U));
    GPIOB->OTYPER|=(((uint16_t)(0x01))<<(9U));
    /* Pull-up Pull down resistor configuration*/
    GPIOB->PUPDR&=~(GPIO_PUPDR_PUPDR0<<(18U));
    GPIOB->PUPDR|=(((uint32_t)(0x00))<<(18U));
    GPIOB->AFR[1]&=~(0xF); // 清除B8的AF设置
    GPIOB->AFR[1]|=(0x4);  // 设置B8为AF4
    GPIOB->AFR[1]&=~(0xF<<4); // 清除B9的AF设置
    GPIOB->AFR[1]|=(0x4<<4);  // 设置B9为AF4
    RCC->APB1RSTR|=(1<<21);    // I2C1 复位使能
    RCC->APB1RSTR&=~(1<<21);   // I2C1 复位解除
    I2C1->CR2&=~I2C_CR2_FREQ;    // 清除频率位
    I2C1->CR2|=8U;                // 8MHz
    I2C1->CR1&=~I2C_CR1_PE;// 禁用I2C
    I2C1->CCR =(uint16_t)(7U| I2C_CCR_FS); // 8MHz/(400KHz*3) + 快速模式
    I2C1->TRISE=3;               
    I2C1->CR1|=I2C_CR1_PE;
    // I2C1 CR1配置
    I2C1->CR1&=0xFBF5;//清除ACK, SMBTYPE和SMBUS位
    I2C1->CR1|=0x0400;//使能ACK
    // I2C1 OAR1配置
    I2C1->OAR1=0x4030;
    // 使能I2C错误中断
    I2C1->CR2|=0x0100;           // 使能错误中断
    NVIC->IP[I2C1_EV_IRQn]=0x70; // 优先级7
    NVIC->ISER[I2C1_EV_IRQn>>5]=(1<<(I2C1_EV_IRQn&0x1F)); // 使能中断
    NVIC->IP[I2C3_ER_IRQn]=0x70;//优先级7
    NVIC->ISER[I2C3_ER_IRQn>>5]=(1<<(I2C3_ER_IRQn&0x1F)); // 使能中断
}

// Unlock I2C bus
static void I2Cdev_Unlock_Bus(GPIO_TypeDef* Port_SCL,GPIO_TypeDef* Port_SDA,uint16_t Pin_SCL,uint16_t Pin_SDA)
{
    // Set SDA high
    Port_SDA->BSRRL = Pin_SDA;
    // Check if SDA is being held low by slave
    while((Port_SDA->IDR&Pin_SDA)==0)
    {
        // Generate clock pulses
        Port_SCL->BSRRL=Pin_SCL;
        GPIO_WAIT_FOR_HIGH(Port_SCL,Pin_SCL,I2CDEV_LOOPS_PER_MS);
        I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
        Port_SCL->BSRRH=Pin_SCL;
        I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
        Port_SCL->BSRRL=Pin_SCL;
        I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
    }
    // Generate start then stop condition
    Port_SCL->BSRRL=Pin_SCL;
    I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
    Port_SDA->BSRRH=Pin_SDA;
    I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
    Port_SDA->BSRRH=Pin_SDA;
    I2Cdev_Rough_Loop_Delay(I2CDEV_CLK_TS);
    // Set both lines high
    Port_SDA->BSRRL=Pin_SDA;
    Port_SCL->BSRRL=Pin_SCL;
    GPIO_WAIT_FOR_HIGH(Port_SCL,Pin_SCL,I2CDEV_LOOPS_PER_MS);
    GPIO_WAIT_FOR_HIGH(Port_SDA,Pin_SDA,I2CDEV_LOOPS_PER_MS);
}
// Initialize I2C device
void I2Cdev_Init(struct I2C_Dev* I2c)
{
    I2Cdev_Init_Bus(I2c);
}
// Create message with internal address
void I2Cdev_Create_Message_Int_Addr(struct I2C_Message *Message,
                                   uint8_t Slave_Address,
                                   bool Is_Internal_16,
                                   uint16_t Int_Address,
                                   uint8_t Direction,
                                   uint32_t Length,
                                   uint8_t *Buffer)
{
    Message->Slave_Address=Slave_Address;
    Message->Direction=Direction;
    Message->Is_Internal_Address=Is_Internal_16;
    Message->Internal_Address=Int_Address;
    Message->Message_Length=Length;
    Message->Status=I2C_ACK;
    Message->Buffer=Buffer;
    Message->Retry_Count=I2C_MAX_RETRIES;
}
// I2C write poll
static bool I2Cdev_Write_Poll(struct I2C_Dev* I2c,struct I2C_Message* Message) 
{
    uint16_t Timeout;
    uint16_t i;
    // Send start condition
    I2C1->CR1|=I2C_CR1_START;
    // Wait for start bit
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_SB))
    {
        if(--Timeout==0) 
        return false;
    }
    // Send device address (write)
    I2C1->DR=Message->Slave_Address<<1;
    
    // Wait for address sent
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_ADDR))
    {
        if(--Timeout==0)
        {
            return false;
        }
        if(I2C1->SR1&I2C_SR1_AF)
        {
            I2C1->SR1&=~I2C_SR1_AF;
            I2C1->CR1|=I2C_CR1_STOP;
            return false;
        }
    }
    // Clear ADDR flag
    (void)I2C1->SR2;
    // Send internal address if needed
    if(Message->Internal_Address!=I2C_NO_INTERNAL_ADDRESS)
    {
        if(Message->Is_Internal_Address)
        {
            // Send high byte
            I2C1->DR=(Message->Internal_Address&0xFF)>>8;
            Timeout=I2C_MESSAGE_TIMEOUT;
            while(!(I2C1->SR1&I2C_SR1_TXE))
            {
                if(--Timeout==0)
                return false;
            }
        }
        // Send low byte
        I2C1->DR=Message->Internal_Address&0xFF;
        Timeout=I2C_MESSAGE_TIMEOUT;
        while(!(I2C1->SR1&I2C_SR1_TXE))
        {
            if(--Timeout==0)
            return false;
        }
        if(Message->Direction==I2C_READ)
        {
            return true;
        }
    }
    // Send data (write operation)
    if(Message->Direction==I2C_WRITE)
    {
        for(i=0;i<Message->Message_Length;i++)
        {
            I2C1->DR=Message->Buffer[i];
            Timeout=I2C_MESSAGE_TIMEOUT;
            while(!(I2C1->SR1&I2C_SR1_TXE))
            {
                if(--Timeout==0)
                return false;
            }
        }
    }
    // Wait for transfer complete
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_BTF))
    {
        if(--Timeout==0)
        return false;
    }
    // Send stop condition
    I2C1->CR1|=I2C_CR1_STOP;
    return true;
}
// I2C read poll
static bool I2Cdev_Read_Poll(struct I2C_Dev* I2c,struct I2cMessage* Message)
{
    uint16_t Timeout;
    // Send start condition
    I2C1->CR1|=I2C_CR1_START;
    // Wait for start bit
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_SB))
    {
        if(--Timeout==0)
        return false;
    }
    // Send device address (read)
    I2C1->DR=(Message->Slave_Address<<1)|0x01;
    // Wait for address sent
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_ADDR))
    {
        if(--Timeout==0)
        return false;
        if(I2C1->SR1&I2C_SR1_AF)
        {
            I2C1->SR1&=~I2C_SR1_AF;
            I2C1->CR1|=I2C_CR1_STOP;
            return false;
        }
    }
    // Configure ACK
    if(Message->Message_Length==1)
    {
        I2C1->CR1&=~I2C_CR1_ACK;
    }
    else
    {
        I2C1->CR1|=I2C_CR1_ACK;
    }
    // Clear ADDR flag
    (void)I2C1->SR2;
    // Read data
    for(int i=0;i<Message->Message_Length;i++)
    {
        if(i==Message->Message_Length-1)
        {
            I2C1->CR1&=~I2C_CR1_ACK;
            I2C1->CR1|=I2C_CR1_STOP;
        }
        
        Timeout=I2C_MESSAGE_TIMEOUT;
        while(!(I2C1->SR1&I2C_SR1_RXNE))
        {
            if(--Timeout==0)
            return false;
        }
        Message->Buffer[i]=I2C1->DR;
    }
    return true;
}
// I2C message transfer
bool I2Cdev_Message_Transfer(struct I2C_Dev* I2c,struct I2cMessage* Message)
{
    bool Status=false;
    int Retries=Message->Retry_Count;
    while(Retries>=0)
    {
        if(Message->Direction==I2C_WRITE||
            Message->Internal_Address!=I2C_NO_INTERNAL_ADDRESS)
        {
            Status=I2Cdev_Write_Poll(I2c,Message);
            if(Status&&Message->Direction==I2C_READ)
            {
                Status=I2Cdev_Read_Poll(I2c,Message);
            }
        }
        else
        {
            Status=I2Cdev_Read_Poll(I2c,Message);
        }
        if(Status)
        {
            break;
        }
        Retries--;
        I2Cdev_Rough_Loop_Delay(100);
    }
    
    if(!Status)
    {
        I2Cdev_Try_To_Restart_Bus(I2c);
    }
    return Status;
}

bool I2Cdev_Read_Byte(struct I2C_Dev *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t *Data)
{
    return I2Cdev_Read(Dev,Dev_Address,Mem_Address,1,Data);
}

bool I2Cdev_Read_Bit(struct I2C_Dev *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t Bit_Num,uint8_t *Data)
{
    uint8_t Byte;
    bool Status=I2Cdev_Read(Dev,Dev_Address,Mem_Address,1,&Byte);
    *Data=Byte&(1<<Bit_Num);
    return Status;
}

bool I2Cdev_Read_Bits(struct I2C_Dev *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t Bit_Start,uint8_t Length,uint8_t *Data)
{
    bool Status;
    uint8_t Byte;
    if((Status=I2Cdev_Read_Byte(Dev,Dev_Address,Mem_Address,&Byte))==true)
    {
        uint8_t Mask=((1<<Length)-1)<<(Bit_Start-Legnth+1);
        Byte&=Mask;
        Byte>>=(Bit_Start-Length+1);
        *Data=Byte;
    }
    return Status;
}

// Device layer implementation
bool I2Cdev_Read(struct I2C_Dev *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint16_t Length,uint8_t *Data)
{
    struct I2cMessage Message;
    I2Cdev_Create_Message_Int_Addr(&Message,Dev_Address,false,Mem_Address,I2C_READ,Length,Data);
    return I2Cdev_Message_Transfer(Dev,&Message);
}

bool I2Cdev_Read16(struct I2C_Dev *Dev,uint8_t Dev_Address,uint16_t Mem_Address,uint16_t Length,uint8_t *Data)
{
    struct I2cMessage Message;
    I2Cdev_Create_Message_Int_Addr(&Message,Dev_Address,true,Mem_Address,I2C_READ,Length,Data);
    return I2Cdev_Message_Transfer(Dev,&Message);
}

bool I2Cdev_Write_Byte(struct I2C_Dev *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t Data)
{
    return I2Cdev_Write(Dev,Dev_Address,Mem_Address,1,&Data);
}

bool I2Cdev_Write_Bit(struct I2cDrv *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t Bit_Num,uint8_t Data)
{
    uint8_t Byte;
    I2Cdev_Read_Byte(Dev,Dev_Address,Mem_Address,&Byte);
    Byte=(Data!=0)?(Byte|(1<<Bit_Num)):(Byte&~(1<<Bit_Num));
    return I2Cdev_Write_Byte(Dev,Dev_Address,Mem_Address,Byte);
}

bool I2Cdev_Write_Bits(struct I2cDrv *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint8_t Bit_Start,uint8_t Length,uint8_t Data)
{
    uint8_t Byte;
    if(I2Cdev_Read_Byte(Dev,Dev_Address,Mem_Address,&Byte))
    {
        uint8_t Mask=((1<<Length)-1)<<(Bit_Start-Length+1);
        Data<<=(Bit_Start-Length+1);
        Data&=Mask;
        Byte&=~Mask;
        Byte|=Data;
        return I2Cdev_Write_Byte(Dev,Dev_Address,Mem_Address,Byte);
    }
    return false;
}

bool I2Cdev_Write(struct I2cDrv *Dev,uint8_t Dev_Address,uint8_t Mem_Address,uint16_t Length,uint8_t *Data)
{
    struct I2cMessage Message;
    I2Cdrv_Create_Message_Int_Addr(&Message,Dev_Address,false,Mem_Address,I2C_WRITE,Length,Data);
    return I2Cdrv_Message_Transfer(Dev,&Message);
}

bool I2Cdev_Write16(struct I2cDrv *Dev,uint8_t Dev_Address,uint16_t Mem_Address,uint16_t Length,uint8_t *Data)
{
    struct I2cMessage Message;
    I2Cdrv_Create_Message_Int_Addr(&Message,Dev_Address,true,Mem_Address,I2C_WRITE,Length,Data);
    return I2Cdrv_Message_Transfer(Dev,&Message);
}


// I2C轮询方式写数据
static bool I2C_Write_Poll(struct I2cDrv* I2c,struct I2cMessage* Message)
{
    uint16_t Timeout;
    //发送起始位
    I2C1->CR1|=I2C_CR1_START;
    //等待起始位发送完成
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_SB))
    {
        if(--Timeout==0)
        return false;
    }
    //发送设备地址(写)
    I2C1->DR=Message->Slave_Address<<1;
    //等待地址发送完成
    Timeout=I2C_MESSAGE_TIMEOUT;
    while (!(I2C1->SR1&I2C_SR1_ADDR))
    {
        if(--Timeout==0)
        return false;
        if(I2C1->SR1&I2C_SR1_AF)
        {
            I2C1->SR1&=~I2C_SR1_AF;
            I2C1->CR1|=I2C_CR1_STOP;
            return false;
        }
    }
    //清除ADDR标志
    uint16_t Temp=I2C1->SR2;
    (void)Temp;
    //发送内部地址（如果有）
    if(Message->Internal_Address!=I2C_NO_INTERNAL_ADDRESS)
    {
        if(Message->Is_Internal_16bit)
        {
            I2C1->DR=(Message->Internal_Address&0xFF00)>>8;
            Timeout=I2C_MESSAGE_TIMEOUT;
            while(!(I2C1->SR1&I2C_SR1_TXE))
            {
                if (--Timeout==0)
                return false;
            }
        }
        I2C1->DR=Message->Internal_Address&0x00FF;
        Timeout=I2C_MESSAGE_TIMEOUT;
        while(!(I2C1->SR1&I2C_SR1_TXE))
        {
            if(--Timeout==0)
            return false;
        }
        if(Message->Direction==I2C_READ)
        {
            return true;//读操作需后续处理
        }
    }
    //发送数据（写操作）
    if(Message->Direction==I2C_WRITE)
    {
        for(int i=0;i<Message->Message_Length;i++)
        {
            I2C1->DR=Message->Buffer[I];
            Timeout=I2C_MESSAGE_TIMEOUT;
            while(!(I2C1->SR1&I2C_SR1_TXE))
            {
                if(--Timeout==0)
                return false;
            }
        }
    }
    // 等待传输完成
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_BTF))
    {
        if(--Timeout==0)
        return false;
    }
    //发送停止位
    I2C1->CR1|=I2C_CR1_STOP;
    return true;
}

// I2C轮询方式读数据
static bool I2C_Read_Poll(struct I2cDrv* I2c,struct I2cMessage* Message)
{
    uint16_t Timeout;
    //发送起始位
    I2C1->CR1|=I2C_CR1_START;
    //等待起始位发送完成
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_SB))
    {
        if(--Timeout==0)
        return false;
    }
    //发送设备地址(读)
    I2C1->DR=(Message->Slave_Address<<1)|0x01;
    // 等待地址发送完成
    Timeout=I2C_MESSAGE_TIMEOUT;
    while(!(I2C1->SR1&I2C_SR1_ADDR))
    {
        if(--Timeout==0)
        return false;
        if(I2C1->SR1&I2C_SR1_AF)
        {
            I2C1->SR1&=~I2C_SR1_AF;
            I2C1->CR1|=I2C_CR1_STOP;
            return false;
        }
    }
    // 配置ACK
    if(Message->Message_Length==1)
    {
        I2C1->CR1&=~I2C_CR1_ACK;//单字节禁用ACK
    }
    else
    {
        I2C1->CR1|=I2C_CR1_ACK;//多字节启用ACK
    }
    //清除ADDR标志
    uint16_t Temp=I2C1->SR2;
    (void)Temp;
    //读取数据
    for(int i=0;i<Message->Message_Length;i++)
    {
        if(i==Message->Message_Length-1)
        {
            I2C1->CR1&=~I2C_CR1_ACK;//最后一个字节发送NACK
            I2C1->CR1|=I2C_CR1_STOP;//发送停止位
        }
        Timeout=I2C_MESSAGE_TIMEOUT;
        while(!(I2C1->SR1&I2C_SR1_RXNE))
        {
            if(--Timeout==0)
            return false;
        }
        Message->Buffer[i]=I2C1->DR;
    }
    return true;
}

// I2C消息传输（主函数）
bool I2Cdev_Message_Transfer(struct I2cDrv* I2c,struct I2cMessage* Message)
{
    bool Status=false;
    int Retry=Message->Retry_Count;
    while(Retry>=0)
    {
        if(Message->Direction==I2C_WRITE||
            Message->Internal_Address!=I2C_NO_INTERNAL_ADDRESS)
        {
            Status=I2C_Write_Poll(I2c,Message);
            if(Status&&Message->Direction==I2C_READ)
            {
                Status=I2C_Read_Poll(I2c,Message);
            }
        } 
        else
        {
            Status=I2C_Read_Poll(I2c,Message);
        }
        if(Status)
        {
            break;
        }
        Retry--;
        I2cdrv_Rough_Loop_Delay(100);
    }
    if(!Status)
    {
        I2cdrv_Try_To_Restart_Bus(I2c);
    }
    return Status;
}
