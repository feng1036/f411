#include <stdbool.h>
#include "led.h"
#include "ledseq.h"

/* LED序列定义 */
const led_t seq_calibrated[] = /*传感器校准完成序列*/
{
    { true,  LEDSEQ_WAITMS(50) },
    { false, LEDSEQ_WAITMS(450) },
    { 0,     LEDSEQ_LOOP }
};

const led_t seq_alive[] = /*开机序列*/
{
    { true,  LEDSEQ_WAITMS(50) },
    { false, LEDSEQ_WAITMS(1950) },
    { 0,     LEDSEQ_LOOP }
};

const led_t seq_linkup[] = /*通信连接序列*/
{
    { true,  LEDSEQ_WAITMS(1) },
    { false, LEDSEQ_WAITMS(0) },
    { 0,     LEDSEQ_STOP }
};

/* LED序列优先级表 */
static led_t const * sequences[] = 
{
    seq_calibrated,
    seq_alive,
    seq_linkup
};

#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

/* 函数声明 */
static void updateActive(led_e led);
static int getPrio(const led_t *seq);
static void runLedseq(xTimerHandle xTimer);

/* 全局变量 */
static bool isInit = false;
static bool ledseqEnabled = true;
static int activeSeq[LED_NUM];          /* 每个LED当前激活的序列 */
static int state[LED_NUM][SEQ_NUM];     /* 每个LED各序列的当前状态 */
static xTimerHandle timer[LED_NUM];     /* 每个LED的定时器 */
static xSemaphoreHandle ledseqSem;      /* 访问控制信号量 */

/**
 * @brief 初始化LED序列控制模块
 */
void ledseqInit()
{
    int i, j;
    
    if (isInit) return;

    ledInit();
    
    /* 初始化状态数组 */
    for (i = 0; i < LED_NUM; i++) 
    {
        activeSeq[i] = LEDSEQ_STOP;
        for (j = 0; j < SEQ_NUM; j++)
            state[i][j] = LEDSEQ_STOP;
    }
    
    /* 创建每个LED的软件定时器 */
    for (i = 0; i < LED_NUM; i++)
        timer[i] = xTimerCreate("ledseqTimer", 1000, pdFALSE, (void*)i, runLedseq);

    vSemaphoreCreateBinary(ledseqSem);  /* 创建二值信号量 */

    isInit = true;
}

/**
 * @brief 测试LED序列模块是否正常
 * @return 测试结果
 */
bool ledseqTest(void)
{
    bool status = isInit & ledTest();
    ledseqEnable(true);
    return status;
}

/**
 * @brief 启用或禁用LED序列控制
 * @param enable 是否启用
 */
void ledseqEnable(bool enable)
{
    ledseqEnabled = enable;
}

/**
 * @brief 设置序列的闪烁时间
 * @param sequence 要修改的序列
 * @param onTime 点亮时间
 * @param offTime 熄灭时间
 */
void ledseqSetTimes(led_t *sequence, int onTime, int offTime)
{
    sequence[0].action = onTime;
    sequence[1].action = offTime;
}

/**
 * @brief 运行指定LED的序列
 * @param led LED编号
 * @param sequence 要运行的序列
 */
void ledseqRun(led_e led, const led_t *sequence)
{
    int prio = getPrio(sequence);
    
    if (prio < 0) return;

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    state[led][prio] = 0; 
    updateActive(led);
    xSemaphoreGive(ledseqSem);

    if (activeSeq[led] == prio)
        runLedseq(timer[led]);
}

/**
 * @brief 定时器回调函数，执行LED序列的下一步
 * @param xTimer 被触发的定时器
 */
static void runLedseq(xTimerHandle xTimer)
{
    bool leave = false;
    const led_t *step;
    led_e led = (led_e)pvTimerGetTimerID(xTimer);

    if (!ledseqEnabled) return;

    while (!leave) 
    {
        int prio = activeSeq[led];

        if (prio == LEDSEQ_STOP)
            return;

        step = &sequences[prio][state[led][prio]];
        state[led][prio]++;

        xSemaphoreTake(ledseqSem, portMAX_DELAY);
        switch (step->action)
        {
            case LEDSEQ_LOOP:
                state[led][prio] = 0;
                break;
                
            case LEDSEQ_STOP:
                state[led][prio] = LEDSEQ_STOP;
                updateActive(led);
                break;
                
            default:  /* LED定时操作 */
                ledSet(led, step->value);
                
                if (step->action != 0) {
                    xTimerChangePeriod(xTimer, step->action, 0);
                    xTimerStart(xTimer, 0);
                    leave = true;
                }
                break;
        }
        xSemaphoreGive(ledseqSem);
    }
}

/**
 * @brief 获取序列在优先级表中的位置
 * @param sequence 要查找的序列
 * @return 序列的优先级索引，-1表示未找到
 */
static int getPrio(const led_t *sequence)
{
    for (int i = 0; i < SEQ_NUM; i++)
        if (sequences[i] == sequence) 
            return i;

    return -1; /* 未找到序列 */
}

/**
 * @brief 更新LED的活动序列为最高优先级的序列
 * @param ledIndex LED编号
 */
static void updateActive(led_e ledIndex)
{
    ledSet(ledIndex, false);
    activeSeq[ledIndex] = LEDSEQ_STOP;
    
    for (int i = 0; i < SEQ_NUM; i++)
    {
        if (state[ledIndex][i] != LEDSEQ_STOP)
        {
            activeSeq[ledIndex] = i;
            break;
        }
    }
}
