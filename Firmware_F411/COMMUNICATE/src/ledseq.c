#include <stdbool.h>
#include "led.h"
#include "ledseq.h"

/* LED���ж��� */
const led_t seq_calibrated[] = /*������У׼�������*/
{
    { true,  LEDSEQ_WAITMS(50) },
    { false, LEDSEQ_WAITMS(450) },
    { 0,     LEDSEQ_LOOP }
};

const led_t seq_alive[] = /*��������*/
{
    { true,  LEDSEQ_WAITMS(50) },
    { false, LEDSEQ_WAITMS(1950) },
    { 0,     LEDSEQ_LOOP }
};

const led_t seq_linkup[] = /*ͨ����������*/
{
    { true,  LEDSEQ_WAITMS(1) },
    { false, LEDSEQ_WAITMS(0) },
    { 0,     LEDSEQ_STOP }
};

/* LED�������ȼ��� */
static led_t const * sequences[] = 
{
    seq_calibrated,
    seq_alive,
    seq_linkup
};

#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

/* �������� */
static void updateActive(led_e led);
static int getPrio(const led_t *seq);
static void runLedseq(xTimerHandle xTimer);

/* ȫ�ֱ��� */
static bool isInit = false;
static bool ledseqEnabled = true;
static int activeSeq[LED_NUM];          /* ÿ��LED��ǰ��������� */
static int state[LED_NUM][SEQ_NUM];     /* ÿ��LED�����еĵ�ǰ״̬ */
static xTimerHandle timer[LED_NUM];     /* ÿ��LED�Ķ�ʱ�� */
static xSemaphoreHandle ledseqSem;      /* ���ʿ����ź��� */

/**
 * @brief ��ʼ��LED���п���ģ��
 */
void ledseqInit()
{
    int i, j;
    
    if (isInit) return;

    ledInit();
    
    /* ��ʼ��״̬���� */
    for (i = 0; i < LED_NUM; i++) 
    {
        activeSeq[i] = LEDSEQ_STOP;
        for (j = 0; j < SEQ_NUM; j++)
            state[i][j] = LEDSEQ_STOP;
    }
    
    /* ����ÿ��LED�������ʱ�� */
    for (i = 0; i < LED_NUM; i++)
        timer[i] = xTimerCreate("ledseqTimer", 1000, pdFALSE, (void*)i, runLedseq);

    vSemaphoreCreateBinary(ledseqSem);  /* ������ֵ�ź��� */

    isInit = true;
}

/**
 * @brief ����LED����ģ���Ƿ�����
 * @return ���Խ��
 */
bool ledseqTest(void)
{
    bool status = isInit & ledTest();
    ledseqEnable(true);
    return status;
}

/**
 * @brief ���û����LED���п���
 * @param enable �Ƿ�����
 */
void ledseqEnable(bool enable)
{
    ledseqEnabled = enable;
}

/**
 * @brief �������е���˸ʱ��
 * @param sequence Ҫ�޸ĵ�����
 * @param onTime ����ʱ��
 * @param offTime Ϩ��ʱ��
 */
void ledseqSetTimes(led_t *sequence, int onTime, int offTime)
{
    sequence[0].action = onTime;
    sequence[1].action = offTime;
}

/**
 * @brief ����ָ��LED������
 * @param led LED���
 * @param sequence Ҫ���е�����
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
 * @brief ��ʱ���ص�������ִ��LED���е���һ��
 * @param xTimer �������Ķ�ʱ��
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
                
            default:  /* LED��ʱ���� */
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
 * @brief ��ȡ���������ȼ����е�λ��
 * @param sequence Ҫ���ҵ�����
 * @return ���е����ȼ�������-1��ʾδ�ҵ�
 */
static int getPrio(const led_t *sequence)
{
    for (int i = 0; i < SEQ_NUM; i++)
        if (sequences[i] == sequence) 
            return i;

    return -1; /* δ�ҵ����� */
}

/**
 * @brief ����LED�Ļ����Ϊ������ȼ�������
 * @param ledIndex LED���
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
