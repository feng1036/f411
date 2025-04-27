#include "watchdog.h"
#include "debug_assert.h"

bool watchdogTest(void)
{
	bool wasNormalStart = true;

	// 检查是否是由独立看门狗复位引起的
	// 使用具体的位定义（bit 29）替代 RCC_CSR_IWDGRSTF
	if (RCC->CSR & (1u << 29)) // IWDGRSTF 位于 CSR 寄存器的第29位
	{
		// 清除所有复位标志
		// 使用具体的位定义（bit 24）替代 RCC_CSR_RMVF
		RCC->CSR |= (1u << 24); // RMVF 位于 CSR 寄存器的第24位
		wasNormalStart = false;
		printAssertSnapshotData();
	}
	return wasNormalStart;
}

// 看门狗复位函数
void watchdogReset()
{
	// 写入特定的键值KR=0xAAAA，启动看门狗计数器重载
	IWDG->KR = 0xAAAA;
}

void watchdogInit(u16 xms)
{
	// 写入键值KR=0x5555，解锁PR和RLR寄存器的写保护
	IWDG->KR = 0x5555;
	
	// 设置预分频器为32 (0x03)
	IWDG->PR = 0x03; // IWDG_Prescaler_32
	
	// 设置重载值
	// IWDG计数器频率 = LSI/32 = 32KHz/32 = 1KHz
	// 超时时间(ms) = 重载值/1KHz * 1000 = 重载值
	// 因此，重载值 = 超时时间(ms) * 1.47 (根据原代码)
	IWDG->RLR = (u16)(1.47 * xms);

	// 重置看门狗计数器
	watchdogReset();
	// IWDG->KR = 0xAAAA;
	
	// 写入键值KR=0xCCCC，启动看门狗
	IWDG->KR = 0xCCCC;
}
