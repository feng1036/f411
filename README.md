![山东大学校徽与中英文校名标准组合_横式_-removebg-preview](Document/sdu.png)

# 基于微控制器虚拟化技术的高可信无人机操作系统设计



[TOC]



## 基本信息

---

| 赛题     | [proj381 - 基于微控制器虚拟化技术的高可信无人机操作系统设计](https://os.educg.net/#/sList?TYPE=2025OS_F) |
| -------- | :----------------------------------------------------------: |
| 队名     |                       `sduer`你要自信                        |
| 团队成员 |                    付翔宇、冯彬 、赵薪宇                     |
| 指导教师 |                        潘润宇、颜廷坤                        |
| 项目导师 |                            肖银皓                            |
| 参赛学校 |                           山东大学                           |



## 项目说明

---
-   项目文档(初赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计.doc](Document/基于微控制器虚拟化技术的高可信无人机操作系统设计.doc)
-   项目PPT(初赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT（sduer你要自信队）.pptx](https://pan.baidu.com/s/1ccpIxvr6AX9CqtCojuIUPw?pwd=1234)
-   项目汇报视频(初赛) ：[项目汇报视频](https://pan.baidu.com/s/1Djhv1h4sxJCPpxlVL3C_Og?pwd=1234)
-   无人机飞行测试视频(初赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1k3KI1naTStXgV1jWZ5nHWg?pwd=1234)

**注意：本项目的  比赛题目分析和技术调研 、 系统框架设计 、 重要进展 、 系统测试情况 、 遇到的主要问题以及解决方法 、 本项目创新点 等更加详细的说明均在项目文档中。**


## 项目文件说明

---
```
Code_Library
├─ report.txt
├─ rvmdk-stm32f411ceu6fc.rvp
├─ Stable_Fly
│  ├─ Sensor
│  │  ├─ Source
│  │  │  ├─ i2c.c
│  │  │  ├─ prc_sensor.c
│  │  │  ├─ prc_sensor_desc.c
│  │  │  ├─ rmp_hook.c
│  │  │  └─ sensor.c
│  │  └─ Include
│  │     ├─ i2c.h
│  │     ├─ prc_sensor.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ sensor.h
│  ├─ Remote
│  │  ├─ Source
│  │  │  ├─ prc_remote.c
│  │  │  ├─ prc_remote_desc.c
│  │  │  ├─ remote.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include
│  │     ├─ prc_remote.h
│  │     ├─ remote.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Realtime
│  │  ├─ Source
│  │  │  ├─ prc_realtime.c
│  │  │  ├─ prc_realtime_desc.c
│  │  │  └─ prc_realtime_thd_startup.c
│  │  └─ Include
│  │     ├─ prc_realtime.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Opflow
│  │  ├─ Source
│  │  │  ├─ optical_flow.c
│  │  │  ├─ prc_opflow.c
│  │  │  ├─ prc_opflow_desc.c
│  │  │  ├─ rmp_hook.c
│  │  │  ├─ vl53l1x.c
│  │  │  ├─ vl53l1_api.c
│  │  │  ├─ vl53l1_api_calibration.c
│  │  │  ├─ vl53l1_api_core.c
│  │  │  ├─ vl53l1_api_debug.c
│  │  │  ├─ vl53l1_api_preset_modes.c
│  │  │  ├─ vl53l1_api_strings.c
│  │  │  ├─ vl53l1_core.c
│  │  │  ├─ vl53l1_core_support.c
│  │  │  ├─ vl53l1_error_strings.c
│  │  │  ├─ vl53l1_register_funcs.c
│  │  │  ├─ vl53l1_silicon_core.c
│  │  │  ├─ vl53l1_wait.c
│  │  │  └─ vl53lxx_i2c.c
│  │  └─ Include
│  │     ├─ optical_flow.h
│  │     ├─ prc_opflow.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     ├─ vl53l1x.h
│  │     ├─ vl53l1_api.h
│  │     ├─ vl53l1_api_calibration.h
│  │     ├─ vl53l1_api_core.h
│  │     ├─ vl53l1_api_debug.h
│  │     ├─ vl53l1_api_preset_modes.h
│  │     ├─ vl53l1_api_strings.h
│  │     ├─ vl53l1_core.h
│  │     ├─ vl53l1_core_support.h
│  │     ├─ vl53l1_def.h
│  │     ├─ vl53l1_error_codes.h
│  │     ├─ vl53l1_error_exceptions.h
│  │     ├─ vl53l1_error_strings.h
│  │     ├─ vl53l1_ll_def.h
│  │     ├─ vl53l1_ll_device.h
│  │     ├─ vl53l1_nvm_map.h
│  │     ├─ vl53l1_platform.h
│  │     ├─ vl53l1_platform_log.h
│  │     ├─ vl53l1_platform_user_config.h
│  │     ├─ vl53l1_platform_user_data.h
│  │     ├─ vl53l1_platform_user_defines.h
│  │     ├─ vl53l1_preset_setup.h
│  │     ├─ vl53l1_register_funcs.h
│  │     ├─ vl53l1_register_map.h
│  │     ├─ vl53l1_register_settings.h
│  │     ├─ vl53l1_register_structs.h
│  │     ├─ vl53l1_silicon_core.h
│  │     ├─ vl53l1_tuning_parm_defaults.h
│  │     ├─ vl53l1_types.h
│  │     ├─ vl53l1_wait.h
│  │     └─ vl53lxx_i2c.h
│  ├─ Monitor
│  │  ├─ Source
│  │  │  ├─ rvm_boot.c
│  │  │  └─ rvm_hook.c
│  │  └─ Include
│  │     ├─ rvm_boot.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ rvm_platform_stm32f411ce.h
│  ├─ Lua
│  │  ├─ Source
│  │  │  ├─ lapi.c
│  │  │  ├─ lauxlib.c
│  │  │  ├─ lbaselib.c
│  │  │  ├─ lcode.c
│  │  │  ├─ lcorolib.c
│  │  │  ├─ lctype.c
│  │  │  ├─ ldblib.c
│  │  │  ├─ ldebug.c
│  │  │  ├─ ldo.c
│  │  │  ├─ ldump.c
│  │  │  ├─ lfunc.c
│  │  │  ├─ lgc.c
│  │  │  ├─ linit.c
│  │  │  ├─ liolib.c
│  │  │  ├─ llex.c
│  │  │  ├─ lmathlib.c
│  │  │  ├─ lmem.c
│  │  │  ├─ loadlib.c
│  │  │  ├─ lobject.c
│  │  │  ├─ lopcodes.c
│  │  │  ├─ loslib.c
│  │  │  ├─ lparser.c
│  │  │  ├─ lstate.c
│  │  │  ├─ lstring.c
│  │  │  ├─ lstrlib.c
│  │  │  ├─ ltable.c
│  │  │  ├─ ltablib.c
│  │  │  ├─ ltm.c
│  │  │  ├─ lua.c
│  │  │  ├─ lundump.c
│  │  │  ├─ lutf8lib.c
│  │  │  ├─ lvm.c
│  │  │  ├─ lzio.c
│  │  │  ├─ prc_lua.c
│  │  │  ├─ prc_lua_desc.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include
│  │     ├─ lapi.h
│  │     ├─ lauxlib.h
│  │     ├─ lcode.h
│  │     ├─ lctype.h
│  │     ├─ ldebug.h
│  │     ├─ ldo.h
│  │     ├─ lfunc.h
│  │     ├─ lgc.h
│  │     ├─ ljumptab.h
│  │     ├─ llex.h
│  │     ├─ llimits.h
│  │     ├─ lmem.h
│  │     ├─ lobject.h
│  │     ├─ lopcodes.h
│  │     ├─ lopnames.h
│  │     ├─ lparser.h
│  │     ├─ lprefix.h
│  │     ├─ lstate.h
│  │     ├─ lstring.h
│  │     ├─ ltable.h
│  │     ├─ ltm.h
│  │     ├─ lua.h
│  │     ├─ lua.hpp
│  │     ├─ luaconf.h
│  │     ├─ lualib.h
│  │     ├─ lundump.h
│  │     ├─ lvm.h
│  │     ├─ lzio.h
│  │     ├─ prc_lua.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Kernel
│  │  ├─ Source
│  │  │  ├─ rme_boot.c
│  │  │  └─ rme_hook.c
│  │  └─ Include
│  │     ├─ rme_boot.h
│  │     ├─ rme_platform.h
│  │     ├─ rme_platform_a7m_conf.h
│  │     └─ rme_platform_stm32f411ce.h
│  ├─ Flight
│  │  ├─ Source
│  │  │  ├─ flight.c
│  │  │  ├─ prc_flight.c
│  │  │  ├─ prc_flight_desc.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include
│  │     ├─ flight.h
│  │     ├─ prc_flight.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  └─ Common
│     ├─ arm_common_tables.h
│     ├─ arm_const_structs.h
│     ├─ arm_math.h
│     ├─ core_cm4.h
│     ├─ core_cm4_simd.h
│     ├─ core_cmFunc.h
│     ├─ core_cmInstr.h
│     ├─ stm32f4xx.h
│     ├─ stm32f4xx_conf.h
│     └─ system_stm32f4xx.h
├─ M7M02_Ammonite
│  ├─ Source
│  │  ├─ Virtlib
│  │  │  └─ rvm_virtlib.c
│  │  ├─ Syslib
│  │  │  └─ rvm_syslib.c
│  │  ├─ Platform
│  │  │  └─ A7M
│  │  │     ├─ rvm_platform_a7m.c
│  │  │     ├─ rvm_platform_a7m_armcc.s
│  │  │     └─ rvm_platform_a7m_gcc.s
│  │  └─ Monitor
│  │     └─ rvm_monitor.c
│  └─ Include
│     ├─ rvm.h
│     ├─ Virtlib
│     │  └─ rvm_virtlib.h
│     ├─ Syslib
│     │  └─ rvm_syslib.h
│     ├─ Platform
│     │  └─ A7M
│     │     ├─ rvm_platform_a7m.h
│     │     ├─ rvm_platform_a7m.rva
│     │     └─ Chip
│     │        └─ STM32F411CE
│     │           ├─ rvm_platform_stm32f411ce.h
│     │           └─ rvm_platform_stm32f411ce.rvc
│     └─ Monitor
│        └─ rvm_monitor.h
├─ M7M01_Eukaron
│  ├─ Source
│  │  ├─ Platform
│  │  │  └─ A7M
│  │  │     ├─ rme_platform_a7m.c
│  │  │     ├─ rme_platform_a7m_armcc.s
│  │  │     └─ rme_platform_a7m_gcc.s
│  │  └─ Kernel
│  │     └─ rme_kernel.c
│  └─ Include
│     ├─ rme.h
│     ├─ Platform
│     │  └─ A7M
│     │     ├─ rme_platform_a7m.h
│     │     └─ Chip
│     │        └─ STM32F411CE
│     │           └─ rme_platform_stm32f411ce.h
│     └─ Kernel
│        └─ rme_kernel.h
└─ M5P01_Prokaron
   ├─ Source
   │  ├─ Platform
   │  │  ├─ A7M_RVM
   │  │  │  ├─ rmp_platform_a7m_rvm.c
   │  │  │  ├─ rmp_platform_a7m_rvm_armcc.s
   │  │  │  └─ rmp_platform_a7m_rvm_gcc.s
   │  │  └─ A7M
   │  │     ├─ rmp_platform_a7m.c
   │  │     ├─ rmp_platform_a7m_armcc.s
   │  │     └─ rmp_platform_a7m_gcc.s
   │  ├─ Kernel
   │  │  └─ rmp_kernel.c
   │  └─ Hook
   │     ├─ rmp_hook.c
   │     └─ rmp_hook_rvm.c
   └─ Include
      ├─ rmp.h
      ├─ Platform
      │  ├─ A7M_RVM
      │  │  ├─ rmp_platform_a7m_chip_rvm.h
      │  │  └─ rmp_platform_a7m_rvm.h
      │  └─ A7M
      │     └─ rmp_platform_a7m.h
      └─ Kernel
         └─ rmp_kernel.h
```

## 项目简介

---

随着无人机在通信、测绘、物流、安防等领域的广泛应用，其飞控系统面临着日益复杂的功能集成需求。传统飞控系统多运行于资源受限的微控制器上，采用`FreeRTOS、RT-Thread`等轻量级RTOS，并将所有控制与功能模块部署于单一操作系统环境中。然而，缺乏模块隔离机制使得任何单点故障均可能引发系统整体失效，严重威胁无人机运行的稳定性与安全性。

本项目旨在探索一种**基于微控制器虚拟化技术的无人机高可信操作系统架构**。通过在[RME](https://github.com/EDI-Systems/M7M01_Eukaron)(`RTOS-Mutate-Eukaron`)基础上引入[RVM](https://github.com/EDI-Systems/M7M02_Ammonite)（`Real-time Virtual machine Monitor`）进行虚拟化分区，充分利用Cortex-M平台的**内存保护单元（MPU）**能力，将飞控系统中的不同功能模块（如AI识别、避障导航等）划分至多个隔离的保护域中，并为每个保护域分配更简单高效易用的全抢占式实时操作系统——[RMP](https://github.com/EDI-Systems/M5P01_Prokaron)。这一设计使系统在面对个别模块异常、崩溃或被攻击时，仍能维持核心飞控功能的稳定运行，实现容错与抗攻击能力的大幅提升。

最终，本项目构建了一个**具有高度安全性、灵活性与可扩展性**的无人机操作系统架构模型，为嵌入式领域探索可信执行环境提供了参考，也为未来无人系统的系统级安全设计提供了可行路径。

### 系统架构

![系统整体架构](Document/structure.jpg)

### 硬件平台 

| 开发板  | ARM Cortex-M4 32位RISC内核的微控制器                         |
| ------- | ------------------------------------------------------------ |
| **MCU** | **`STM32F411CEU6 (Cortex-M4, 96MHz, 128KB SRAM, 512KB Flash)`** |



## 项目目标

---

本项目旨在构建一种**面向资源受限嵌入式平台的高可信无人机系统架构**，探索**微控制器虚拟化技术**在无人机飞控中的应用。基于这一总体目变，我们将项目拆分为以下几个子目标：

| 目标编号 | 目标内容                                                     | 完成情况           |
| -------- | ------------------------------------------------------------ | ------------------ |
| 1        | 完成RME实时操作系统的部署运行                                | :heavy_check_mark: |
| 2        | 完成RVM虚拟机监视器的部署运行                                | :heavy_check_mark: |
| 3        | 实现对开源飞控代码（正点原子）的功能模块化拆分与虚拟化移植        | :heavy_check_mark: |
| 4        | 构建跨保护域通信与功能访问机制                                | :heavy_check_mark: |
| 5        | 引入更轻量、高效、易于验证的RMP内核对原有操作系统进行替换与重构   | :heavy_check_mark: |
| 6        | 完成系统级安全测试框架与失效恢复验证                          | :heavy_check_mark: |



## 预期决赛目标

---

目标一：扩展系统兼容性，集成更多第三方功能模块，打造飞控“应用生态”原型

目标二：完善微控制器级虚拟化机制，实现可靠、可控、可部署的飞控分区调度环境



## 项目开发进展

---
📦 ` 2025 年 3 月份`

> ✅ 完成：
>
> - 编写STM32F411CEU6平台的RVM虚拟化启动代码
> - 配置Benchmark测试工程以评估初步性能
> - 执行虚拟化平台初步适配与迁移验证
> - 移除正点原子飞控项目中的Bootloader依赖
> - 精简原有项目中的非必要遥控器相关逻辑
>
> 📌 计划： 
>
> - 完成RVM启动入口`_start()`在STM32F411上的部署与验证
>
> - 编写`benchmark_config.mk`实现多配置选项编译支持
> - 成功构建并运行基础虚拟机实例，评估上下文切换延迟
> - 注释/清除飞控原始Bootloader相关链接脚本与初始化代码
> - 删除遥控器通道绑定、协议解码等冗余模块函数

📦 ` 2025 年 4 月份`

> ✅ 完成：
>
> - 对现有飞控工程代码进行结构性裁剪
> - 移除USB、光流等非核心扩展模块
> - 重构工程结构，将功能划分为独立子模块
> - 摒弃对STM32官方FWLIB固件库的依赖
> - 修改I2C总线的驱动策略，由DMA方式切换为轮询方式以简化资源控制
>
> 📌 计划： 
>
> - 注释与重构`main_flight.c`与其依赖关系，压缩代码规模
> - 移除`usb_device, optical_flow, usbd_cdc_if`等模块及相关中断服务函数
> - 将`Sensor_Update()`函数从`Flight_Control`中剥离，独立为`sensor.c`模块
> - 新建`modules/, drivers/, core/`等目录并进行源码重组织
> - 移除`stm32f4xx_hal.c/.h`与`stm32f4xx_conf.h`等库文件引用，切换为裸机控制
> - 在`sensor_i2c.c`中实现基于轮询的I2C读写函数`I2C_Read_Byte_Polling()`与`I2C_Write_Byte_Polling()`

📦 ` 2025 年 5 月份`

> ✅ 完成：
>
> - 将系统原有实时操作系统（RTOS）替换为M5P01内核
> - 实现M5P01内核的线程调度机制初始化流程
> - 替换跨线程通信机制为M5P01兼容的消息传递函数
> - 裁剪并移除多余的同步锁机制以精简系统开销
> - 建立事件源访问控制机制，实现权限注册管理
> - 建立虚拟中断与物理中断的映射关系模型
> - 编写适配M5P01内核的事件处理例程
> - 构建基于共享内存的高效数据交互区域
>
> 📌 计划： 
>
> - 移除 `xTaskCreate` 接口，统一使用 `RMP_Thd_Crt` 完成线程创建与调度初始化
> - 调用 `RMP_Thd_Crt(&Thd_1, Sensor_Task)` 创建传感器数据采集线程
> - 以 `RMP_Msgq_Snd_ISR` 替换中断服务中的 `xQueueSendFromISR` 接口，实现消息投递的内核兼容性切换
> - 移除 `xSemaphoreTake` 阻塞接口，替换为基于轮询的非阻塞式资源访问方法
> - 通过 `RVM_Hyp_Evt_Add(22U)` 注册编号为22的事件源，实现事件管理与分发的统一调控
> - 调用 `RVM_Virt_Vct_Reg` 注册 `Contact_Sensor` 事件处理函数至虚拟中断向量表
> - 在 `Contact_Sensor` 函数中提取 `SHARED_SENSOR` 区域共享数据并执行状态更新逻辑
> - 初始化 `SHARED_SENSOR` 指针并完成共享内存区域映射，支持多任务/虚拟机间数据访问

📦 ` 2025 年 6 月份`

> ✅ 完成：
>
> - 重构中断处理流程以适配虚拟化环境下的中断转发与响应机制
> - 设计并实现虚拟机间的通信与共享资源访问模型
> - 修复虚拟化监视器中的指针异常错误以提升系统稳定性
> - 配置并优化中断控制器架构寄存器（如NVIC、VTOR等）
> - 开展平台级兼容性验证与跨架构移植性测试
> - 完成飞控系统在虚拟化平台下的移植与功能适配
> - 构建测试方案并验证虚拟机间通信的正确性与实时性
> - 完成技术文档编写与开发过程的工程化归档
> - 制作涵盖架构原理、技术亮点与测试结果的汇报PPT
> - 录制系统运行场景下的飞行演示视频，直观呈现项目成果
>
> 📌 计划： 
>
> - 实现 `IRQ38_Handler` 中断服务函数并读取 `USART2->DR` 以完成底层串口接收流程
> - 调用 `RVM_Hyp_Evt_Snd` 接口完成虚拟中断触发流程，支持异步通信机制
> - 修正 `_RVM_Run_Ins(Virt)` 函数中重复插入虚拟机运行实例的逻辑错误
> - 配置 `SCB->CCR` 控制寄存器，启用栈对齐功能并开启除零陷阱（DIV_0_TRP）机制
> - 利用 `RME_A7M_PUTCHAR` 接口实现串口调试信息输出，支持系统运行日志记录
> - 使用 `RVM_Virt_Vct_Reg` 函数注册 `UART2_IRQHandler` 到虚拟中断向量表中
> - 对Remote端与Flight端的通信数据进行一致性验证并输出调试结果
> - 撰写《基于微控制器虚拟化技术的高可信无人机操作系统设计》项目技术报告
> - 完成《基于微控制器虚拟化技术的高可信无人机操作系统设计》项目汇报PPT
> - 录制虚拟化平台移植前后系统运行对比飞行演示视频，直观展现系统性能演进


## 项目人员分工

---

| 人员   | 任务分工                                                     |
| ------ | ------------------------------------------------------------ |
| 付翔宇 | 编写STM32F411CEU6平台 RME 虚拟化启动代码;移除 USB、光流等非核心扩展模块;重构工程结构；划分 Flight 模块；编写适配 响应 Sensor 与 Remote 的事件处理例程； |
| 冯彬   | 完成 RVM 启动入口_start()在STM32F411上部署；摒弃对 STM32 官方 FWLIB 固件库的依赖；划分 Sensor 模块；构建基于共享内存的高效数据交互区域； |
| 赵薪宇 | 移除正点原子飞控项目中的Bootloader依赖；修改 I2C 总线的驱动策略简化资源控制;划分 Remote 模块；构建测试攻击代码的编写与注入，进行测试跟踪分析； |



## 比赛收获

---

通过参加此次比赛，我们组深度聚焦于**嵌入式虚拟化与高可信操作系统构建**的系统性项目，在理论与实践的结合中全面提升了系统软件研发能力与工程素养。项目以STM32F411CEU6微控制器为平台，成功完成了**RMP轻量级实时操作系统与RVM虚拟机监控器的移植部署**，并针对正点原子飞控系统进行模块分析与功能划分，构建了多保护域并行运行的高可靠性飞控环境。

在项目实施过程中，我系统掌握了嵌入式操作系统、内存保护机制（MPU）、任务调度、系统调用接口设计等核心技术，积累了虚拟化架构设计与安全隔离机制工程实现的宝贵经验。同时，项目中对飞控系统进行功能最小化重构与模块化封装，使我更加理解“可信执行环境”在实际场景中的必要性与挑战性，培养了面向实际问题进行系统抽象与安全设计的综合能力。

本次比赛不仅加深了我们对操作系统底层原理与嵌入式系统安全机制的理解，也让我体会到将学术研究成果转化为实际系统方案所需的工程协同与思维严谨性。未来，我将继续在系统软件与嵌入式安全方向深入钻研，力求在可信计算、智能终端系统架构等前沿领域探索更多的可能性。

## 项目开源说明

---

Unlicensed

## 参考资料

---

-   [M7M01](https://github.com/EDI-Systems/M7M01_Eukaron)
-   [M7M02](https://github.com/EDI-Systems/M7M02_Ammonite)
-   [M5P01](https://github.com/EDI-Systems/M5P01_Prokaron)
-   [正点原子——`MiniFly`四轴飞行器](http://www.openedv.com/docs/fouraxis-fly/minifly.html)













