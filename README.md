![山东大学校徽与中英文校名标准组合_横式_-removebg-preview](Document/sdu.png)

# 基于微控制器虚拟化技术的高可信无人机操作系统设计



[TOC]



## 基本信息

---

| 赛题     | [proj381 - 基于微控制器虚拟化技术的高可信无人机操作系统设计](https://github.com/ML-hacker/FirmFly) |
| -------- | :----------------------------------------------------------: |
| 队名     |                        sduer 你要自信                        |
| 团队成员 |                    付翔宇、冯彬 、赵薪宇                     |
| 指导教师 |                        潘润宇、颜廷坤                        |
| 项目导师 |                            肖银皓                            |
| 参赛学校 |                           山东大学                           |



## 项目说明

---
### 初赛阶段
-   项目文档(初赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计(初赛).doc](https://pan.baidu.com/s/1R3IAmBcJhdIH-pikP2Blxw?pwd=1234)
-   进展汇报PPT(初赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT（sduer你要自信队）.pptx](https://pan.baidu.com/s/1ccpIxvr6AX9CqtCojuIUPw?pwd=1234)
-   项目汇报视频(初赛) ：[项目汇报视频](https://pan.baidu.com/s/1Djhv1h4sxJCPpxlVL3C_Og?pwd=1234)
-   无人机飞行测试视频(初赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1k3KI1naTStXgV1jWZ5nHWg?pwd=1234)

### 决赛阶段
-   项目文档(决赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计.doc](https://pan.baidu.com/s/1ha-8qT-dunuo0-GSEwr2OQ?pwd=1234)
-   进展汇报PPT(决赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT（sduer你要自信队）.pptx](https://pan.baidu.com/s/1reNKg0NNYVLwifpwLowaAA?pwd=1234)
-   项目汇报视频(决赛) ：[项目汇报视频](https://pan.baidu.com/s/1HLv_fYtJK7v5DyPU6-2yaQ?pwd=1234)
-   无人机飞行测试视频(决赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1NADmcFigpwyrFCz9EBL66A?pwd=1234)
-   项目相关图片：[项目图片](https://pan.baidu.com/s/1-FCMzqOF1Ym54UIoIKhKmQ?pwd=1234) 

**注意：本项目的详细说明均在项目文档中。**



## 项目简介

---
### 项目背景


**① 国家战略与产业推动：**《国家综合立体交通网规划纲要》明确，将低空经济上升为国家战略，强调了加强无人机新兴产业的重要性。安全可靠的无人机操作系统作为维护国家低空经济发展的核心抓手，直接关系到国家无人机基础设施建设能否安全稳定运行。近年来，无人机在通信、测绘、物流、安防等领域的应用规模持续扩大，飞行控制系统所需处理的功能愈发多样且复杂。

**② 传统架构瓶颈与微控制虚拟化困境：** 当前仍面临着三大核心挑战。第一，传统无人机多以裸机或单核实时操作系统为主，虽资源占用小，但缺乏硬件隔离，安全性无法得到保障；第二，一些安全可靠的操作系统，成本极高，虚拟化开销极大，无法在低成本微控制器平台上实现大规模部署；第三，现有的操作系统兼容性较差，无法进行广泛推广。

### 项目目标


本项目旨在于**低成本**、**资源受限**的硬件平台上实现一种**低开销**、**高可靠性**的新型**虚拟化**无人机架构，并保持对原有实时操作系统的良好**兼容性**，从而在有限硬件条件下实现稳定、安全且高效的飞行控制与系统运行。

### 项目架构

本项目在 **Cortex-M4** 平台下的 STM32F411CEU6 芯片上，基于**内存保护单元(MPU)**实现了微内核操作系统[RME](https://github.com/EDI-Systems/M7M01_Eukaron) ，轻量级虚拟机监视器 [RVM](https://github.com/EDI-Systems/M7M02_Ammonite) ，超轻量实时操作系统 [RMP](https://github.com/EDI-Systems/M5P01_Prokaron) 的**一对一对多**的虚拟化架构，在此基础上下设五个虚拟机，将飞控系统核心功能精简整合为 Flight  Remote  Sensor  Optical_Flow四个模块，并增设 Lua 模块以提高无人机系统的跨平台性与可移植性。

本项目**首次**实现了基于商用微控制虚拟化技术并集低资源占用、高实时性能、强安全可靠于一体的无人机部署。与传统无人机架构相比，本架构具有以下六个显著优势，即**轻量高效**、**安全可控**、**高可靠性**、**实时响应**、**生态兼容**、**普遍适用**。

![项目整体架构](Document/structure.jpg)

### 项目硬件平台

| 开发板   | ARM Cortex-M4 32位RISC内核的微控制器                         |
| -------- | ------------------------------------------------------------ |
| 微控制器 | **`STM32F411CEU6 (Cortex-M4, 96MHz, 128KB SRAM, 512KB Flash)`** |



## 项目任务

---

###  初赛进展实现

| 编号 | 实现内容                                                     | 完成情况           |
| ---- | ------------------------------------------------------------ | ------------------ |
| 1    | 实现 RME 微内核操作系统对底层 STM32F411 芯片的部署           | :heavy_check_mark: |
| 2    | 实现 RVM 虚拟机监视器对底层 STM32F411 芯片的移植以及对飞控系统的适配 | :heavy_check_mark: |
| 3    | 实现对正点原子开源飞控系统的体系化功能解耦与虚拟化适配       | :heavy_check_mark: |
| 4    | 设计并实现具备高实时性的跨保护域通信机制                     | :heavy_check_mark: |
| 5    | 实现更加轻量、极致高效的 RMP 实时操作系统对原有操作系统的替换与重构 | :heavy_check_mark: |
| 6    | 构建系统级安全测试框架、完成基础实时性能及失效恢复功能验证   | :heavy_check_mark: |



### 决赛进展实现

| 编号 | 实现内容                                                     | 完成情况           |
| ---- | ------------------------------------------------------------ | ------------------ |
| 1    | 优化飞控系统的核心控制逻辑，提升飞行控制环路的精度与响应性能 | :heavy_check_mark: |
| 2    | 新增光流模块，提升系统兼容性与飞行稳定性                     | :heavy_check_mark: |
| 3    | 移植 Lua 脚本引擎，提高跨平台性与可移植性                    | :heavy_check_mark: |
| 4    | 完成对虚拟机各模块内存占用情况的测试与分析                   | :heavy_check_mark: |



### 项目实现框架

![项目实现架构](Document/task.jpg)


### 项目开发进展

📦 ` 2025 年 3 月份`


> 📌 计划： 
>
> - 完成RVM启动入口`_start()`在STM32F411上的部署与验证
>
> - 编写`benchmark_config.mk`实现多配置选项编译支持
> - 成功构建并运行基础虚拟机实例，评估上下文切换延迟
> - 注释/清除飞控原始Bootloader相关链接脚本与初始化代码
> - 删除遥控器通道绑定、协议解码等冗余模块函数
>
> ✅ 完成：
>
> - 编写STM32F411CEU6平台的RVM虚拟化启动代码
> - 配置Benchmark测试工程以评估初步性能
> - 执行虚拟化平台初步适配与迁移验证
> - 移除正点原子飞控项目中的Bootloader依赖
> - 精简原有项目中的非必要遥控器相关逻辑


📦 ` 2025 年 4 月份`


> 📌 计划： 
>
> - 注释与重构`main_flight.c`与其依赖关系，压缩代码规模
> - 移除`usb_device, optical_flow, usbd_cdc_if`等模块及相关中断服务函数
> - 将`Sensor_Update()`函数从`Flight_Control`中剥离，独立为`sensor.c`模块
> - 新建`modules/, drivers/, core/`等目录并进行源码重组织
> - 移除`stm32f4xx_hal.c/.h`与`stm32f4xx_conf.h`等库文件引用，切换为裸机控制
> - 在`sensor_i2c.c`中实现基于轮询的I2C读写函数`I2C_Read_Byte_Polling()`与`I2C_Write_Byte_Polling()`
>
> ✅ 完成：
>
> - 对现有飞控工程代码进行结构性裁剪
> - 移除USB、光流等非核心扩展模块
> - 重构工程结构，将功能划分为独立子模块
> - 摒弃对STM32官方FWLIB固件库的依赖
> - 修改I2C总线的驱动策略，由DMA方式切换为轮询方式以简化资源控制


📦 ` 2025 年 5 月份`


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
>
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


📦 ` 2025 年 6 月份`


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
>
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


📦 ` 2025 年 7 月份`


> 📌 计划： 
>
> - 调整光流与激光模块的线程逻辑，确保数据融合的实时性与稳定性
> - 在rvmdk-stm32f411ceu6fc.rvp中注册光流与激光虚拟机，分配合适大小的内存空间
> - 编写共享缓冲区访问接口，支持光流与激光模块间的高效数据交换
> - 优化融合算法的参数更新逻辑，确保运行过程中参数稳定不被意外修改
> - 在技术文档中新增光流与激光模块的系统集成设计说明
> - 在PPT中增加光流与激光模块的功能演示与数据性能对比内容
>
> ✅ 完成：
>
> - 移除冗余的插件检测代码，精简系统运行开销
> - 整合激光测距模块与光流定位模块，实现多源数据融合
> - 优化光流模块的数据传输逻辑，提升数据更新效率与实时性
> - 精简光流通信Flow_Data结构体，降低内存占用与解析复杂度
> - 测试移植后虚拟化平台us级延迟，实现Delay_Us()接口替换
> - 修改与光流传感器通信的SPI协议通信方式，DMA通信方式为轮询
> - 完成激光与光流模块在虚拟化平台下的移植与功能适配
> - 完成技术文档编写与开发过程的工程化归档
> - 初步优化项目技术文档结构与内容组织
> - 初步优化项目展示PPT的排版与技术亮点呈现方式


📦 ` 2025 年 8 月份`


> 📌 计划： 
>
> - 编写Lua虚拟机的插件加载接口，支持第三方算法在飞控系统中的快速部署
> - 在Lua环境中调用光流与激光模块数据接口，验证跨语言调用的稳定性与实时性
> - 优化光流模块的异常检测机制，提升对低光照与高频抖动场景的适应能力
> - 飞行参数新增只读保护机制，防止运行中被非法修改
> - 定位无人机异常崩溃bug，确定缩减空余空间、扩大Lua空间的优化方向
> - 在项目文档中补充 Lua 虚拟机与光流模块的系统架构与交互流程图
> - 在PPT中增加 Lua 插件支持与光流改进效果的性能对比图表
> - 录制“光流+Lua”模式下的飞行演示视频，用于项目成果展示
>
> ✅ 完成：
>
> - 优化无人机飞行控制参数，提升飞行姿态稳定性与轨迹精度
> - 修复运行中飞行参数被非预期修改的问题，增强系统安全性
> - 新增激光与光流数据缓存变量，提升多任务访问时的数据一致性
> - 优化系统内存分配，缩减空闲已分配内存，为Lua虚拟机提供充足空间
> - 引入Lua虚拟机，实现无人机对第三方插件的动态加载与运行支持
> - 使用RMP动态内存分配系统调用实现Lua动态分配器
> - 实现Lua输出重定向配置
> - 编写Lua脚本测试进行移植测试
> - 进一步完善项目技术文档，补充性能测试与应用场景分析
> - 深化展示PPT的技术亮点与成果呈现逻辑
> - 录制集成光流模块后的飞行演示视频，直观展示改进效果



### 项目人员分工

|  人员  | 任务分工                                                     |
| :----: | :----------------------------------------------------------- |
| 付翔宇 | 编写STM32F411CEU6平台RME虚拟化启动代码;移除USB、ATKP等非核心扩展模块,重构工程结构;划分Flight模块,编写适配响应Sensor、Remote与光流Optical_Flow的事件处理例程;将光流数据融合应用于姿态控制与位置估算中;通过调试飞控系统的PID参数,提升飞行的稳定性与控制精度,实现了飞控系统的高效移植与性能优化。 |
|  冯彬  | 完成RVM启动入口_start()在STM32F411CEU6平台的部署,摒弃对STM32官方FWLIB外设固件库的依赖;重构并划分Sensor模块，剔除未使用的多余传感器代码,同时将I2C硬件通信方式由DMA调整为轮询模式以增加监视器权限监管能力;构建基于共享内存的高效数据交互区域,实现跨虚拟机的传感器数据通信与交互;去除冗余的扩展模块检测代码,成功在初赛基础上移植并适配Lua运行环境,完成Lua脚本的全面测试验证。 |
| 赵薪宇 | 移除正点原子飞控项目中的Bootloader依赖;简化I2C总线驱动策略以优化资源管理;划分 Remote 模块,完成测试攻击代码的编写与注入,并开展系统级测试与跟踪分析;通过透明中断机制有效解决高频UART中断问题;设计读写双缓冲方案以保障读写异步操作的稳定性;在初赛虚拟机基础上成功移植光流虚拟机,实时采集光流数据,为提升飞行稳定性提供有力支持。 |

## 项目设计创新与实现难点()

## 移植后虚拟化平台主要指标测试结果()


## 项目文件说明

---

### 开发及完善代码概要

- **Stable_Fly**
```
├─ Stable_Fly(虚拟化飞控系统整体工程项目)
│  ├─ Kernel(微内核操作系统内核)
│  │  ├─ Source(内核启动与与虚拟化适配)
│  │  │  ├─ rme_boot.c
│  │  │  └─ rme_hook.c
│  │  └─ Include(内核头文件与芯片平台配置)
│  │     ├─ rme_boot.h
│  │     ├─ rme_platform.h
│  │     ├─ rme_platform_a7m_conf.h
│  │     └─ rme_platform_stm32f411ce.h
│  ├─ Monitor(虚拟机监视器,管理各个虚拟机)
│  │  ├─ Source(虚拟机启动与虚拟化接口)
│  │  │  ├─ rvm_boot.c
│  │  │  └─ rvm_hook.c
│  │  └─ Include(监视器头文件与平台配置)
│  │     ├─ rvm_boot.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ rvm_platform_stm32f411ce.h
│  ├─ Flight(飞行控制虚拟机,负责无人机姿态与控制逻辑)
│  │  ├─ Source(飞行控制核心代码与进程描述文件)
│  │  │  ├─ flight.c
│  │  │  ├─ prc_flight.c
│  │  │  ├─ prc_flight_desc.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include(飞行控制头文件与平台配置)
│  │     ├─ flight.h
│  │     ├─ prc_flight.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Sensor(传感器虚拟机,负责处理加速度等传感器数据)
│  │  ├─ Source(传感器采集核心代码与进程描述文件)
│  │  │  ├─ i2c.c
│  │  │  ├─ prc_sensor.c
│  │  │  ├─ prc_sensor_desc.c
│  │  │  ├─ rmp_hook.c
│  │  │  └─ sensor.c
│  │  └─ Include(传感器头文件与平台配置)
│  │     ├─ i2c.h
│  │     ├─ prc_sensor.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ sensor.h
│  ├─ Remote(遥控通信虚拟机,负责遥控数据接受与解析)
│  │  ├─ Source(遥控通信与进程描述)
│  │  │  ├─ prc_remote.c
│  │  │  ├─ prc_remote_desc.c
│  │  │  ├─ remote.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include(遥控通信头文件与平台配置)
│  │     ├─ prc_remote.h
│  │     ├─ remote.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Opflow(光流虚拟机,负责光流传感器通信)
│  │  ├─ Source(光流与VL53L1x传感器驱动)
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
│  │  └─ Include(光流传感器与VL53L1 API头文件)
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
│  ├─ Lua(Lua引擎移植虚拟机)
│  │  ├─ Source(Lua核心源码及适配扩展)
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
│  │  └─ Include(Lua引擎的完整头文件集合)
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
│  │     ├─ rmp_platform_a7m_rvm_conf.h(虚拟化平台适配层配置文件)
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  └─ Common(STM32f4xx芯片相关头文件)
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
```

- **rvmdk-stm32f411ceu6fc.rvp**
```
├─ rvmdk-stm32f411ceu6fc.rvp(飞控项目生成文件)
```
- **M7M01_Eukaron/Include/Platform/A7M/Chip/STM32F411CE/rme_platform_stm32f411ce.h**
```
├─ M7M01_Eukaron(微内核通用实时操作系统)
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
│     │        └─ STM32F411CE(移植到目标芯片平台配置文件)
│     │           └─ rme_platform_stm32f411ce.h
│     └─ Kernel
│        └─ rme_kernel.h
```


- **M7M02_Ammonite/Include/Platform/A7M/Chip/STM32F411CE/rvm_platform_stm32f411ce.h**
- **M7M02_Ammonite/Include/Platform/A7M/Chip/STM32F411CE/rvm_platform_stm32f411ce.rvc**
```
├─ M7M02_Ammonite(虚拟机监视器)
│  ├─ Source()
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
│  └─ Include()
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
│     │        └─ STM32F411CE(适配移植目标芯片平台编写设置文件)
│     │           ├─ rvm_platform_stm32f411ce.h
│     │           └─ rvm_platform_stm32f411ce.rvc
│     └─ Monitor
│        └─ rvm_monitor.h
```

- **M5P01_Prokaron/Include/Test/Chip/rmp_test_stm32f411ce_rvm.h**
```
└─ M5P01_Prokaron(轻量级实时操作系统)
   ├─ Source
   │  ├─ Test
   │  │  ├─ rmp_benchmark.c
   │  │  ├─ rmp_coverage.c
   │  │  ├─ rmp_lwip_iperf.c
   │  │  └─ rmp_tickless.c
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
      ├─ Test
      │  └─ Chip
      │     └─ rmp_test_stm32f411ce_rvm.h(目标芯片benchmark测试配置)
      ├─ Platform
      │  ├─ A7M_RVM
      │  │  ├─ rmp_platform_a7m_chip_rvm.h
      │  │  └─ rmp_platform_a7m_rvm.h
      │  └─ A7M
      │     └─ rmp_platform_a7m.h
      └─ Kernel
         └─ rmp_kernel.h
```
### 完整项目树

```
Code_Library
├─ report.txt
├─ rvmdk-stm32f411ceu6fc.rvp(飞控项目生成文件)
├─ Stable_Fly(虚拟化飞控系统整体工程项目)
│  ├─ Kernel(微内核操作系统内核)
│  │  ├─ Source(内核启动与与虚拟化适配)
│  │  │  ├─ rme_boot.c
│  │  │  └─ rme_hook.c
│  │  └─ Include(内核头文件与芯片平台配置)
│  │     ├─ rme_boot.h
│  │     ├─ rme_platform.h
│  │     ├─ rme_platform_a7m_conf.h
│  │     └─ rme_platform_stm32f411ce.h
│  ├─ Monitor(虚拟机监视器,管理各个虚拟机)
│  │  ├─ Source(虚拟机启动与虚拟化接口)
│  │  │  ├─ rvm_boot.c
│  │  │  └─ rvm_hook.c
│  │  └─ Include(监视器头文件与平台配置)
│  │     ├─ rvm_boot.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ rvm_platform_stm32f411ce.h
│  ├─ Flight(飞行控制虚拟机,负责无人机姿态与控制逻辑)
│  │  ├─ Source(飞行控制核心代码与进程描述文件)
│  │  │  ├─ flight.c
│  │  │  ├─ prc_flight.c
│  │  │  ├─ prc_flight_desc.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include(飞行控制头文件与平台配置)
│  │     ├─ flight.h
│  │     ├─ prc_flight.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Sensor(传感器虚拟机,负责处理加速度等传感器数据)
│  │  ├─ Source(传感器采集核心代码与进程描述文件)
│  │  │  ├─ i2c.c
│  │  │  ├─ prc_sensor.c
│  │  │  ├─ prc_sensor_desc.c
│  │  │  ├─ rmp_hook.c
│  │  │  └─ sensor.c
│  │  └─ Include(传感器头文件与平台配置)
│  │     ├─ i2c.h
│  │     ├─ prc_sensor.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ sensor.h
│  ├─ Remote(遥控通信虚拟机,负责遥控数据接受与解析)
│  │  ├─ Source(遥控通信与进程描述)
│  │  │  ├─ prc_remote.c
│  │  │  ├─ prc_remote_desc.c
│  │  │  ├─ remote.c
│  │  │  └─ rmp_hook.c
│  │  └─ Include(遥控通信头文件与平台配置)
│  │     ├─ prc_remote.h
│  │     ├─ remote.h
│  │     ├─ rmp_platform.h
│  │     ├─ rmp_platform_a7m_rvm_conf.h
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  ├─ Opflow(光流虚拟机,负责光流传感器通信)
│  │  ├─ Source(光流与VL53L1x传感器驱动)
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
│  │  └─ Include(光流传感器与VL53L1 API头文件)
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
│  ├─ Lua(Lua引擎移植虚拟机)
│  │  ├─ Source(Lua核心源码及适配扩展)
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
│  │  └─ Include(Lua引擎的完整头文件集合)
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
│  │     ├─ rmp_platform_a7m_rvm_conf.h(虚拟化平台适配层配置文件)
│  │     ├─ rmp_platform_stm32f411ce_rvm.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
│  └─ Common(STM32f4xx芯片相关头文件)
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
├─ M7M01_Eukaron(微内核通用实时操作系统)
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
├─ M7M02_Ammonite(虚拟机监视器)
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
└─ M5P01_Prokaron(轻量级实时操作系统)
   ├─ Source
   │  ├─ Test
   │  │  ├─ rmp_benchmark.c
   │  │  ├─ rmp_coverage.c
   │  │  ├─ rmp_lwip_iperf.c
   │  │  └─ rmp_tickless.c
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
      ├─ Test
      │  └─ Chip
      │     └─ rmp_test_stm32f411ce_rvm.h(目标芯片benchmark测试配置)
      ├─ Platform
      │  ├─ A7M_RVM
      │  │  ├─ rmp_platform_a7m_chip_rvm.h
      │  │  └─ rmp_platform_a7m_rvm.h
      │  └─ A7M
      │     └─ rmp_platform_a7m.h
      └─ Kernel
         └─ rmp_kernel.h
```


## 比赛收获

---

通过参加此次比赛，我们组深度聚焦于**嵌入式虚拟化与高可信操作系统构建**的系统性项目，在理论与实践的结合中全面提升了系统软件研发能力与工程素养。项目以STM32F411CEU6微控制器为平台，成功完成了**RMP轻量级实时操作系统与RVM虚拟机监控器的移植部署**，并针对正点原子飞控系统进行模块分析与功能划分，构建了多保护域并行运行的高可靠性飞控环境。

在项目实施过程中，我系统掌握了嵌入式操作系统、内存保护机制、任务调度、系统调用接口设计等核心技术，积累了虚拟化架构设计与安全隔离机制工程实现的宝贵经验。同时，项目中对飞控系统进行功能最小化重构与模块化封装，使我更加理解“可信执行环境”在实际场景中的必要性与挑战性，培养了面向实际问题进行系统抽象与安全设计的综合能力。

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

