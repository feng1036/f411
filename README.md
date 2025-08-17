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
<br>
<br>

## 项目说明

---
### 初赛阶段

-   项目文档(初赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计(初赛)](https://pan.baidu.com/s/1R3IAmBcJhdIH-pikP2Blxw?pwd=1234)
-   进展汇报PPT(初赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT(初赛)](https://pan.baidu.com/s/1ccpIxvr6AX9CqtCojuIUPw?pwd=1234)
-   项目汇报视频(初赛) ：[项目汇报视频](https://pan.baidu.com/s/1Djhv1h4sxJCPpxlVL3C_Og?pwd=1234)
-   无人机飞行测试视频(初赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1k3KI1naTStXgV1jWZ5nHWg?pwd=1234)
<br>

### 决赛阶段
-   项目文档(决赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统项目文档](https://pan.baidu.com/s/1GgnmqQsMOfy0e0_sxfsGzA?pwd=1234)
-   进展汇报PPT(决赛) ：[基于微控制器虚拟化技术的高可信操作系统进度汇报PPT](https://pan.baidu.com/s/1V3iGBiWm4UmD59oQVqO4DA?pwd=1234)
-   项目汇报视频(决赛) ：[项目汇报视频](https://pan.baidu.com/s/1RmIfkAdsPLsRvYbej3W8oA?pwd=1234)
-   无人机飞行测试视频(决赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1NADmcFigpwyrFCz9EBL66A?pwd=1234)
-   项目相关图片：[项目图片](https://pan.baidu.com/s/1EK1rh36zy4Wvt8CYr_iCKQ?pwd=1234) 

**注意：本项目的详细说明均在项目文档中**
<br>
<br>

## 项目简介

---

### 项目背景

**① 国家战略与产业推动：**《国家综合立体交通网规划纲要》明确，将低空经济上升为国家战略，强调了加强无人机新兴产业的重要性。安全可靠的无人机操作系统作为维护国家低空经济发展的核心抓手，直接关系到国家无人机基础设施建设能否安全稳定运行。近年来，无人机在通信、测绘、物流、安防等领域的应用规模持续扩大，飞行控制系统所需处理的功能愈发多样且复杂。

**② 传统架构瓶颈与微控制器虚拟化困境：** 当前仍面临着三大核心挑战。第一，传统无人机多以裸机或单核实时操作系统为主，虽资源占用小，但缺乏硬件隔离，安全性无法得到保障；第二，一些安全可靠的操作系统，成本极高，虚拟化开销极大，无法在低成本微控制器平台上实现大规模部署；第三，现有的操作系统兼容性较差，无法进行广泛推广。  
<br>

### 项目目标

本项目旨在于**低成本**、**资源受限**的硬件平台上实现一种**低开销**、**高可靠性**的新型**虚拟化**无人机架构，并保持对原有实时操作系统的良好**兼容性**，从而在有限硬件条件下实现稳定、安全且高效的飞行控制与系统运行。  
<br>

### 项目架构

本项目在 **Cortex-M4** 平台下的 STM32F411CEU6 芯片上，基于**内存保护单元(MPU)**实现了微内核操作系统[RME](https://github.com/EDI-Systems/M7M01_Eukaron) ，轻量级虚拟机监视器 [RVM](https://github.com/EDI-Systems/M7M02_Ammonite) ，超轻量实时操作系统 [RMP](https://github.com/EDI-Systems/M5P01_Prokaron) 的**一对一对多**的虚拟化架构，在此基础上下设五个虚拟机，将飞控系统核心功能精简整合为 Flight、Remote、Sensor、Optical_Flow四个模块，并增设 Lua 模块以支持插件的热更新。  
<br>

### 项目优势

本项目**首次**实现了基于商用微控制虚拟化技术并集**低资源占用、高实时性能、强安全可靠**于一体的无人机部署。与传统无人机架构相比，本架构具有以下六个显著优势：

-   **轻量高效**：支持商用低成本MCU部署，适配资源受限无人机
-   **安全可控**：核心飞控独立分模块运行，防御组件缺陷与恶意入侵
-   **高可靠性**：无感知恢复实现单点故障损失最小化，保障飞行安全
-   **实时响应**：确保关键任务硬实时响应，满足无人机最坏执行时间约束
-   **生态兼容**：支持 GCC 等开源工具链，同时兼容 IAR、Keil 等工具链
-   **普遍适用** ：跨领域复用，支撑可信嵌入式系统构建

![项目整体架构](Document/structure.jpg)
<br>

### 项目硬件平台

| 开发板   | ARM Cortex-M4 32位RISC内核的微控制器                         |
| -------- | ------------------------------------------------------------ |
| 微控制器 | **`STM32F411CEU6 (Cortex-M4, 96MHz, 128KB SRAM, 512KB Flash)`** |
<br>
<br>

## 项目任务

---

### 项目总体实现

本项目由三名**本科生**完成，主要实现了**微控制器虚拟化框架部署**、**飞控系统的模块化重构与虚拟化适配**以及**深度嵌入式实时操作系统的集成与替换**。底层部分，我们将微内核操作系统RME与虚拟机监视器RVM**移植**到 Cortex-M4 架构下的 STM32F411CEU6 芯片上，在资源受限的微控制器上成功部署了虚拟化架构；上层部分，基于开源正点原子飞控代码，我们将飞控系统**重构**为四大核心模块——主飞控模块、传感器模块、通信模块以及光流模块，并**移植**了Lua模块以支持插件的热更新，成功部署至虚拟化架构下五个独立的虚拟机中，；在此基础上，我们还对客户机操作系统进行了轻量实时操作系统RMP的**替换**。

![项目实现架构](Document/task.jpg)
<br>

### 项目实现难点

-   **跨保护域通信机制** ：为实现强隔离虚拟机通信，借助虚拟中断机制+共享内存+消息队列缓冲设计并实现虚拟机通信机制
-   **高频UART透明中断解耦** ：使用透明中断，解耦物理中断与内核调度，降低高频UART中断占用CPU负载
-   **传感器DMA的轮询改造** ：将低频DMA传输改为轮询模式，防止恶意外设利用DMA进行越权访问，从而提升系统安全性
-   **底层架构错误修复** ：通过调试修复监视器指针错误；修改快慢路径切换逻辑，解决长指令执行中的错误  
<br>

###  初赛进展实现

| 编号 | 实现内容                                                     | 完成情况           |
| ---- | ------------------------------------------------------------ | ------------------ |
| 1    | 实现 RME 微内核操作系统对底层 STM32F411 芯片的部署           | :heavy_check_mark: |
| 2    | 实现 RVM 虚拟机监视器对底层 STM32F411 芯片的移植以及对飞控系统的适配 | :heavy_check_mark: |
| 3    | 实现对正点原子开源飞控系统的体系化功能解耦与虚拟化适配       | :heavy_check_mark: |
| 4    | 设计并实现具备高实时性的跨保护域通信机制                     | :heavy_check_mark: |
| 5    | 实现更加轻量、极致高效的 RMP 实时操作系统对原有操作系统的替换与重构 | :heavy_check_mark: |
| 6    | 构建系统级安全测试框架、完成基础实时性能及失效恢复功能验证   | :heavy_check_mark: |
<br>

### 决赛进展实现

| 编号 | 实现内容                                                     | 完成情况           |
| ---- | ------------------------------------------------------------ | ------------------ |
| 1    | 优化飞控系统的核心控制逻辑，提升飞行控制环路的精度与响应性能 | :heavy_check_mark: |
| 2    | 新增光流模块，提升系统兼容性与飞行稳定性                     | :heavy_check_mark: |
| 3    | 移植 Lua 脚本引擎，提高跨平台性与可移植性                    | :heavy_check_mark: |
| 4    | 完成对虚拟机各模块内存占用情况的测试与分析                   | :heavy_check_mark: |
<br>

### 项目开发进展

📦 ` 2025 年 3 月份`


> 📌 计划： 
>
> - 完成RVM启动入口_start()在STM32F411上的部署与验证
> - 编写benchmark项目配置文件，进行系统在目标平台的基本性能测试
> - 成功构建并运行基础虚拟机实例，评估上下文切换延迟
> - 注释/清除飞控原始Bootloader相关链接脚本与初始化代码
> - 删除遥控器通道绑定、协议解码等冗余模块函数
>
> ✅ 完成：
>
> - 编写STM32F411CEU6平台的RVM虚拟化启动代码
> - 配置benchmark测试工程以评估初步性能
> - 执行虚拟化平台初步适配与迁移验证
> - 移除正点原子飞控项目中的Bootloader依赖
> - 精简原有项目中的非必要遥控器相关逻辑


📦 ` 2025 年 4 月份`


> 📌 计划： 
>
> - 注释与重构原正点原子飞行控制代码与其他文件依赖关系，压缩代码规模
> - 移除USB、ATKP、扩展模块等模块及相关中断服务函数
> - 将Sensor传感器相关代码从Flight中剥离，独立为sensor模块
> - 移除FWLIB外设固件库中接口，改为直接寄存器读写方式
> - 在Sensor中实现基于轮询的I2C读写函数I2C_Read_Byte_Polling与I2C_Write_Byte_Polling  
    摒弃原本低频，存在安全隐患的DMA通信方式
>
> ✅ 完成：
>
> - 对现有飞控工程代码进行结构性裁剪
> - 移除USB、光流等非核心扩展模块
> - 重构工程结构，将功能划分为独立子模块
> - 摒弃对STM32官方FWLIB固件库的依赖
> - 修改I2C总线的驱动策略，由DMA方式切换为轮询方式以增强系统安全性


📦 ` 2025 年 5 月份`


> 📌 计划： 
>
> - 移除原本Free RTOS线程相关接口，统一使用RMP_Thd_Crt等系统调用完成线程创建等线程管理
> - 借助RMP中钩子函数实现虚拟机中线程，线程栈与虚拟中断设置
> - 移除xSemaphoreTake等阻塞接口，替换为基于轮询的非阻塞式资源访问方法
> - 阅读原虚拟平台资料，了解虚拟事件机制以及实现流程
> - 调用RVM_Virt_Vct_Reg等接口实现虚拟事件映射，注册中断处理等操作
> - 实现Contact_Sensor等虚拟中断处理例程，读取共享内存数据。
> - 修改项目生成文件增设共享内存区，初始化SHARED_SENSOR指针并完成共享内存区域映射。
>
> ✅ 完成：
>
> - 将系统原有Free RTOS替换为M5P01内核
> - 实现M5P01内核的线程调度机制初始化流程
> - 替换跨线程通信机制为M5P01兼容的消息传递函数
> - 移除多余的同步锁机制以精简系统开销
> - 建立事件源访问控制机制，实现权限注册管理
> - 建立虚拟中断与物理中断的映射关系模型
> - 编写适配M5P01内核的虚拟事件处理例程
> - 构建基于共享内存的高效数据交互区域


📦 ` 2025 年 6 月份`


> 📌 计划： 
>
> - 使用RVM_Virt_Vct_Reg等接口完成UART物理中断配置
> - 实现IRQ38_Handler中断服务函数并读取 USART2->DR以完成底层串口接收流程
> - 进一步完善借助虚拟事件机制实现的跨保护域通信机制
> - 利用RME_A7M_PUTCHAR接口实现串口调试信息输出，支持系统运行日志记录
> - 对Remote端与Flight端的通信数据进行一致性验证并输出调试结果
> - 撰写《基于微控制器虚拟化技术的高可信无人机操作系统设计》项目技术报告
> - 完成《基于微控制器虚拟化技术的高可信无人机操作系统设计》项目汇报PPT
> - 录制虚拟化平台移植前后系统运行对比飞行演示视频，直观展现系统性能演进
>
> ✅ 完成：
>
> - 重构中断处理流程以适配虚拟化环境下的中断转发与响应机制
> - 进一步完善虚拟机间的通信与共享资源访问模型
> - 修复虚拟化监视器中的指针异常错误以提升系统稳定性
> - 配置并优化中断控制器架构寄存器
> - 开展平台级兼容性验证与跨架构移植性测试
> - 完成飞控系统在虚拟化平台下的移植与功能测试
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
> - 在项目文档中补充 Lua 虚拟机与光流模块的系统架构与交互流程图
> - 在PPT中增加 Lua 插件支持与光流模块移植后整体内容描述
> - 录制“光流+Lua”模式下的飞行演示视频，用于项目成果展示
>
> ✅ 完成：
>
> - 优化无人机飞行控制参数，提升飞行姿态稳定性与轨迹精度
> - 新增激光与光流数据缓存变量，提升多任务访问时的数据一致性
> - 优化系统内存分配，缩减空闲已分配内存，为Lua虚拟机提供充足空间
> - 引入Lua虚拟机，实现无人机对第三方插件的动态加载与运行支持
> - 定位无人机异常崩溃bug，确定缩减空余空间、扩大Lua空间的优化方向
> - 使用RMP动态内存分配系统调用实现Lua动态分配器
> - 实现Lua输出重定向配置
> - 编写Lua脚本测试进行移植测试
> - 进一步完善项目技术文档，补充性能测试与应用场景分析
> - 深化展示PPT的技术亮点与成果呈现逻辑
> - 录制集成光流模块后的飞行演示视频，直观展示改进效果
<br>

### 项目人员分工

|  人员  | 任务分工                                                     |
| :----: | :----------------------------------------------------------- |
| 付翔宇 | 编写STM32F411CEU6平台RME虚拟化启动代码;移除USB、ATKP等非核心扩展模块,重构工程结构;划分Flight模块,编写适配响应Sensor、Remote与光流Optical_Flow的事件处理例程;将光流数据融合应用于姿态控制与位置估算中;通过调试飞控系统的PID参数,提升飞行的稳定性与控制精度,实现了飞控系统的高效移植与性能优化。 |
|  冯彬  | 完成RVM启动入口_start()在STM32F411CEU6平台的部署,摒弃对STM32官方FWLIB外设固件库的依赖;重构并划分Sensor模块，剔除未使用的多余传感器代码,同时将I2C硬件通信方式由DMA调整为轮询模式以增加监视器权限监管能力;构建基于共享内存的高效数据交互区域,实现跨虚拟机的传感器数据通信与交互;去除冗余的扩展模块检测代码,成功在初赛基础上移植并适配Lua运行环境,完成Lua脚本的全面测试验证。 |
| 赵薪宇 | 移除正点原子飞控项目中的Bootloader依赖;简化I2C总线驱动策略以优化资源管理;划分 Remote 模块,完成测试攻击代码的编写与注入,并开展系统级测试与跟踪分析;通过透明中断机制有效解决高频UART中断问题;设计读写双缓冲方案以保障读写异步操作的稳定性;在初赛虚拟机基础上成功移植光流虚拟机,实时采集光流数据,为提升飞行稳定性提供有力支持。 |
<br>
<br>

##  项目测试

---

-   虚拟化平台在STM32F411CEU6芯片基本性能测试

![项目基础性能测试](Document/test.png)

- 虚拟化飞控系统内存消耗统计

|             | Flash（512KB） | SRAM（128KB） |
| ----------- | -------------- | ------------- |
| Kernel      | 48KB           | 16KB          |
| Monitor     | 18KB           | 10KB          |
| Flight      | 28KB           | 9KB           |
| Sensor      | 16KB           | 14KB          |
| Remote      | 12KB           | 8KB           |
| OpticalFlow | 32KB           | 8KB           |
| Lua         | 172KB          | 48KB          |
| Total       | 314KB          | 113KB         |

-   其他关于无人机飞行测试，遭受攻击快速恢复测试，光流模块定点飞行测试，虚拟机间通信测试以及Lua脚本运行测试可查看 **[飞行测试视频](https://pan.baidu.com/s/1NADmcFigpwyrFCz9EBL66A?pwd=1234)** 与 **[项目文档](https://pan.baidu.com/s/1GgnmqQsMOfy0e0_sxfsGzA?pwd=1234)** 有关测试内容
<br>
<br>

## 项目文件说明

---

### 开发及完善代码概要

- **Stable_Fly**

<details>
<summary>点击查看 Stable_Fly 相关文件项目树</summary>

```
└─Stable_Fly(虚拟化飞控系统整体工程项目)
   ├─ Kernel(微内核操作系统内核)
   │  ├─ Source(内核启动与与虚拟化适配)
   │  │  ├─ rme_boot.c
   │  │  └─ rme_hook.c
   │  └─ Include(内核头文件与芯片平台配置)
   │     ├─ rme_boot.h
   │     ├─ rme_platform.h
   │     ├─ rme_platform_a7m_conf.h
   │     └─ rme_platform_stm32f411ce.h
   ├─ Monitor(虚拟机监视器,管理各个虚拟机)
   │  ├─ Source(虚拟机启动与虚拟化接口)
   │  │  ├─ rvm_boot.c
   │  │  └─ rvm_hook.c
   │  └─ Include(监视器头文件与平台配置)
   │     ├─ rvm_boot.h
   │     ├─ rvm_platform.h
   │     ├─ rvm_platform_a7m_conf.h
   │     └─ rvm_platform_stm32f411ce.h
   ├─ Flight(飞行控制虚拟机,负责无人机姿态与控制逻辑)
   │  ├─ Source(飞行控制核心代码与进程描述文件)
   │  │  ├─ flight.c
   │  │  ├─ prc_flight.c
   │  │  ├─ prc_flight_desc.c
   │  │  └─ rmp_hook.c
   │  └─ Include(飞行控制头文件与平台配置)
   │     ├─ flight.h
   │     ├─ prc_flight.h
   │     ├─ rmp_platform.h
   │     ├─ rmp_platform_a7m_rvm_conf.h
   │     ├─ rmp_platform_stm32f411ce_rvm.h
   │     ├─ rvm_platform.h
   │     └─ rvm_platform_a7m_conf.h
   ├─ Sensor(传感器虚拟机,负责处理加速度等传感器数据)
   │  ├─ Source(传感器采集核心代码与进程描述文件)
   │  │  ├─ i2c.c
   │  │  ├─ prc_sensor.c
   │  │  ├─ prc_sensor_desc.c
   │  │  ├─ rmp_hook.c
   │  │  └─ sensor.c
   │  └─ Include(传感器头文件与平台配置)
   │     ├─ i2c.h
   │     ├─ prc_sensor.h
   │     ├─ rmp_platform.h
   │     ├─ rmp_platform_a7m_rvm_conf.h
   │     ├─ rmp_platform_stm32f411ce_rvm.h
   │     ├─ rvm_platform.h
   │     ├─ rvm_platform_a7m_conf.h
   │     └─ sensor.h
   ├─ Remote(遥控通信虚拟机,负责遥控数据接受与解析)
   │  ├─ Source(遥控通信与进程描述)
   │  │  ├─ prc_remote.c
   │  │  ├─ prc_remote_desc.c
   │  │  ├─ remote.c
   │  │  └─ rmp_hook.c
   │  └─ Include(遥控通信头文件与平台配置)
   │     ├─ prc_remote.h
   │     ├─ remote.h
   │     ├─ rmp_platform.h
   │     ├─ rmp_platform_a7m_rvm_conf.h
   │     ├─ rmp_platform_stm32f411ce_rvm.h
   │     ├─ rvm_platform.h
   │     └─ rvm_platform_a7m_conf.h
   ├─ Opflow(光流虚拟机,负责光流传感器通信)
   │  ├─ Source(光流与VL53L1x传感器驱动)
   │  │  ├─ optical_flow.c
   │  │  ├─ prc_opflow.c
   │  │  ├─ prc_opflow_desc.c
   │  │  ├─ rmp_hook.c
   │  │  ├─ vl53l1x.c
   │  │  ├─ vl53l1_api.c
   │  │  ├─ vl53l1_api_calibration.c
   │  │  ├─ vl53l1_api_core.c
   │  │  ├─ vl53l1_api_debug.c
   │  │  ├─ vl53l1_api_preset_modes.c
   │  │  ├─ vl53l1_api_strings.c
   │  │  ├─ vl53l1_core.c
   │  │  ├─ vl53l1_core_support.c
   │  │  ├─ vl53l1_error_strings.c
   │  │  ├─ vl53l1_register_funcs.c
   │  │  ├─ vl53l1_silicon_core.c
   │  │  ├─ vl53l1_wait.c
   │  │  └─ vl53lxx_i2c.c
   │  └─ Include(光流传感器与VL53L1 API头文件)
   │     ├─ optical_flow.h
   │     ├─ prc_opflow.h
   │     ├─ rmp_platform.h
   │     ├─ rmp_platform_a7m_rvm_conf.h
   │     ├─ rmp_platform_stm32f411ce_rvm.h
   │     ├─ rvm_platform.h
   │     ├─ rvm_platform_a7m_conf.h
   │     ├─ vl53l1x.h
   │     ├─ vl53l1_api.h
   │     ├─ vl53l1_api_calibration.h
   │     ├─ vl53l1_api_core.h
   │     ├─ vl53l1_api_debug.h
   │     ├─ vl53l1_api_preset_modes.h
   │     ├─ vl53l1_api_strings.h
   │     ├─ vl53l1_core.h
   │     ├─ vl53l1_core_support.h
   │     ├─ vl53l1_def.h
   │     ├─ vl53l1_error_codes.h
   │     ├─ vl53l1_error_exceptions.h
   │     ├─ vl53l1_error_strings.h
   │     ├─ vl53l1_ll_def.h
   │     ├─ vl53l1_ll_device.h
   │     ├─ vl53l1_nvm_map.h
   │     ├─ vl53l1_platform.h
   │     ├─ vl53l1_platform_log.h
   │     ├─ vl53l1_platform_user_config.h
   │     ├─ vl53l1_platform_user_data.h
   │     ├─ vl53l1_platform_user_defines.h
   │     ├─ vl53l1_preset_setup.h
   │     ├─ vl53l1_register_funcs.h
   │     ├─ vl53l1_register_map.h
   │     ├─ vl53l1_register_settings.h
   │     ├─ vl53l1_register_structs.h
   │     ├─ vl53l1_silicon_core.h
   │     ├─ vl53l1_tuning_parm_defaults.h
   │     ├─ vl53l1_types.h
   │     ├─ vl53l1_wait.h
   │     └─ vl53lxx_i2c.h
   ├─ Lua(Lua引擎移植虚拟机)
   │  ├─ Source(Lua核心源码及适配扩展)
   │  │  ├─ lapi.c
   │  │  ├─ lauxlib.c
   │  │  ├─ lbaselib.c
   │  │  ├─ lcode.c
   │  │  ├─ lcorolib.c
   │  │  ├─ lctype.c
   │  │  ├─ ldblib.c
   │  │  ├─ ldebug.c
   │  │  ├─ ldo.c
   │  │  ├─ ldump.c
   │  │  ├─ lfunc.c
   │  │  ├─ lgc.c
   │  │  ├─ linit.c
   │  │  ├─ liolib.c
   │  │  ├─ llex.c
   │  │  ├─ lmathlib.c
   │  │  ├─ lmem.c
   │  │  ├─ loadlib.c
   │  │  ├─ lobject.c
   │  │  ├─ lopcodes.c
   │  │  ├─ loslib.c
   │  │  ├─ lparser.c
   │  │  ├─ lstate.c
   │  │  ├─ lstring.c
   │  │  ├─ lstrlib.c
   │  │  ├─ ltable.c
   │  │  ├─ ltablib.c
   │  │  ├─ ltm.c
   │  │  ├─ lua.c
   │  │  ├─ lundump.c
   │  │  ├─ lutf8lib.c
   │  │  ├─ lvm.c
   │  │  ├─ lzio.c
   │  │  ├─ prc_lua.c
   │  │  ├─ prc_lua_desc.c
   │  │  └─ rmp_hook.c
   │  └─ Include(Lua引擎的完整头文件集合)
   │     ├─ lapi.h
   │     ├─ lauxlib.h
   │     ├─ lcode.h
   │     ├─ lctype.h
   │     ├─ ldebug.h
   │     ├─ ldo.h
   │     ├─ lfunc.h
   │     ├─ lgc.h
   │     ├─ ljumptab.h
   │     ├─ llex.h
   │     ├─ llimits.h
   │     ├─ lmem.h
   │     ├─ lobject.h
   │     ├─ lopcodes.h
   │     ├─ lopnames.h
   │     ├─ lparser.h
   │     ├─ lprefix.h
   │     ├─ lstate.h
   │     ├─ lstring.h
   │     ├─ ltable.h
   │     ├─ ltm.h
   │     ├─ lua.h
   │     ├─ lua.hpp
   │     ├─ luaconf.h
   │     ├─ lualib.h
   │     ├─ lundump.h
   │     ├─ lvm.h
   │     ├─ lzio.h
   │     ├─ prc_lua.h
   │     ├─ rmp_platform.h
   │     ├─ rmp_platform_a7m_rvm_conf.h(虚拟化平台适配层配置文件)
   │     ├─ rmp_platform_stm32f411ce_rvm.h
   │     ├─ rvm_platform.h
   │     └─ rvm_platform_a7m_conf.h
   └─ Common(STM32f4xx芯片相关头文件)
      ├─ arm_common_tables.h
      ├─ arm_math.h
      ├─ core_cm4.h
      ├─ core_cm4_simd.h
      ├─ core_cmFunc.h
      ├─ core_cmInstr.h
      ├─ stm32f4xx.h
      ├─ stm32f4xx_conf.h
      └─ system_stm32f4xx.h
```
</details> 
<br>

- **rvmdk-stm32f411ceu6fc.rvp**
```
 └─rvmdk-stm32f411ceu6fc.rvp(飞控项目内核，虚拟机监视器及各虚拟机描述文件)
```
<br>

- **M7M01_Eukaron/Include/Platform/A7M/Chip/STM32F411CE/rme_platform_stm32f411ce.h**

<details>
<summary>点击查看微内核通用操作系统RME文件项目树</summary>

```
└─ M7M01_Eukaron(微内核通用实时操作系统)
   ├─ Source
   │  ├─ Platform
   │  │  └─ A7M
   │  │     ├─ rme_platform_a7m.c
   │  │     ├─ rme_platform_a7m_armcc.s
   │  │     └─ rme_platform_a7m_gcc.s
   │  └─ Kernel
   │     └─ rme_kernel.c
   └─ Include
      ├─ rme.h
      ├─ Platform
      │  └─ A7M
      │     ├─ rme_platform_a7m.h
      │     └─ Chip
      │        └─ STM32F411CE(移植到目标芯片平台配置文件)
      │           └─ rme_platform_stm32f411ce.h
      └─ Kernel
         └─ rme_kernel.h
```

</details>
<br>

- **M7M02_Ammonite/Include/Platform/A7M/Chip/STM32F411CE/rvm_platform_stm32f411ce.h**
- **M7M02_Ammonite/Include/Platform/A7M/Chip/STM32F411CE/rvm_platform_stm32f411ce.rvc**

<details>
<summary>点击查看虚拟机监视器RVM文件项目树</summary>

```
└─ M7M02_Ammonite(虚拟机监视器)
   ├─ Source()
   │  ├─ Virtlib
   │  │  └─ rvm_virtlib.c
   │  ├─ Syslib
   │  │  └─ rvm_syslib.c
   │  ├─ Platform
   │  │  └─ A7M
   │  │     ├─ rvm_platform_a7m.c
   │  │     ├─ rvm_platform_a7m_armcc.s
   │  │     └─ rvm_platform_a7m_gcc.s
   │  └─ Monitor
   │     └─ rvm_monitor.c
   └─ Include()
      ├─ rvm.h
      ├─ Virtlib
      │  └─ rvm_virtlib.h
      ├─ Syslib
      │  └─ rvm_syslib.h
      ├─ Platform
      │  └─ A7M
      │     ├─ rvm_platform_a7m.h
      │     ├─ rvm_platform_a7m.rva
      │     └─ Chip
      │        └─ STM32F411CE(适配移植目标芯片平台编写设置文件)
      │           ├─ rvm_platform_stm32f411ce.h
      │           └─ rvm_platform_stm32f411ce.rvc
      └─ Monitor
         └─ rvm_monitor.h
```

</details>
<br>

- **M5P01_Prokaron/Include/Test/Chip/rmp_test_stm32f411ce_rvm.h**

<details>
<summary>点击查看轻量级实时操作系统RMP项目树</summary>

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

</details>
<br>

### 完整项目树

<details>
<summary>点击查看完整项目树</summary>

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
</details>
<br>
<br>

## 比赛收获

---

本项目聚焦于在资源受限的嵌入式平台上构建虚拟化操作系统栈，并完成了飞控系统的模块化迁移。在目标平台上，我们成功实现了RME微内核、RVM虚拟机监控器与RMP轻量实时操作系统三层协同架构的移植与部署，完成了从传统宏内核环境到多虚拟机强隔离、模块松耦合架构的转变，构建出了一个兼具 **可信性** 、 **安全性** 与 **高可靠性** 的轻量化虚拟化平台。

在总体实现方面，本项目由三名本科生完成，涵盖了从底层虚拟化框架部署到上层飞控系统模块化适配的完整过程。在硬件层面，我们成功将RME和RVM移植到基于 Cortex‑M4架构的STM32F411CEU6芯片上，实现了在有限资源条件下的自举与运行。在飞控软件层面，我们对开源正点原子飞控代码进行了重构，将原本紧耦合的系统拆分为飞控、传感器、通信和光流四个核心模块，并移植了Lua模块以支持脚本化热更新。最终，这些模块运行在五个独立的虚拟机内，由RMP实时操作系统调度管理，构建出了一个 **隔离性强**、 **稳定可靠** ， **鲁棒性强** 的飞控体系。

在技术实现过程中，我们遇到并解决了多个关键问题。虚拟机之间的通信是核心难点之一。为实现强隔离的同时保持实时性，我们设计并实现了基于虚拟中断、共享内存和消息队列缓冲的通信机制，使数据交互既安全又高效。高频UART中断同样成为影响系统性能的挑战，对此我们采用透明中断机制，将物理中断与调度过程解耦，降低了对CPU的负担。针对飞控中低频DMA带来的潜在风险，我们在保证数据不丢失和负载可控的前提下，将DMA方式改造为轮询方式，从而避免了外设越权访问的问题。同时，我们还修复了RVM监控器中的指针错误，调整了快慢路径切换逻辑，解决了长指令执行过程中的不稳定现象，保证了虚拟化栈的稳定运行。

在比赛过程中，团队不仅掌握了微内核设计、嵌入式虚拟化调度和硬件抽象等基础原理，还在解决通信延迟、内存隔离、全局变量依赖和多模块同步等实际问题时积累了调试和改进的经验。在应对MPU配置冲突、指针错误等具体挑战时，我们提升了系统架构理解与跨模块协作的能力；在不断调试和修改的过程中，我们也锻炼了团队沟通、分工协作和压力应对能力。这些实践能力与经验的提升，是本次比赛给予我们的最大收获。

本次比赛不仅加深了我们对操作系统底层原理与嵌入式系统安全机制的理解，也让我们体会到将学术研究成果转化为实际系统方案时，所需要的工程协同精神与严谨思维方式。未来，我们将继续在系统软件与嵌入式安全方向不断钻研，力求在可信计算与智能终端系统架构等前沿领域探索更多可能性。
<br>
<br>

## 项目开源说明

---

Unlicensed
<br>
<br>

## 参考资料

---

-   [M7M01](https://github.com/EDI-Systems/M7M01_Eukaron)
-   [M7M02](https://github.com/EDI-Systems/M7M02_Ammonite)
-   [M5P01](https://github.com/EDI-Systems/M5P01_Prokaron)
-   [正点原子——`MiniFly`四轴飞行器](http://www.openedv.com/docs/fouraxis-fly/minifly.html)

