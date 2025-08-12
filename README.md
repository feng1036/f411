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
### 初赛阶段
-   项目文档(初赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计(初赛).doc](https://pan.baidu.com/s/1R3IAmBcJhdIH-pikP2Blxw?pwd=1234)
-   进展汇报PPT(初赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT（sduer你要自信队）.pptx](https://pan.baidu.com/s/1ccpIxvr6AX9CqtCojuIUPw?pwd=1234)
-   项目汇报视频(初赛) ：[项目汇报视频](https://pan.baidu.com/s/1Djhv1h4sxJCPpxlVL3C_Og?pwd=1234)
-   无人机飞行测试视频(初赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1k3KI1naTStXgV1jWZ5nHWg?pwd=1234)

### 决赛阶段
-   项目文档(决赛) ：[基于微控制器虚拟化技术的高可信无人机操作系统设计.doc](https://pan.baidu.com/s/1S1WeS5PXJYJ54StVTIeqNA?pwd=1234)
-   进展汇报PPT(决赛) ：[基于微控制器虚拟化技术的高可信操作系统设计进度汇报PPT（sduer你要自信队）.pptx](https://pan.baidu.com/s/1LWISBzckcSDymNrvzHAM3A?pwd=1234)
-   项目汇报视频(决赛) ：[项目汇报视频](https://pan.baidu.com/s/1HLv_fYtJK7v5DyPU6-2yaQ?pwd=1234)
-   无人机飞行测试视频(决赛) ：[无人机飞行测试视频](https://pan.baidu.com/s/1NADmcFigpwyrFCz9EBL66A?pwd=1234)


**注意：本项目的  比赛题目分析和技术调研 、 系统框架设计 、 重要进展 、 系统测试情况 、 遇到的主要问题以及解决方法 、 本项目创新点 等更加详细的说明均在项目文档中。**


## 项目文件说明

### 以下重点目录为移植过程中修改相关代码
- **Stable_Fly**
- **rvmdk-stm32f411ceu6fc.rvp**
- **M7M01_Eukaron/Include/Platform/A7M/Chip/STM32F411CE**
- **M7M02_Ammonite/Include/Platform/A7M/Chip/STM32F411CE**

### 完整项目树

---
```
Code_Library
├─ report.txt
├─ rvmdk-stm32f411ceu6fc.rvp
├─ Stable_Fly
│  ├─ Kernel
│  │  ├─ Source
│  │  │  ├─ rme_boot.c
│  │  │  └─ rme_hook.c
│  │  └─ Include
│  │     ├─ rme_boot.h
│  │     ├─ rme_platform.h
│  │     ├─ rme_platform_a7m_conf.h
│  │     └─ rme_platform_stm32f411ce.h
│  ├─ Monitor
│  │  ├─ Source
│  │  │  ├─ rvm_boot.c
│  │  │  └─ rvm_hook.c
│  │  └─ Include
│  │     ├─ rvm_boot.h
│  │     ├─ rvm_platform.h
│  │     ├─ rvm_platform_a7m_conf.h
│  │     └─ rvm_platform_stm32f411ce.h
│  ├─ Realtime
│  │  ├─ Source
│  │  │  ├─ prc_realtime.c
│  │  │  ├─ prc_realtime_desc.c
│  │  │  └─ prc_realtime_thd_startup.c
│  │  └─ Include
│  │     ├─ prc_realtime.h
│  │     ├─ rvm_platform.h
│  │     └─ rvm_platform_a7m_conf.h
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
│     │        └─ **STM32F411CE**
│     │           └─ rme_platform_stm32f411ce.h
│     └─ Kernel
│        └─ rme_kernel.h
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
│     │        └─ **STM32F411CE**
│     │           ├─ rvm_platform_stm32f411ce.h
│     │           └─ rvm_platform_stm32f411ce.rvc
│     └─ Monitor
│        └─ rvm_monitor.h
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


近年来，无人机在通信、测绘、物流、安防等领域的应用规模持续扩大，飞行控制系统所需处理的功能愈发多样且复杂。在新质生产力发展和国家低空经济政策推动的背景下，无人机正逐步成为支撑产业升级与行业创新的重要基础设施，这对低成本 MCU 平台在高可信性与安全可控性方面提出了更高要求。现有飞控系统多基于裸机或单核实时操作系统运行，虽然具备资源占用低、实现简洁等优势，但普遍缺乏硬件级隔离与模块化防护机制，易受单点故障影响而导致系统整体失效，难以满足多任务协同处理和自主决策等**高可靠性**应用的需求。相比之下，嵌入式虚拟化与微内核架构能够提供更强的**安全隔离与容错能力**，但在**资源受限**的 MCU 平台上，其应用仍面临性能开销大、实现复杂度高等问题。目前，主流无人机飞控系统常运行于`FreeRTOS、RT-Thread`等轻量级 RTOS 环境中，将各类控制与功能模块集中部署在单一运行空间内。在缺乏有效隔离的条件下，任一模块发生异常均可能引发系统全局崩溃，对飞行稳定性与任务安全性造成严重威胁。因此，探索面向 MCU 平台的高可信飞控架构，兼顾**资源效率与安全隔离**，已成为无人机技术发展的关键方向之一。

本项目旨在探索一种**基于微控制器虚拟化技术的无人机高可信操作系统架构**。通过在[RME](https://github.com/EDI-Systems/M7M01_Eukaron)(`RTOS-Mutate-Eukaron`)基础上引入[RVM](https://github.com/EDI-Systems/M7M02_Ammonite)（`Real-time Virtual machine Monitor`）进行虚拟化分区，充分利用**Cortex-M**平台的内存保护单元能力，将飞控系统中的不同功能模块（如光流数据收集、飞行控制等）划分至多个隔离的保护域中，并为每个保护域分配更简单高效易用的全抢占式实时操作系统——[RMP](https://github.com/EDI-Systems/M5P01_Prokaron)。这一设计使系统在面对个别模块异常、崩溃或被攻击时，仍能维持核心飞控功能的稳定运行，实现容错与抗攻击能力的大幅提升。

最终，本项目构建了一个**兼具高可靠性、安全可控性、轻量高效、实时响应能力、生态兼容性及广泛适用性**的无人机操作系统架构模型，为嵌入式领域探索可信执行环境提供了参考，也为未来无人系统的系统级安全设计提供了可行路径。

### 系统架构

![系统整体架构](Document/structure.png)

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
| 7        | 加入更多功能模块进一步扩展功能集成能力，以验证系统的兼容性与扩展性 | :heavy_check_mark: |



## 决赛进展

---

项目进展一：在初赛成果的基础上，对飞控系统的位置估算、姿态控制及飞行 PID 参数等核心控制逻辑进行了优化与调整，进一步完善了飞行控制环路的控制精度与响应能力，进一步提升无人机的飞行稳定性与控制可靠性。

项目进展二：扩展系统兼容性与功能集成能力，在现有飞控基础上新增光流功能模块，将光流数据纳入飞行控制闭环，以提升无人机飞行稳定性。

项目进展三：移植Lua脚本引擎，验证系统对较大型项目的移植适配能力，并完善微控制器级虚拟化机制，构建可靠、可控、可部署的飞控分区调度环境。


## 项目开发进展

---
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


## 项目人员分工

---

| 人员   | 任务分工                                                     |
| ------ | ------------------------------------------------------------ |
| 付翔宇 | 编写STM32F411CEU6平台RME虚拟化启动代码;移除USB、ATKP等非核心扩展模块,重构工程结构;划分Flight模块,编写适配响应Sensor、Remote与光流Opt_Flow的事件处理例程;将光流数据融合应用于姿态控制与位置估算中;通过调试飞控系统的PID参数,提升飞行的稳定性与控制精度,实现了飞控系统的高效移植与性能优化。|
| 冯彬   | 完成RVM启动入口_start()在STM32F411CEU6平台的部署,摒弃对STM32官方FWLIB外设固件库的依赖;重构并划分Sensor模块，剔除未使用的多余传感器代码,同时将I2C硬件通信方式由DMA调整为轮询模式以增加监视器权限监管能力;构建基于共享内存的高效数据交互区域,实现跨虚拟机的传感器数据通信与交互;去除冗余的扩展模块检测代码,成功在初赛基础上移植并适配Lua运行环境,完成Lua脚本的全面测试验证。|
| 赵薪宇 |移除正点原子飞控项目中的Bootloader依赖;简化I2C总线驱动策略以优化资源管理;划分 Remote 模块,完成测试攻击代码的编写与注入,并开展系统级测试与跟踪分析;通过透明中断机制有效解决高频UART中断问题;设计读写双缓冲方案以保障读写异步操作的稳定性;在初赛虚拟机基础上成功移植光流虚拟机,实时采集光流数据,为提升飞行稳定性提供有力支持。|



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













