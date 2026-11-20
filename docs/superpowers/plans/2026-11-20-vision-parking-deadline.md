# Vision Parking Deadline Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan inline. Steps use checkbox syntax for tracking.

**Goal:** 首次停车待击打后5秒未接受K任务则释放视觉控制，重新调节不能刷新期限。

**Architecture:** 复用100 Hz底盘状态机和现有等待时间戳，增加一个停车计时有效标志，在重新对齐和丢失恢复期间统一检查期限。超时复用现有清锁号、清辅助DX和检测封锁路径；原输入可为手柄或导航，不增加拨杆重新使能条件。

**Tech Stack:** STM32 C、FreeRTOS、ARM仿真、EIDE/ARMCC。

**Spec:** 用户2026-11-20回复“进入停车模式后，超过5s未击打，直接释放”；5秒指提交作业，不要求完成机构动作。

## Global Constraints

- 机构请求接受后保留安全返程锁定，不受停车准备5秒期限中断。
- 全局IMU、方向防撞及10秒初次对齐规则保留。
- G=0、模式取消和检测超时应清除本轮停车计时；新一轮停车重新计时。
- 同一超时绿果事件沿用已有封锁，释放后不靠拨杆动作重新允许自动。

## Review Focus

- 等待中编号短暂丢失仍遵守已有2秒失效期限。
- DX或后距重新调节不能清空首次停车计时。
- 期限前允许作业，期限到达先释放，不接受迟到K请求。
- 机构在5秒后仍忙时继续保持安全停车。
- 释放后持续输入能继续，视觉不得在同一封锁事件重抢。

## Task 1: Uninterrupted parking deadline

**Files:** `BALANCE/balance.c`、`USER/vision_strike.h`（均位于固件子目录），`tests/chassis/test_vision_parking_timeout.c`、`tests/chassis/test_vision_align_integration.c`。

**Interfaces:** 保持现有视觉／导航入口；增加内部停车有效标志。期限常量改为5000 ms。

- [x] 新增回归：静态不稳到5秒释放；反复到位／调整不刷新计时；原输入恢复；已接受作业不被中断。
- [x] 运行并确认旧实现因5秒后仍占用而失败。
- [x] 实现统一期限与生命周期重置。
- [x] 运行新测试、所有视觉集成测试并构建固件。
- [x] 更新视觉／VOFA说明，检查差异并提交。

## Verification record

- 原实现新测试在5秒释放断言失败；仅调整常量后，重复调整的5秒释放断言仍失败，确认问题不止等待常量。
- 统一期限后新测试通过，包括恰好5秒恢复条件也不接受迟到任务、无需新拨杆动作恢复原输入、无手柄导航入口继续、机构忙超过5秒仍保持安全锁。
- 22项 `test_vision*.c` 集成回归通过；旧重试测试的7.5秒等待改为新期限内4秒恢复，确认仍可正常提交。
- EIDE ARMCC固件构建通过，输出 `build/FreeRTOS/WHEELTEC.hex`，未烧录、未进行实车验证。
- 本次只修复停车后期限；未改首次停车前对齐规则、全局IMU参数或VOFA通道格式。
