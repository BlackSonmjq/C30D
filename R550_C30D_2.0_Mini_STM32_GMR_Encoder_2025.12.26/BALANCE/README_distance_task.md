# 激光测距任务

`distance_task.c` / `distance_task.h` 已加入 Keil 工程，由 `start_task` 直接调用 `xTaskCreate(distance_task, ...)` 创建。使用 PA8=SCL、PC9=SDA；测距任务启动后默认将 PA3 的 XSHUT 拉高并开启传感器。

任务独占传感器总线，使用 100ms 测量预算的单次测距，每次测量前延时 50ms。通信错误或超时后间隔 1 秒重新初始化。无效测量不作为有效距离发布；超过 500ms 未更新的数据快照返回超时。

OLED 默认显示测距页，保留距离（mm）和基本状态，删除 Raw、SPAD、信号率和初始化阶段诊断。显示函数由现有 show_task 统一调用和刷新；长按 PE0 用户键约 0.6 秒循环切换三页。测距任务不直接操作 OLED 总线。

头文件提供任务入口 distance_task、任务优先级和栈大小，以及数据读取和显示接口。测距轮询和重试参数封装在 .c 文件中。应用读取方式：

```c
#include "distance_task.h"

DistanceTask_Data data;
DistanceTask_GetData(&data);
if (data.valid) {
    /* data.distance_mm 为有效距离，单位 mm */
}
```

结构体保留 distance_mm、valid、status 和 update_tick。无效或过期时 distance_mm 为 0，必须检查 valid，不能将 0 当成有效测距结果。

已删除未使用的 USER/stepper_test.c、.h 及工程引用。驱动保留 NVM SPAD 初始化、配置回读校验和错误处理，移除临时诊断接口。清理后全部 70 个源文件编译、链接通过，未新增硬件实测。
