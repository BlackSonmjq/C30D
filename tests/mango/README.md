# 芒果击打状态机仿真

在仓库根目录执行：

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py
```

默认使用 `D:/Keil_v5/ARM/ARMCC/bin` 中的 ARMCC 5 工具链；其他安装路径可通过环境变量 `ARMCC_BIN` 指定。构建产物写入本目录 `.build/`，不提交 Git。

测试直接编译生产代码中的协调任务、舵机动作任务 `servo_task.c` 和测距任务，通过 Unicorn 执行 ARM 指令。FreeRTOS 调度、舵机总线回包、步进定时器、GPIO 和传感器底层由桩函数模拟，不连接或驱动实物。

当前原始三段舍机流程的 30 组场景覆盖：

- 四个芒果编号与 left/right、同编号舵机的对应关系，以及忙碌请求拒绝。
- 停止前进时最后一个 STEP 边沿的计数，以及反向等脉冲返程。
- 1～4 号阈值分别为 108、60、45、35 mm；等于对应阈值时继续靠近，降至阈值减 1 mm 时停止并启动舵机。重复执行同一编号，初始距离低于阈值时直接动作且不复用上轮计数。
- K1-K4 all use feedback-confirmed home -> 180 degrees -> home. Tests cover +/-100 arrival boundaries, lost outward/return commands, feedback failure and tick wraparound.
- 首次测距超时、前进中测距失效、脉冲上限、舵机失败和运动超时。
- 四个编号前进达到 26000 个 STEP 脉冲时，立即跳过舵机并反向返回相同脉冲数；即使该轮距离恰好降至对应阈值以下，也优先执行上限返程。
- 舵机无回包、ACK/错误 ID/命令回显/非法数据/未到位回包均不计为完成；转动失败后尝试回零；回零失败、发送失败、总线手动占用与待执行请求取消。
- 返程启动失败或途中停止时锁定故障，系统节拍回绕时正常计时。
- 当前 PA2、PA3、PA6、PC11 四路 XSHUT 互斥切换、全关及过期数据失效。

断言中的中间等待会重复执行，因此检查次数大于场景数量。仿真通过不能证明实物不失步或准确回到机械原点，现场联调应以实际机构表现为准。
