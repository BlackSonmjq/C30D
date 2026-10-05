# C30D 底盘侧 IMU 航向控制

航向保持算法已从 ROS 2 上位机移至 C30D 固件。`Balance_task` 以 100 Hz 调用
`HeadingControl_Update()`，使用完成零偏修正后的 `imu.gyro.z` 计算底盘角速度修正量。

## 生效条件

- C30D 已完成上电 10 秒 IMU 零偏采集；采集期间底盘必须保持静止。
- 当前为默认 USART3/ROS 控制模式。
- 车型不是阿克曼车型。
- 未启用 APP、航模遥控器、PS2、CAN、USART1、USART5 或自动回充控制。

切换到其他控制模式时 PID 状态会清零；重新进入 ROS 控制模式时会以当前方向作为新的零航向。

IMU 零偏采集完成、底盘进入可控制状态时，PA8 有源蜂鸣器会短鸣一次，持续约 0.4 秒。

## 参数

参数位于 `heading_control.h`：

- `HEADING_CONTROL_KP`：1.32
- `HEADING_CONTROL_KI`：0.0
- `HEADING_CONTROL_KD`：0.18
- `HEADING_CONTROL_MAX_ANGULAR_VELOCITY`：1.0 rad/s
- `HEADING_CONTROL_MIN_ANGULAR_VELOCITY`：0.08 rad/s
- `HEADING_CONTROL_INTEGRAL_LIMIT`：0.5
- `HEADING_CONTROL_YAW_TOLERANCE`：0.01 rad
- `HEADING_CONTROL_GYRO_TOLERANCE`：0.03 rad/s
- `HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE`：0.002 rad/s

启动阶段使用约 10 秒内的全部静止 IMU 样本计算平均零偏，不再使用单个末次采样。小于 `0.002 rad/s` 的残余静态角速度不参与航向积分，以抑制长时间累计漂移。

微分项直接使用陀螺仪角速度进行阻尼。只有航向误差和旋转角速度同时进入容差范围，控制器才停止输出并清空积分；尚未稳定且 PID 输出小于 `0.08 rad/s` 时，控制器保持同方向的最小角速度，以克服电机和底盘死区。

如需临时关闭底盘侧航向保持，将 `HEADING_CONTROL_ENABLE` 改为 `0` 并重新编译烧录。
