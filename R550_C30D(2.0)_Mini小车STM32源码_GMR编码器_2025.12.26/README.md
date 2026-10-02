# 推杆（Pushrod）驱动模块

STM32F4 标准外设库实现的 **4 路电动推杆（直线舵机）驱动**。每个推杆使用两个 GPIO 引脚构成双线控制：

| 引脚状态 | 推杆动作 |
|---|---|
| A 高、B 低 | 正转（伸出） |
| A 低、B 高 | 反转（缩回） |
| A 低、B 低 | 停止 |
| A 高、B 高 | 非法状态，代码中永不产生 |

## 文件说明

| 文件 | 说明 |
|---|---|
| `pushrod.h` | 对外接口声明、`Pushrod_Direction` / `Pushrod_Result` 枚举、`PUSHROD_COUNT` 宏 |
| `pushrod.c` | 驱动实现（本文档对应源码） |

## 硬件连接

| 推杆 id（1 起） | A 引脚（正转） | B 引脚（反转） |
|---|---|---|
| 1 | PC0 | PC1 |
| 2 | PC2 | PC3 |
| 3 | PC5 | PC10 |
| 4 | PC11 | PD15 |

> 引脚配置：推挽输出、2 MHz、下拉。初始化时**只配置用到的 8 个引脚**，不影响同端口其他引脚。

## API 说明

### 初始化

```c
void Pushrod_Init(void);
```

上电后调用一次。内部会先关中断，将 4 路全部写为 STOP，再配置 GPIO，最后恢复中断，避免初始化过程产生误动作。

### 控制接口（日常使用）

```c
Pushrod_Result Pushrod_Forward(uint8_t id);   // 第 id 路正转（伸出）
Pushrod_Result Pushrod_Reverse(uint8_t id);   // 第 id 路反转（缩回）
Pushrod_Result Pushrod_Stop(uint8_t id);      // 第 id 路停止
void           Pushrod_StopAll(void);         // 全部停止（急停用）
```

- `id` 取值范围：`1 ~ PUSHROD_COUNT`（即 1~4）。
- 前三个接口是 `Pushrod_Control(id, direction)` 的封装，返回值含义见下表。

### 核心控制（一般不需要直接调用）

```c
Pushrod_Result Pushrod_Control(uint8_t id, Pushrod_Direction direction);
```

内部流程：参数校验 → 初始化检查 → 防反转保护 → 同向去重 → 写引脚（全程关中断，保证原子性）。

### 返回值

| 返回值 | 含义 | 处理建议 |
|---|---|---|
| `PUSHROD_OK` | 指令已生效 | 无需处理 |
| `PUSHROD_INVALID_ARGUMENT` | id 越界或方向参数非法 | 检查调用参数 |
| `PUSHROD_NOT_INITIALIZED` | 未调用 `Pushrod_Init()` | 先初始化再控制 |
| `PUSHROD_REVERSAL_BLOCKED` | 运动中请求反转：已强制停止，但**未执行反转** | 等推杆停稳后重新发反转指令 |

## 快速开始

```c
#include "pushrod.h"

int main(void)
{
    Pushrod_Init();            /* 1. 上电初始化 */

    Pushrod_Forward(1);        /* 2. 推杆 1 正转（伸出） */
    // Delay_ms(1000);

    Pushrod_Stop(1);           /* 3. 停止 */

    Pushrod_Reverse(3);        /* 推杆 3 反转（缩回） */
    Pushrod_StopAll();         /* 4. 需要时全部停止 */
}
```

## 使用注意事项

1. **禁止直接反转**：推杆正在运动时直接请求相反方向，驱动会先强制停止并返回 `PUSHROD_REVERSAL_BLOCKED`。正确流程是 `Stop` → 等待推杆停稳 → 再发送一次反转指令。这是为防止冲击损坏机械结构。
2. **同向指令去重**：同一方向重复调用只会生效一次，不会产生重复触发/抖动。
3. **中断安全**：方向切换全程关中断（`__disable_irq` / `__set_PRIMASK`），可在中断回调中安全调用。
4. **停止后引脚为双低**，推杆处于断电保持状态，不会自行动作。

## 已知问题

- 源码注释存在 GBK/UTF-8 编码混用导致的乱码，不影响编译；在编辑器中按 GB2312 打开可正常显示。
