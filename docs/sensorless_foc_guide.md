# 无感 FOC 学习笔记

本文档基于 `Li_FOC` 项目实际调试过程中遇到的问题和原理分析整理。

---

## 1. 核心概念：有感 vs 无感

### 1.1 有感模式（Sensored）

角度来源：MT6701 磁编码器，直接读机械角度 → 修正电角度。

**开环特点**：给定 uq/ud 电压，角度用编码器，电机转子真实跟随磁场。增大 uq → 电流增大 → 扭矩增大 → 电机加速。

**不需要设置 tar_speed**，因为编码器给了真实角度。

### 1.2 无感模式（Sensorless）

角度来源：SMO 滑模观测器 + PLL 锁相环，从电流和电压中**反推**反电势，再从反电势**提取**转子角度。

**核心区别**：没有编码器告诉你转子在哪，需要靠算法估计。

---

## 2. 无感开环：虚拟角度

### 2.1 为什么需要虚拟角度

电机静止时反电势为 0，SMO 无法估计角度。所以需要一个**虚拟磁场**先把电机拖起来，等反电势足够大后 SMO 才能接管。

### 2.2 虚拟角度如何工作

```
sensorlessOpenLoopAngle += 电气转速 × PWM周期
电气转速 = 机械转速 × 极对数
```

虚拟角度每 PWM 周期递增一点，形成一个旋转的磁场。电机转子被这个旋转磁场拖动。

### 2.3 虚拟转速从哪来

当前实现（`FOC.c:FocContorl`）：

```c
if (uq > EPSILON) {
    openLoopMechSpeed = uq × UQ_TO_SPEED;  // uq 决定虚拟磁场转速
} else {
    openLoopMechSpeed = 0;                 // uq=0 不动
}
```

`tar_speed`（速度环目标）在开环模式下**不生效**，防止切模式时旧值残留导致失步。

### 2.4 UQ_TO_SPEED 增益调多大

| 增益 | uq=1V 对应转速 | 说明 |
|------|---------------|------|
| 20 | ~2100 RPM | 保守，容易拖起来 |
| 50 | ~5200 RPM | 需要 uq 够大 |
| 100 | ~10500 RPM | 1V 推不动，必然失步 |

**为什么增益不能太大？**

电机加速需要扭矩，扭矩来自电流，电流取决于电压。如果虚拟磁场转太快：

```
虚拟转速 300 rad/s → 反电势 ≈ Ke × 300 × 11 ≈ 33V（假设 Ke=0.01）
uq 只给了 3V → 3V < 33V → 零扭矩 → 转子跟不上 → 失步 → 震动
```

**公式**：电机能达到的最大转速 ≈ uq / (反电势常数 × 极对数)

### 2.5 为什么设了 uq，电机转速不如预期

因为你设的是**电压**（V），不是**转速指令**。电机实际转速 = 虚拟磁场转速（物理能达到的前提下）。

虚拟磁场按 `uq × gain` 转，但电机能不能跟上取决于：
- uq 够不够大（克服反电势）
- 负载重不重
- 电机自身参数

**解决**：加 uq。1V 推不动就 5V、8V。

---

## 3. 电流环：闭环控制

### 3.1 电流环和开环的区别

| | 开环（Open Loop） | 电流环（Current Loop） |
|---|---|---|
| uq 来源 | 手动设定 `CMD_SETUQ` | PI 自动计算 `iqPID.out` |
| 电流控制 | 无，电流 = (uq-反电势)/Rs | PI 自动调 uq 保持 Iq=目标 |
| 转速 | 虚拟磁场决定 | 取决于负载和电压裕量 |

### 3.2 电流环模式下为什么设 uq 无效

```c
// FOC.c line 489
if (ctrolmode != FOC_OPEN_LOOP) {
    pFOC->uq = pFOC->iqPID.out;   // ← PI 输出覆盖了你的 uq！
}
```

电流环、速度环、位置环都会用 PI 输出覆盖 uq。想直接设 uq 必须用开环模式。

### 3.3 电流环工作原理

```
CMD_SETIQ = 1.0         →  目标 Iq = 1A
PI: bias = 1.0 - 实际Iq →  uq = PI(bias)
实际Iq 偏低 → uq 自动增加 → 电流增大 → 跟踪目标
```

PI 参数（`freertos_app.c` 初始化）：
- kp = 0.0005, ki = 0.5
- outMax = 12.0V（Uq 电压上限）

---

## 4. 电流限制机制

### 4.1 三层限制结构

| 层级 | 参数 | 设置命令 | 说明 |
|------|------|---------|------|
| 第一层 | `tariq` | `CMD_SETIQ` (0x18) | 电流环=目标；速度/位置环=上限 |
| 第二层 | `tariqMax` | `CMD_SETIQMAX` (0x5A) | 绝对安全上限，所有模式生效 |
| 第三层 | `iqPID.outMax` | `CMD_SETIQPIDOUT` (0x43) | Uq 电压限幅（V），**不是**电流限制 |

### 4.2 各模式下 tariq 的角色

| 模式 | tariq 作用 |
|------|-----------|
| 电流环 | **目标电流**：`iqPID.tar = tariq` |
| 速度环 | **电流上限**：`iqPID.tar = clamp(speedPID.out, ±tariq)` |
| 位置环 | **电流上限**：同上 |

### 4.3 速度环 + 电流限制的正确用法

```
CMD_SPEED_LOOP
CMD_SETIQ = 2.0           ← 设电流上限 2A（关键！）
CMD_SETSPEEDTAR = 50      ← 目标速度
```

设了 tariq 后：速度环 P 输出再大，Iq 参考也不会超过 2A。不设 tariq（=0）则不限制。

### 4.4 速度环下 CMD_SETIQ 为何不生效（旧版 bug，已修复）

旧版代码中，速度环模式下 `iqPID.tar` 被 `speedPID.out` 直接覆盖，用户设的 `tariq` 完全被忽略。修复后 `tariq` 在速度/位置环中作为**电流上限**钳位 `speedPID.out`。

---

## 5. SMO 滑模观测器

### 5.1 SMO 如何估计角度

```
电流估计误差 = i_hat - i_actual
滑模注入 z = k_slide × sat(误差/band)
电流观测器: i_hat += Ts/Ls × (u - Rs·i_hat - e_hat - z)
反电势滤波: e_hat += alpha × (z - e_hat)
PLL 锁相: 从 e_hat 中提取角度和速度
```

### 5.2 K_slide 为什么必须大于反电势

**核心约束**：稳态下 `e + z = e_actual`（实际反电势）。z 被限制在 `±k_slide`，e（z 的滤波输出）也被限制在 `±k_slide`。

因此：**SMO 能估计的最大反电势 ≈ 2 × k_slide**

| k_slide | 最大可估计反电势 | 适用场景 |
|---------|----------------|---------|
| 0.2 | 0.4V | 几乎不适用（≈ 几 rad/s） |
| 20 | 40V | 24V 母线电机全速范围 |

### 5.3 K_slide 太小会怎样（经典故障）

**现象**：电机空载加速 → 电流到 ~400mA → 突然抽搐 → 掉速 → 再加速 → 循环

**原因**：
1. 电机加速，反电势超过 0.4V
2. SMO 反电势估计饱和 → 角度错误
3. FOC 用错误角度控制 → 扭矩方向错乱 → 电机抽搐
4. 掉速后反电势降回 0.4V 以下 → SMO 重新锁定
5. 再次加速 → 循环

**解决**：增大 k_slide（24V 母线 ≥ 20）。

---

## 6. 调试速查表

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 开环不转或很慢 | uq 太小 | 加 uq 到 3~8V |
| 开环震动/失步 | UQ_TO_SPEED 增益太大 | 降低增益（≤ 50） |
| 电流环设 uq 无反应 | uq 被 PI 覆盖 | 用开环模式，或设 tariq 用电流环 |
| 无感高速抽搐 | k_slide 太小 | 增大到 ≥ 20 |
| 电流限制不生效 | 设了 CMD_SETIQ 但没生效 | 速度环下 tariq=0 视为不限制，设非零值 |
| tar_speed 在开环下影响电机 | 旧版代码用 tar_speed 驱虚拟角度 | 已修复，开环只看 uq |

---

## 7. 关键参数速查

| 参数 | 默认值 | 位置 | 说明 |
|------|--------|------|------|
| k_slide | 20.0 | foc_config.h | 滑模增益，需 > 最大反电势 |
| e_lpf_alpha | 0.02 | foc_config.h | 反电势 LPF 系数 |
| PLL kp/ki | 800/80000 | foc_config.h | PLL 锁相环增益 |
| UQ_TO_SPEED | 50.0 | foc_config.h | 开环 uq→转速系数 (rad/s)/V |
| current kp/ki | 0.0005/0.5 | freertos_app.c | 电流环 PI |
| speed kp/ki | 0.002/0.1 | freertos_app.c | 速度环 PI |
| iqPID.outMax | 12.0 | freertos_app.c | Uq 电压限幅 |
| speedPID.outMax | 10.0 | freertos_app.c | 速度环输出/Iq参考限幅 |
| tariqMax | 40.0 | FOC.c 初始化 | Iq 绝对安全上限 |

---

## 8. 无感 FOC 启动流程（推荐）

```
1. 有感模式校准零偏：
   CMD_SENSOR_SENSORED → CMD_ZEROCALIBRATIO → 等待完成

2. 有感开环验证：
   CMD_OPEN_LOOP → CMD_SETUQ=2 → 确认电机转、编码器正常

3. 有感电流环验证：
   CMD_CURRENT_LOOP → CMD_SETIQ=1 → 确认电流跟踪正常

4. 切无感测试：
   CMD_SENSOR_SENSORLESS → 开 SMO 遥测对比编码器角度

5. 无感开环启动：
   CMD_OPEN_LOOP → CMD_SETUQ=3 → 观察 SMO 角度是否稳定

6. 无感电流环运行：
   CMD_CURRENT_LOOP → CMD_SETIQ=1 → 无感闭环运行
```
