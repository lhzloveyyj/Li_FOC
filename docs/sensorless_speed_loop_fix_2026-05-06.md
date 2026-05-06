# 无感速度环与 I/F 启动问题排查记录

日期：2026-05-06

## 背景

调试时有感速度环已经工作正常，但无感速度环存在明显问题：

- 开启无感速度环后，设置目标速度时电机只抖一下，不能稳定启动。
- 手动拨一下电机后，速度环能接管，说明闭环本身不是完全断开。
- 低速、小 `Iq` 时偶尔可控；目标速度升高或 `Iq` 稍大时容易失控、满转。
- 无感速度反馈曾经抖动很乱，但 `PLL速度` 相对稳定。
- 加入 I/F 启动后，早期版本偶尔能启动，后来加了对齐状态后出现过完全不转的问题。
- 正转 `1000 rpm` 可以正常启动和闭环，反转 `-1000 rpm` 能启动转一下，但切闭环后会卡住。

最终目标是让无感模式能从静止可靠启动，并切入速度闭环。

## 控制链路确认

有感和无感速度环的主链路一致：

```text
tar_speed
  -> SpeedPIControl()
  -> speedPID.out
  -> iqPID.tar
  -> CurrentPIControlIQ()
  -> iqPID.out
  -> uq
  -> SVPWM
```

所以问题不是速度目标没有进入电流环，而是无感模式下的角度、速度反馈、启动状态切换和限流语义需要单独处理。

## 关键问题

### 1. 无感静止时不能直接闭速度环

SMO/PLL 依赖反电势。电机静止或极低速时反电势很小，PLL 角度和速度还没有可靠锁定。

如果这时直接进入速度闭环，电流环会使用不可靠的无感角度做 Park 变换，表现就是电机抖动、不转，手动拨一下后反而能接管。

解决方式：加入无感 I/F 启动，先用虚拟角度和固定电流把电机拖到足够转速，再交给 SMO/PLL。

### 2. 速度反馈不能用低频角度差分

中间曾尝试在 2ms 速度任务中对电角度差分计算无感速度。对于 11 极对电机，2ms 差分的无混叠速度上限约为：

```text
60 / (2 * 0.002 * 11) = 1363 rpm
```

这解释了为什么目标速度在 `1200 rpm` 附近容易失控。

最终改为直接使用 PLL 内部速度：

```c
pFOC->sensorlessMechanicalSpeed =
    g_smoObserver.speed / (float)pFOC->pole_pairs * pFOC->speedDir
    * 60.0f / FOC_2PI;
```

速度任务 `CalculateSpeed()` 只读取 `sensorlessMechanicalSpeed` 并低通，避免 2ms 电角度差分混叠。

### 3. 无感速度 PI 需要避免持续顶满

原速度 PI 公式会在每个周期重复累加 `kp * bias`，无感反馈异常时很容易把输出推到限幅。

无感速度环改为标准增量式 PI：

```c
out += kp * (bias - lastBias) + ki * bias * FOC_SPEED_LOOP_TS;
```

有感速度环已经调好，所以保留原有公式，只对无感速度环使用新公式。

### 4. 速度环下 `Iq` 是限流，不是直接目标

在速度环和位置环模式下，`CMD_SETIQ` 设置的 `tariq` 被用作速度环输出限幅：

```text
speedPID.out -> iqPID.tar
iqPID.tar 被 tariq 限幅
```

因此调无感速度环时，需要先设置一个合理的 `Iq` 限流。当前启动调试使用：

```text
Iq限流 = 1A
目标速度 = 1000 rpm
```

### 5. 反转时 SMO 反电势角度会差 pi

反转问题的最终根因在 SMO 角度提取。永磁同步电机反电势和转子磁链角的关系为：

```text
E = omega_e * psi * [-sin(theta), cos(theta)]
```

正电角速度时，`omega_e > 0`，可以直接用：

```c
rawAngle = atan2(-eAlpha, eBeta);
```

得到磁链角。

但负电角速度时，`omega_e < 0`，反电势矢量整体翻转 `pi`。如果仍然使用同一个公式，SMO/PLL 输出角度会和实际磁链角差 `pi`。

现场现象正好对应这个问题：

```text
目标速度 = -1000
I/F 可以拖动一下
切到 SMO/PLL 闭环后卡住
```

原因是 I/F 阶段使用虚拟角度，方向正确；切闭环后使用差 `pi` 的 PLL 角度，`Iq` 力矩方向变乱，电机立即卡住。

最终修复是在 SMO 内部按当前电角速度方向统一反电势极性：

```c
bemfDir = sign(electrical_speed);
eAlphaFlux = bemfDir * eAlpha;
eBetaFlux  = bemfDir * eBeta;

rawAngle = atan2(-eAlphaFlux, eBetaFlux);
pllErr = -eAlphaFlux * cos(theta_hat)
         -eBetaFlux  * sin(theta_hat);
```

正转时 `bemfDir = +1`，原公式不变；反转时 `bemfDir = -1`，先把反电势矢量翻回来，再计算磁链角和 PLL 误差。

当前实现中，电角速度方向优先来自 I/F 启动速度和目标速度，并结合 `speedDir`：

```text
electrical direction = sign(sensorlessIfSpeed * speedDir)
备用方向 = sign(tar_speed * speedDir)
```

这样反转启动时，SMO/PLL 在切闭环前就按正确方向解释反电势。

## 最终 I/F 启动方案

当前无感速度环启动状态：

```text
FOC_SENSORLESS_IF_OFF
  -> FOC_SENSORLESS_IF_ALIGN
  -> FOC_SENSORLESS_IF_RAMP
  -> FOC_SENSORLESS_IF_DONE
```

### ALIGN 阶段

目的：用固定虚拟电角度和 `Id` 对齐转子，降低随机初始角度导致的起步失败。

行为：

- 固定 `sensorlessIfAngle`
- `Id = sensorlessIfId`
- `Iq = 0`
- 速度 PI 输出清零并暂停

注意：加 `ALIGN` 后必须在主控制链路中持续调用 `FOC_UpdateSensorlessIF()`。曾经出现过只在 `RAMP` 状态调用更新函数，导致状态卡在 `ALIGN`，电机完全不转。

修复后的判断：

```c
if ((pFOC->sensorlessIfState == FOC_SENSORLESS_IF_ALIGN)
    || (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_RAMP)) {
    FOC_UpdateSensorlessIF(pFOC);
}
```

### RAMP 阶段

目的：使用虚拟角度和固定 `Iq` 做 I/F 拖动，让电机产生足够反电势。

行为：

- `sensorlessIfSpeed` 从启动速度开始爬坡
- `sensorlessIfAngle` 按虚拟速度积分
- `Iq = sensorlessIfIq`
- 速度 PI 暂停，避免速度环和 I/F 输出叠加

### DONE 阶段

当 SMO/PLL 满足锁定条件后，切回正常无感速度闭环：

- 控制角度回到 `g_smoObserver.angle`
- 速度反馈使用 PLL 速度
- `speedPID.out` 初始化为 I/F 启动电流，减小切换冲击

切换条件：

```text
I/F 虚拟速度 >= FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM
g_smoObserver.eMag > FOC_SENSORLESS_IF_MIN_EMAG
abs(g_smoObserver.pllError) < FOC_SENSORLESS_IF_MAX_PLL_ERROR
连续满足 FOC_SENSORLESS_IF_LOCK_COUNT 个 PWM 周期
```

## 当前有效参数

位置：`AT32F403ACCT7_WorkBench/project/Hardware/foc_config.h`

```c
#define FOC_SENSORLESS_IF_ALIGN_ID             0.50f
#define FOC_SENSORLESS_IF_ALIGN_COUNT          6000U
#define FOC_SENSORLESS_IF_START_IQ             1.00f
#define FOC_SENSORLESS_IF_START_SPEED_RPM      20.0f
#define FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM   1000.0f
#define FOC_SENSORLESS_IF_RAMP_RPM_PER_S       1500.0f
#define FOC_SENSORLESS_IF_START_MAX_SPEED_RPM  150.0f
#define FOC_SENSORLESS_IF_MIN_TARGET_RPM       10.0f
#define FOC_SENSORLESS_IF_MIN_EMAG             0.15f
#define FOC_SENSORLESS_IF_MAX_PLL_ERROR        0.35f
#define FOC_SENSORLESS_IF_LOCK_COUNT           200U
```

当前实测结论：

- `Iq = 1A`
- `目标速度 = 1000 rpm`
- `I/F 爬坡 = 1500 rpm/s`

可以正常启动并切入速度闭环。

反转修复后，`目标速度 = -1000 rpm` 也可以正常启动并切入速度闭环。

## 调试步骤

推荐启动顺序：

```text
切无感
设置 Iq限流 = 1
设置目标速度 = 1000
打开速度环
```

反转启动顺序：

```text
切无感
设置 Iq限流 = 1
设置目标速度 = -1000
打开速度环
```

注意：速度环下 `Iq限流 = 1` 表示允许 `speedPID.out` 在 `-1A ~ +1A` 范围内变化，不需要为了反转手动设置 `Iq = -1`。目标速度为负时，速度环和 I/F 启动会自动给负向 `Iq`。

优先观察变量：

1. `速度反馈`：应平滑，方向应与实际转向一致。
2. `PLL速度`：应比角度差分速度更稳定。
3. `Iq/Id`：`ALIGN` 阶段应接近 `Id=0.5, Iq=0`；`RAMP` 阶段应接近 `Iq=1, Id=0`。
4. `速度环输出`：`ALIGN/RAMP` 阶段应被 I/F 启动电流接管，切闭环后才由速度 PI 正常调节。
5. `PLL误差/eMag`：正常转起来后，`eMag` 应有明显幅值，`pllError` 应较小。

## 后续调参方向

如果启动失败或抖动：

- 增大 `FOC_SENSORLESS_IF_START_IQ`，例如从 `1.0A` 试到 `1.2A`。
- 增大 `FOC_SENSORLESS_IF_ALIGN_ID`，例如从 `0.5A` 试到 `0.7A`。
- 降低 `FOC_SENSORLESS_IF_RAMP_RPM_PER_S`，例如从 `1500 rpm/s` 降到 `800 rpm/s`。

如果启动可靠但太慢：

- 增大 `FOC_SENSORLESS_IF_RAMP_RPM_PER_S`。
- 当前已经从 `300 rpm/s -> 800 rpm/s -> 1500 rpm/s`，实测 `1500 rpm/s` 可用。

如果启动能拖起来，但切闭环瞬间失败：

- 提高 `FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM`。
- 增大 `FOC_SENSORLESS_IF_LOCK_COUNT`。
- 检查 `pllError` 是否收敛、`eMag` 是否足够。

如果速度环能控制但速度波形乱：

- 优先看 `PLL速度` 和 `速度反馈` 是否一致。
- 不要再回到 2ms 电角度差分速度，容易在高极对电机上混叠。

如果正转正常、反转切闭环卡住：

- 优先检查 SMO 反电势角度是否处理了电角速度符号。
- 观察 `PLL速度` 和 `速度反馈` 在反转时是否为负。
- 如果 I/F 能反转拖动，但 `DONE` 后卡住，重点怀疑 SMO/PLL 角度差 `pi` 或 PLL 误差符号。

## 本次修复涉及的主要文件

- `AT32F403ACCT7_WorkBench/project/Hardware/FOC.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/FOC.h`
- `AT32F403ACCT7_WorkBench/project/Hardware/current_control.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/speed_control.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/foc_config.h`
- `AT32F403ACCT7_WorkBench/project/Hardware/protocol.c`
