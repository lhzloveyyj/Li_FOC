# 无感速度环满转问题排查记录

## 原始问题描述

用户现场描述：

> 无感速度环都没设置速度，速度给 0，一开启就满转，给什么都没效果。但是设置 Iq 好像可以控制。速度环好像没有生效一样，但是有感的就没问题。

## 现象

- 无感模式下进入速度环，目标速度给 `0` 或修改成其它值，电机一开启就满转，速度给定看起来不生效。
- 有感模式速度环工作正常。
- 电流环下直接设置 `Iq` 时，电机电流/力矩能被控制。

## 当前控制链路

外环任务在 `AT32F403ACCT7_WorkBench/project/src/freertos_app.c` 中运行：

```c
CalculateSpeed(g_pMotor, 0.002f, PM1_LPF_Speed);

if (g_pMotor->ctrolmode == FOC_SPEED_LOOP
    || g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
    SpeedPIControl(g_pMotor);
}
```

速度反馈在 `AT32F403ACCT7_WorkBench/project/Hardware/speed_control.c` 中计算：

```c
if (FOC_GetSensorMode(pFOC) == FOC_SENSOR_MODE_SENSORLESS) {
    pFOC->speed = pFOC->sensorlessMechanicalSpeed;
    LPF_Speed_Update(pSpeedFilter, pFOC->speed, &(pFOC->speed));
    mechanicalAngle_last = pFOC->mechanicalAngle;
    return;
}
```

无感速度来自 `AT32F403ACCT7_WorkBench/project/Hardware/FOC.c`：

```c
pFOC->sensorlessMechanicalSpeed =
    g_smoObserver.speed / (float)pFOC->pole_pairs * pFOC->speedDir;
```

速度环输出在 `AT32F403ACCT7_WorkBench/project/Hardware/current_control.c` 中作为 `Iq` 电流环目标：

```c
if (g_pMotor->ctrolmode == FOC_SPEED_LOOP
    || g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
    pFOC->iqPID.tar = pFOC->speedPID.out;
}
```

所以速度环不是没接上，而是 `speedPID.out -> iqPID.tar -> iqPID.out -> uq` 这条链路存在。

## 高风险点

### 1. 切入速度环时输出初始化不安全

在 `AT32F403ACCT7_WorkBench/project/Hardware/protocol.c` 的 `CMD_SPEED_LOOP` 分支中：

```c
g_pMotor->ctrolmode = FOC_SPEED_LOOP;
g_pMotor->speedPID.out = g_pMotor->iq;
g_pMotor->speedPID.lastBias = g_pMotor->tar_speed - g_pMotor->speed;
g_pMotor->iqPID.out = g_pMotor->uq;
g_pMotor->iqPID.lastBias = 0.0f;
```

无感刚切入时，SMO/PLL 角度可能未锁定，Park 变换得到的 `iq` 可能是错误的大值。这里把当前 `iq` 直接作为速度环输出，相当于把一个不可信的测量值变成了电流目标。目标速度即使是 `0`，电流环也可能立即收到一个很大的 `Iq` 目标。

有感模式正常，是因为编码器角度稳定，`iq` 和速度反馈更可信。

### 2. `tariq` 在速度环下不是直接 Iq 目标，而是限幅

在 `CurrentPIControlIQ()` 中，速度环/位置环模式下：

```c
pFOC->iqPID.tar = pFOC->speedPID.out;

if (pFOC->tariq > FOC_EPSILON) {
    if (pFOC->iqPID.tar > pFOC->tariq) {
        pFOC->iqPID.tar = pFOC->tariq;
    }
    if (pFOC->iqPID.tar < -pFOC->tariq) {
        pFOC->iqPID.tar = -pFOC->tariq;
    }
}
```

因此速度环下设置 `Iq` 看起来有效，实际很可能是因为 `tariq` 限制了速度环输出，而不是直接控制了 `Iq` 目标。

需要特别注意：当 `tariq = 0` 时，这段限幅不会生效，速度环输出只受 `speedPID.outMax` 和 `tariqMax` 约束。

### 3. 速度 PI 算法容易累加到满输出

当前 `SpeedPIControl()` 中：

```c
pFOC->speedPID.out += pFOC->speedPID.ki
                      * (pFOC->speedPID.bias - pFOC->speedPID.lastBias)
                      + pFOC->speedPID.kp * pFOC->speedPID.bias;
```

这不像常见的增量 PI。标准增量 PI 通常是：

```c
out += kp * (bias - lastBias) + ki * bias * dt;
```

当前写法中，只要 `bias` 不为 0，`kp * bias` 会每 2ms 反复累加，速度反馈方向或数值异常时很容易顶到 `outMax`。

### 4. 无感低速/静止切闭环时速度反馈本身不可靠

SMO/PLL 依赖反电动势。低速、静止、刚切入时反电动势弱，`g_smoObserver.speed` 可能方向错误、数值跳变或未锁定。速度环拿到错误反馈后，会按错误方向继续加大输出，表现为一开就满转。

## 建议验证项

先打开这些遥测，观察进入无感速度环前后波形：

- `speed`：控制实际使用的速度反馈。
- `smoSpeed`：SMO/PLL 估计电角速度。
- `speedPID.out`：速度环输出，也就是速度环给电流环的目标。
- `iqPID.tar`：最终进入 q 轴电流环的目标。
- `iq`：Park 变换得到的 q 轴电流反馈。
- `smoAngle` / `smoRawAngle` / `electricalAngle`：对比 SMO 角度和编码器电角度。
- `smoDiag`：观察 `pllError` 和 `eMag`，判断 PLL 是否长期偏差大或反电动势幅值过低。

判断方式：

- 目标速度为 `0`，一切入速度环 `speedPID.out` 立刻很大：优先怀疑速度环切换初始化。
- `speedPID.out` 从 0 快速爬到限幅：优先怀疑无感速度反馈方向/数值异常，或速度 PI 算法导致积分/累加过快。
- `smoSpeed` 在静止或低速时明显乱跳：不能直接闭无感速度环，需要先开环拖动或加入锁定判据。

## 建议修改方向

### 1. 切入速度环时不要继承当前 `iq`

最小验证改法：

```c
case CMD_SPEED_LOOP:
    g_pMotor->ctrolmode = FOC_SPEED_LOOP;
    g_pMotor->speedPID.out = 0.0f;
    g_pMotor->speedPID.lastBias = 0.0f;
    g_pMotor->iqPID.out = 0.0f;
    g_pMotor->iqPID.lastBias = 0.0f;
    g_commCmd = CMD_NONE;
    break;
```

如果希望平滑切换，可以在确认角度和电流反馈可靠后，再做 bumpless transfer。无感未锁定前不建议直接用当前 `iq` 做速度环输出初值。

### 2. 速度 PI 改成标准增量式或位置式

最小验证改法：

```c
const float dt = 0.002f;

pFOC->speedPID.out += pFOC->speedPID.kp
                      * (pFOC->speedPID.bias - pFOC->speedPID.lastBias)
                      + pFOC->speedPID.ki * pFOC->speedPID.bias * dt;
```

修改后需要重新整定 `speedPID.kp/ki`，因为参数含义会变化。

### 3. 明确速度环电流限幅语义

建议把 `tariq` 在级联模式下的用途明确为“速度环输出限幅”，并保证进入速度环前设置一个合理值，例如小电机先从 `0.2A ~ 1A` 级别试起，而不是让 `tariq = 0` 导致不限幅。

更清晰的做法是新增单独变量，例如：

```c
float speedIqLimit;
```

避免 `CMD_SETIQ` 在电流环和速度环下语义不同造成误判。

### 4. 无感速度闭环前加入锁定/启动流程

推荐流程：

1. 有感模式下运行，观察 `smoAngle`、`smoRawAngle`、`smoSpeed` 与编码器角度/速度是否一致。
2. 开环或电流环小电流拖动到有足够反电动势的转速。
3. 等 `eMag` 足够大、`pllError` 收敛、`smoSpeed` 方向稳定。
4. 再切到无感速度环。

可以考虑增加无感闭环允许条件：

```c
fabsf(g_smoObserver.speed) > minSpeed
&& g_smoObserver.eMag > minBackEmf
&& fabsf(g_smoObserver.pllError) < maxPllError
```

条件不满足时，不允许直接进入 `FOC_SPEED_LOOP` 的无感闭环，或保持开环拖动。

## 当前结论

这次现象最像是多个问题叠加：

- `CMD_SPEED_LOOP` 把不可信的 `iq` 初始化为 `speedPID.out`，导致一切入就有大电流目标。
- 速度 PI 算法形式容易持续累加到限幅。
- 无感低速/刚切入时 SMO/PLL 速度反馈不可靠，速度为 0 的闭环控制没有可靠反馈基础。

优先验证顺序：

1. 把 `CMD_SPEED_LOOP` 中 `speedPID.out = g_pMotor->iq` 改为清零。
2. 给速度环加小的 `Iq` 限幅，确认目标速度 0 时不会满转。
3. 观察 `smoSpeed` 和 `speedPID.out`，确认是反馈问题还是 PI 算法问题。
4. 再改速度 PI 算法，并重新整定参数。
