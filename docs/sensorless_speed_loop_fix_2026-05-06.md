# 无感速度环失控问题排查与修复记录

日期：2026-05-06

## 问题现象

无感速度环调试过程中出现多个连续现象：

1. 有感速度环工作正常。
2. 无感速度环设置目标速度后，电机最初只抖一下，速度控制不了。
3. 无感开环给正 `Uq` 时速度方向为正；无感电流环给正 `Iq` 时，最初速度反馈显示为负。
4. 修正速度方向后，无感速度环开始能影响速度，但低速、小 `Iq` 时可控，高速或稍大 `Iq` 时容易失控。
5. `Iq < 1A` 且目标速度低于约 `1200 rpm` 时可以控制；目标速度大于约 `1200 rpm`，或从 `100 rpm` 直接给到 `2000 rpm` 时，电机会直接满转。

## 链路对比

速度环到电流环的主链路，有感和无感是一样的：

```c
tar_speed
  -> SpeedPIControl()
  -> speedPID.out
  -> iqPID.tar
  -> CurrentPIControlIQ()
  -> iqPID.out
  -> uq
  -> SVPWM
```

关键区别在角度和速度反馈：

### 有感模式

- 控制角度：MT6701 编码器角度 `sensoredCorrectedAngle`
- 速度反馈：机械角度差分
- 正 `Uq`、正 `Iq`、速度反馈方向一致

### 无感模式

- 控制角度：SMO/PLL 输出角度 `g_smoObserver.angle`
- 速度反馈：SMO/PLL 相关速度估算
- 速度环依赖无感速度反馈方向、幅值和采样频率

## 排查过程

### 1. 确认不是速度环链路断开

检查代码确认速度环输出确实进入了电流环：

```c
if (g_pMotor->ctrolmode == FOC_SPEED_LOOP
    || g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
    pFOC->iqPID.tar = pFOC->speedPID.out;
}
```

所以问题不是速度目标没有接到 `Iq`，而是无感反馈或速度环输出异常。

### 2. 确认 SMO/PLL 角度方向

对比：

- 编码器电角度 `electricalAngle`
- PLL 角度 `smoAngle`
- SMO 原始角度 `smoRawAngle`

实测 PLL 角度和实际电角度接近，增长方向一致。因此排除了“SMO 角度整体反向”或“差 `π`”的问题。

### 3. 发现无感速度反馈方向异常

现象：

- 无感开环正 `Uq`，实际转向为正
- 无感电流环正 `Iq`，实际转向也为正
- 但无感速度反馈曾显示为负

因此问题集中到无感速度反馈符号，而不是实际转矩方向。

最初修复方法是：无感速度不直接使用 `g_smoObserver.speed`，而是对 `sensorlessElectricalAngle` 做差分。这样速度反馈方向与 PLL 角度增长方向一致。

### 4. 发现 1200 rpm 附近失控

修正速度方向后，低速可控，但目标速度超过约 `1200 rpm` 就满转。

原因是当时无感速度是在 2ms 速度任务里对电角度差分：

```c
angle_diff = sensorlessElectricalAngle - sensorlessElectricalAngle_last;
speed = speedDir * angle_diff / pole_pairs / dt * 60 / 2π;
```

速度任务周期：

```c
dt = 0.002s
```

对于 `11` 极对电机，电角度差分的无歧义机械转速上限约为：

```text
60 / (2 * 0.002 * 11) = 1363 rpm
```

超过这个速度，电角度每 2ms 变化超过 `π`，差分会混叠，导致速度反馈方向或幅值错误。速度环看到错误反馈后会把输出顶满，表现为直接满转。

这解释了为什么 `1200 rpm` 附近开始失控，也解释了从 `100 rpm` 直接给到 `2000 rpm` 会控制不住。

## 最终修复

### 1. 无感速度差分移到 FOC 高频中断

把无感速度计算从 2ms 速度任务移到 `FocContorl()` 中，使用 `FOC_SMO_TS = 0.00005s` 的高频周期进行电角度差分。

修复后速度混叠上限大幅提高：

```text
60 / (2 * 0.00005 * 11) ≈ 54545 rpm
```

核心代码：

```c
static float sensorlessElectricalAngleLast = 0.0f;
static uint8_t sensorlessSpeedValid = 0U;

pFOC->sensorlessElectricalAngle = g_smoObserver.angle;
if (pFOC->pole_pairs != 0) {
    float sensorlessAngleDiff = pFOC->sensorlessElectricalAngle
                                - sensorlessElectricalAngleLast;
    if (sensorlessAngleDiff > FOC_PI) {
        sensorlessAngleDiff -= FOC_2PI;
    } else if (sensorlessAngleDiff < -FOC_PI) {
        sensorlessAngleDiff += FOC_2PI;
    }

    if (sensorlessSpeedValid == 0U) {
        pFOC->sensorlessMechanicalSpeed = 0.0f;
        sensorlessSpeedValid = 1U;
    } else {
        pFOC->sensorlessMechanicalSpeed =
            pFOC->speedDir * sensorlessAngleDiff / (float)pFOC->pole_pairs
            / FOC_SMO_TS * 60.0f / FOC_2PI;
    }
} else {
    pFOC->sensorlessMechanicalSpeed = 0.0f;
    sensorlessSpeedValid = 0U;
}
sensorlessElectricalAngleLast = pFOC->sensorlessElectricalAngle;
```

### 2. 速度任务只读取无感速度并低通

`CalculateSpeed()` 中，无感模式不再做 2ms 电角度差分，只读取高频侧计算好的 `sensorlessMechanicalSpeed`：

```c
if (FOC_GetSensorMode(pFOC) == FOC_SENSOR_MODE_SENSORLESS) {
    pFOC->speed = pFOC->sensorlessMechanicalSpeed;
    LPF_Speed_Update(pSpeedFilter, pFOC->speed, &(pFOC->speed));
    mechanicalAngle_last = pFOC->mechanicalAngle;
    return;
}
```

### 3. 无感速度环 PI 改为标准增量式

原速度 PI 公式：

```c
out += ki * (bias - lastBias) + kp * bias;
```

这个公式会在每个 2ms 周期重复累加 `kp * bias`。当 `Iq` 限流很小时，输出被压住，看起来能控制；当 `Iq` 放大后，速度环输出容易快速顶满。

无感速度环改为标准增量式：

```c
out += kp * (bias - lastBias) + ki * bias * FOC_SPEED_LOOP_TS;
```

为避免影响已经调好的有感速度环，当前只在无感模式使用新公式，有感模式仍沿用原公式。

### 4. 保留 `SETIQ` 作为速度环限流

速度环下 `SETIQ` 设置的 `tariq` 仍然作为 `speedPID.out` / `iqPID.tar` 的电流限幅。

需要注意：

```c
if (pFOC->tariq > FOC_EPSILON) {
    ...
}
```

因此 `tariq = 0` 表示不启用该限流，不是限流为 0。调无感速度环时建议先设置一个小的非零 `Iq` 限流。

另外，在线调整 `SETIQ` 时，同步夹住 `speedPID.out`，避免速度环内部输出残留超过新的限流。

## 当前结论

这次无感速度环失控的根因不是速度环没有接通，也不是 SMO/PLL 角度整体反向，而是两个问题叠加：

1. 无感速度反馈方向最初取自 `g_smoObserver.speed`，符号与实际控制角度差分不一致。
2. 后续用 2ms 任务差分电角度后，在 11 极对电机上约 `1363 rpm` 就会发生角度差分混叠，导致 `1200 rpm` 以上速度环满转。

最终通过在 FOC 高频中断内计算无感速度，并让 2ms 速度任务只做低通和速度环控制，解决了 `1200 rpm` 附近失控的问题。

## 调试建议

无感速度环继续调参时建议观察：

- `speed`：控制实际使用的速度反馈
- `smoAngle`：PLL 输出角度
- `smoRawAngle`：SMO 原始角度
- `smoSpeed`：PLL 原始速度，仅作参考
- `speedPID.out`：速度环输出，即期望 `Iq`
- `iqPID.tar`：最终进入电流环的 `Iq` 目标
- `iq`：实际 q 轴电流反馈
- `smoDiag`：`pllError` 和 `eMag`

推荐调参顺序：

1. 先设小的非零 `Iq` 限流，例如 `0.1A ~ 0.5A`。
2. 无感电流环验证正 `Iq` 时实际转向和 `speed` 符号一致。
3. 低速目标开始测试，例如 `100 rpm`、`300 rpm`、`600 rpm`。
4. 再测试跨越 `1200 rpm` 到 `2000 rpm`。
5. 无感速度环参数重新从小调起，尤其是改成标准增量式后，`kp/ki` 含义已经和有感旧公式不同。
