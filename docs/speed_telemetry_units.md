# 速度遥测单位说明

## 原始问题描述

用户现场描述：

> 有感速度环测出来满转速度是 900 多，SMO 测出来的速度是 9000 多，但是感觉转速差不多。感觉 SMO 的可能更准一些，因为确实转得特别快。有感测出来的为什么低很多，是因为单位问题还是什么？

## 结论

这两个速度大概率不是同一个单位含义：

- `CMD_SPEED` 发送的是 `g_pMotor->speed`，在有感模式下是机械角速度，单位是 `rad/s`。
- `CMD_SMO_SPEED` 发送的是 `g_smoObserver.speed`，这是 SMO/PLL 输出的电角速度，单位也是 `rad/s`，但它是电角度的速度。

电角速度和机械角速度关系：

```text
electrical_speed = mechanical_speed * pole_pairs
mechanical_speed = electrical_speed / pole_pairs
```

所以看到：

```text
有感速度 CMD_SPEED     ~= 900
SMO 速度 CMD_SMO_SPEED ~= 9000
```

如果电机极对数 `pole_pairs = 10`：

```text
9000 / 10 = 900
```

这说明两者其实是匹配的，只是一个是机械角速度，一个是电角速度。

## 代码链路

### 有感速度遥测

在 `AT32F403ACCT7_WorkBench/project/src/at32f403a_407_int.c` 中：

```c
if (speed_Enabled == 1) {
    focData[0] = g_pMotor->speed;
    USART3_SendPacket(CMD_SPEED, &focData[0], 1);
}
```

`g_pMotor->speed` 在 `AT32F403ACCT7_WorkBench/project/Hardware/speed_control.c` 中计算。

有感模式下：

```c
angle_diff = (pFOC->mechanicalAngle - mechanicalAngle_last);
pFOC->speed = pFOC->speedDir * angle_diff / dt;
LPF_Speed_Update(pSpeedFilter, pFOC->speed, &(pFOC->speed));
```

这里的 `mechanicalAngle` 是机械角度，所以计算结果是机械角速度，单位是 `rad/s`。

### SMO 速度遥测

在 `AT32F403ACCT7_WorkBench/project/src/at32f403a_407_int.c` 中：

```c
if ((slot == 2U) && (smoSpeed_Enabled == 1)) {
    focData[0] = g_smoObserver.speed;
    USART3_SendPacket(CMD_SMO_SPEED, &focData[0], 1);
}
```

`g_smoObserver.speed` 在 `AT32F403ACCT7_WorkBench/project/Hardware/smo_observer.c` 中来自 PLL：

```c
smo->speed = smo->pll.omega;
```

PLL 追踪的是 SMO 反电动势得到的电角度，所以这里的速度是电角速度，单位是 `rad/s`。

### 无感速度环实际使用的速度

在 `AT32F403ACCT7_WorkBench/project/Hardware/FOC.c` 中，代码已经把 SMO 电角速度换算成机械角速度：

```c
pFOC->sensorlessMechanicalSpeed =
    g_smoObserver.speed / (float)pFOC->pole_pairs * pFOC->speedDir;
```

然后 `CalculateSpeed()` 在无感模式下使用：

```c
pFOC->speed = pFOC->sensorlessMechanicalSpeed;
```

因此，无感速度环实际使用的 `g_pMotor->speed` 应该是机械角速度，而不是原始 `g_smoObserver.speed`。

## rpm 换算

当前速度变量都是 `rad/s`，不是 `rpm`。

机械角速度转 rpm：

```text
rpm = mechanical_rad_s * 60 / (2*pi)
```

例如：

```text
900 rad/s ~= 900 * 60 / 6.28318 ~= 8594 rpm
```

所以有感速度显示 `900` 并不低，它对应的机械转速已经接近 `8600 rpm`。

如果 SMO 显示 `9000`，需要先除以极对数再换 rpm：

```text
mechanical_rad_s = 9000 / pole_pairs
rpm = mechanical_rad_s * 60 / (2*pi)
```

## 对比方法

判断有感速度和 SMO 速度是否一致时，不要直接比较：

```text
CMD_SPEED vs CMD_SMO_SPEED
```

应该比较：

```text
CMD_SPEED ~= CMD_SMO_SPEED / pole_pairs
```

或者比较 rpm：

```text
sensored_rpm = CMD_SPEED * 60 / (2*pi)
smo_rpm      = CMD_SMO_SPEED / pole_pairs * 60 / (2*pi)
```

## 建议改进

为了避免后续混淆，建议新增一个无感机械速度遥测，例如：

```c
focData[0] = g_pMotor->sensorlessMechanicalSpeed;
USART3_SendPacket(CMD_SMO_MECH_SPEED, &focData[0], 1);
```

或者把上位机中 `CMD_SMO_SPEED` 标注为：

```text
SMO electrical speed, rad/s
```

并在界面中额外显示：

```text
SMO mechanical speed = SMO electrical speed / pole_pairs
```

## 注意事项

- 极对数 `pole_pairs` 必须配置正确，否则 SMO 电角速度换算机械速度会错。
- `speedDir` 只影响方向符号，不改变速度单位。
- `CMD_SPEED` 在有感模式下是编码器机械角速度；在无感模式下会被 `sensorlessMechanicalSpeed` 覆盖，仍然应理解为机械角速度。
- `CMD_SMO_SPEED` 当前是原始 SMO/PLL 电角速度，不能直接和机械速度比较。
