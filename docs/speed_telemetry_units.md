# 速度单位说明

## 当前状态

速度内部存储和遥测都已统一为 **rpm（转/分钟）**。

- `g_pMotor->speed` — 实际转速，单位 rpm
- `g_pMotor->tar_speed` — 目标转速，单位 rpm
- `CMD_SPEED` 遥测 — 直接发送 `g_pMotor->speed`，单位 rpm
- `CMD_SMO_SPEED` 遥测 — 从 PLL 电角速度换算为 rpm
- `sensorlessMechanicalSpeed` — 从 SMO/PLL 换算的机械转速，单位 rpm

速度环 PID 也在 rpm 下运算。

## 代码链路

### CMD_SPEED 遥测

在 `at32f403a_407_int.c` 中直接发送 rpm：

```c
if (speed_Enabled == 1) {
    focData[0] = g_pMotor->speed;
    USART3_SendPacket(CMD_SPEED, &focData[0], 1);
}
```

### CMD_SMO_SPEED 遥测

从 PLL 电角速度（rad/s）换算为 rpm：

```c
focData[0] = g_smoObserver.speed / (float)g_pMotor->pole_pairs
             * 60.0f / FOC_2PI;
USART3_SendPacket(CMD_SMO_SPEED, &focData[0], 1);
```

其中 `g_smoObserver.speed = smo->pll.omega`，是 SMO/PLL 估算的电角速度（rad/s）。

### 有感速度计算（speed_control.c）

编码器机械角度差分后转为 rpm：

```c
angle_diff = (pFOC->mechanicalAngle - mechanicalAngle_last);
// 处理 0/2π 跨周期跳变
pFOC->speed = pFOC->speedDir * angle_diff / dt * 60.0f / FOC_2PI;
```

### 无感速度计算（FOC.c）

PLL 电角速度先换算为机械角速度，再转为 rpm：

```c
pFOC->sensorlessMechanicalSpeed =
    g_smoObserver.speed / (float)pFOC->pole_pairs * pFOC->speedDir
    * 60.0f / FOC_2PI;
```

### 无感开环

`FOC_SENSORLESS_OPEN_LOOP_UQ_TO_SPEED` 单位为 rpm/V，开环角度积分时内部转为电角速度：

```c
openLoopMechSpeed = pFOC->uq * FOC_SENSORLESS_OPEN_LOOP_UQ_TO_SPEED;  // rpm
openLoopElecSpeed = openLoopMechSpeed * FOC_2PI / 60.0f * pole_pairs * speedDir;  // elec rad/s
```

## 对比方法

`CMD_SPEED` 和 `CMD_SMO_SPEED` 都是 rpm，可直接对比。

## 速度环 PID 参数

`speed` 从 rad/s 改为 rpm 后，数值放大约 **9.55 倍**（= 60 / 2π）。

- 同样物理转速，速度误差放大了 9.55 倍
- P 项输出 = kp × 误差，电流给定也随之放大 9.55 倍
- 积分项累积速度加快，更容易饱和 → 转矩剧烈波动 → 电机抖动

因此速度环 kp / ki 需要同比例缩小。推荐参数：

| 参数 | 旧值 (rad/s) | 新值 (rpm) |
|------|-------------|-----------|
| kp   | 0.002       | **0.0002** |
| ki   | 0.1         | **0.01**   |

这个参数组合经实测验证，稳态平稳、动态响应正常。

## 注意事项

- `pole_pairs` 必须配置正确，否则 SMO 速度换算 rpm 会出错。
- `speedDir` 只影响符号，不影响幅值。
- 上位机发送速度目标（`CMD_SETSPEEDTAR`）时，数据单位为 rpm。
- 转速 rad/s ↔ rpm 换算：`rpm = rad_per_s * 60 / (2π)`，`rad_per_s = rpm * (2π) / 60`。
- **开环/电流环模式不受影响**，`speed` 在这两种模式下不参与控制。
