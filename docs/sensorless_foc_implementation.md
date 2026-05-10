# 无感 FOC 实现详解：I/F 启动、SMO、PLL、电流环、速度环

本文档按当前 `Li_FOC` 工程源码整理，目标是把“原理”和“代码”一一对应起来。重点覆盖：

- 无感启动为什么必须有 I/F 阶段
- `FocContorl()` 中每个 PWM 周期做了什么
- SMO 如何从电压、电流估算反电势
- PLL 如何从反电势提取角度和速度
- 速度环、电流环如何级联到最终 PWM
- 低速、零速、堵转为什么特殊处理

对应源码主要在：

- `AT32F403ACCT7_WorkBench/project/Hardware/FOC.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/FOC.h`
- `AT32F403ACCT7_WorkBench/project/Hardware/smo_observer.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/smo_observer.h`
- `AT32F403ACCT7_WorkBench/project/Hardware/pll_observer.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/current_control.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/speed_control.c`
- `AT32F403ACCT7_WorkBench/project/src/freertos_app.c`
- `AT32F403ACCT7_WorkBench/project/src/at32f403a_407_int.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/foc_config.h`

---

## 1. 总体控制链路

### 1.1 高频中断链路

FOC 主循环在 ADC1 抢占转换完成中断里执行：

```text
TIM1 触发 ADC 采样
  -> ADC1_2_IRQHandler()
  -> 读取三相电流 ADC + 母线电压 ADC
  -> FocContorl(g_pMotor, PSVpwm)
```

代码入口：

- `AT32F403ACCT7_WorkBench/project/src/at32f403a_407_int.c`
- `ADC1_2_IRQHandler()`
- `FocContorl()`

`FocContorl()` 是电流环和无感观测器的实时核心，周期由 PWM/ADC 触发频率决定。当前配置中 `FOC_SMO_TS = 0.00005f`，即 20kHz、50us。

每个 PWM 周期的大流程：

```text
1. 选择角度来源
   有感: MT6701 编码器 -> correctedAngle
   无感: I/F 虚拟角度 或 SMO/PLL 角度 -> correctedAngle

2. 读取 ADC 并换算三相电流 Ia/Ib/Ic

3. 电流重构
   Ia 硬件有问题时，用 Ib/Ic 重构缺失相

4. Clarke 变换
   Ia/Ib/Ic -> iAlpha/iBeta

5. Park 变换
   iAlpha/iBeta + correctedAngle -> id/iq

6. Id/Iq 低通滤波

7. 电流 PI
   idPID -> ud
   iqPID -> uq

8. 逆 Park
   uq/ud + correctedAngle -> uAlpha/uBeta

9. SVPWM
   uAlpha/uBeta -> TMR1 CH1/CH2/CH3 PWM

10. 无感模式下更新 SMO
    uAlpha/uBeta + iAlpha/iBeta -> g_smoObserver.angle/speed
```

### 1.2 低频速度环链路

速度环不在 ADC 中断里跑，而是在 FreeRTOS 的 `control_task_func()` 中每 2ms 执行一次：

```text
control_task_func()
  -> CalculateSpeed(g_pMotor, 0.002f, PM1_LPF_Speed)
  -> SpeedPIControl(g_pMotor)
```

对应源码：

- `AT32F403ACCT7_WorkBench/project/src/freertos_app.c`
- `AT32F403ACCT7_WorkBench/project/Hardware/speed_control.c`

速度环输出不是电压，而是 `Iq` 电流参考：

```text
tar_speed - speed
  -> SpeedPIControl()
  -> speedPID.out
  -> CurrentPIControlIQ()
  -> iqPID.tar
  -> iqPID.out
  -> uq
```

所以速度闭环本质是：

```text
速度 PI 给 Iq 目标
电流 PI 给 Uq 电压
SVPWM 把 Uq 输出到三相桥
```

---

## 2. 关键状态和变量

核心状态结构体是 `FocState`，定义在 `FOC.h`。

### 2.1 角度相关

```c
float mechanicalAngle;
float correctedAngle;
float sensoredMechanicalAngle;
float sensoredCorrectedAngle;
float sensorlessElectricalAngle;
float sensorlessMechanicalSpeed;
float sensorlessOpenLoopAngle;
uint8_t sensorMode;
```

含义：

| 变量 | 单位 | 作用 |
|------|------|------|
| `sensoredMechanicalAngle` | rad | MT6701 读到的机械角度 |
| `sensoredCorrectedAngle` | rad | 编码器机械角度换算出的修正电角度 |
| `sensorlessElectricalAngle` | rad | SMO/PLL 输出电角度 |
| `sensorlessMechanicalSpeed` | rpm | SMO/PLL 电角速度换算出的机械转速 |
| `sensorlessOpenLoopAngle` | rad | 无感开环/低速冻结使用的虚拟电角度 |
| `correctedAngle` | rad | 当前 FOC 真正使用的电角度 |
| `sensorMode` | enum | 有感或无感 |

最重要的是 `correctedAngle`。Park、逆 Park 都使用它。只要这个角度错，电流环即使 PI 算法正确，也会把电压打到错误方向。

### 2.2 控制模式

`CtrolMode_TypeDef` 定义：

```c
FOC_OPEN_LOOP     = 0x01
FPC_CURRENT_LOOP  = 0x02
FOC_SPEED_LOOP    = 0x03
FOC_POSITION_LOOP = 0x04
```

各模式关系：

| 模式 | 外环 | 电流环 | `uq` 来源 |
|------|------|--------|-----------|
| 开环 | 无 | 不闭环 | 用户直接给 `uq` |
| 电流环 | 无 | Id/Iq PI | `iqPID.out` |
| 速度环 | 速度 PI | Id/Iq PI | `iqPID.out` |
| 位置环 | 位置 PD + 速度 PI | Id/Iq PI | `iqPID.out` |

在 `FocContorl()` 中有一句关键逻辑：

```c
if (g_pMotor->ctrolmode != FOC_OPEN_LOOP) {
    pFOC->uq = pFOC->iqPID.out;
    pFOC->ud = pFOC->idPID.out;
}
```

因此只要不是开环，手动设置的 `uq/ud` 都会被 PI 输出覆盖。

### 2.3 无感 I/F 状态

`FocSensorlessIFState_TypeDef` 定义：

```c
FOC_SENSORLESS_IF_OFF
FOC_SENSORLESS_IF_ALIGN
FOC_SENSORLESS_IF_RAMP
FOC_SENSORLESS_IF_DONE
```

I/F 是 `Current / Frequency` 启动，也就是“给定电流 + 给定虚拟频率”的开环拖动。它不是 SMO 的一部分，而是为了让 SMO 有可观测的反电势。

状态变量：

| 变量 | 单位 | 作用 |
|------|------|------|
| `sensorlessIfState` | enum | I/F 当前阶段 |
| `sensorlessIfAngle` | rad | I/F 虚拟电角度 |
| `sensorlessIfSpeed` | rpm | I/F 虚拟机械转速 |
| `sensorlessIfIq` | A | RAMP 阶段启动 q 轴电流 |
| `sensorlessIfId` | A | ALIGN 阶段对齐 d 轴电流 |
| `sensorlessIfAlignCount` | PWM 周期数 | ALIGN 计数 |
| `sensorlessIfLockCount` | PWM 周期数 | 连续满足 SMO 交接条件的计数 |

---

## 3. 启动流程：从静止到 SMO 闭环

### 3.1 为什么静止时不能直接用 SMO

永磁同步电机的反电势与转速成正比：

```text
E ≈ omega_e * psi
```

静止时：

```text
omega_e = 0
E ≈ 0
```

SMO/PLL 靠反电势估算角度。低速或零速时，反电势接近 0，观测器看到的主要是电流采样噪声、PWM 纹波、零偏误差，因此角度不可靠。

更准确地说，SMO 并不是“直接看见转子磁极”，它看见的是由转子磁链运动产生的反电势。转子不动时，磁链虽然还在，但磁链没有切割定子绕组，电压方程里就没有足够明显的速度电势项。此时从电压、电流反推出来的 `eAlpha/eBeta` 幅值非常小，方向等价于在噪声里取 `atan2`，角度会随机跳。

可以把无感低速问题理解成一个信噪比问题：

```text
反电势信号 E ∝ 转速
采样零偏 / PWM 纹波 / ADC 量化噪声 / 参数误差 ≈ 基本不随转速同比例下降

高速: E 远大于噪声 -> 角度方向清楚
低速: E 接近噪声 -> 角度方向随机
零速: E 接近 0    -> 没有可观测角度
```

所以无感速度环不能从 0 速直接闭环。当前实现使用 I/F 启动：

```text
OFF
  -> ALIGN: 固定虚拟角度 + Id 对齐
  -> RAMP: 虚拟角度旋转 + Iq 拖动
  -> DONE: SMO/PLL 锁定后接管
```

### 3.2 I/F 启动的基本原理

I/F 是 `Current / Frequency` 的缩写，可以直译为“电流/频率启动”：

```text
Current: 给定一个可控的启动电流，决定能产生多大电磁转矩
Frequency: 给定一个虚拟电角频率，决定定子磁场按多快速度旋转
```

它的思想很朴素：既然低速时 SMO 看不见转子角度，那就先不用 SMO，而是由控制器自己生成一个虚拟角度 `theta_if`。FOC 电流环以这个虚拟角度为坐标系，输出三相电流，形成一个可控旋转磁场。转子永磁体会被这个旋转磁场“拖着走”。等转子被拖到足够速度，反电势变大，SMO/PLL 有了可信角度，再切换到真正无感闭环。

可以把 I/F 看成无感 FOC 的“助跑”：

```text
零速: 没有反电势，不能观测角度
I/F: 用虚拟角度强行建立旋转磁场
加速: 转子跟随虚拟磁场转动，反电势逐渐变大
交接: SMO/PLL 角度和速度稳定后，虚拟角度退出
闭环: correctedAngle 改用 SMO/PLL 角度
```

### 3.3 I/F 为什么需要“电流”和“频率”两个量

只给频率不够。假设只让虚拟角度快速旋转，但不给足够 `Iq`，定子磁场很弱，转子没有足够转矩跟上，立刻失步。

只给电流也不够。假设给固定电流但角度不转，转子最多被吸到某个固定位置，不会持续加速，也不会产生越来越大的反电势。

所以 I/F 同时规定：

```text
Iq_if     -> 启动力矩大小
omega_if  -> 虚拟磁场转速
```

当前代码里对应：

```text
sensorlessIfIq     -> I/F 阶段 q 轴电流目标
sensorlessIfSpeed  -> I/F 虚拟机械转速 rpm
sensorlessIfAngle  -> I/F 虚拟电角度
```

力矩近似关系：

```text
T_e ≈ 1.5 * pole_pairs * psi_f * Iq
```

因此 `FOC_SENSORLESS_IF_START_IQ` 越大，启动转矩越大，越容易带负载启动；但电流越大，发热和冲击也越大。如果启动时只是抖、不跟随，常见方向是适当增大 `START_IQ` 或降低 RAMP 斜率，而不是盲目提高速度目标。

频率对应虚拟角度积分：

```text
omega_if_e = speed_if_rpm * 2pi / 60 * pole_pairs * speedDir
theta_if[k+1] = theta_if[k] + omega_if_e * Ts
```

这里的 `theta_if` 是电角度，不是机械角度。机械转一圈，电角度会转 `pole_pairs` 圈。所以极对数越大，同样机械 rpm 下电角频率越高，I/F 和 SMO 的计算压力、角度误差敏感性也越高。

### 3.4 I/F 启动的稳定条件

I/F 是开环拖动，控制器不知道真实转子是否真的跟上。它能稳定启动，需要满足一个相位条件：真实转子磁链角 `theta` 不能落后虚拟角度 `theta_if` 太多。

定义负载角：

```text
delta = theta_if - theta
```

`delta` 可以理解为定子旋转磁场相对转子的领先角。启动时需要一点领先角来产生加速转矩，但领先角太大就会失步：

```text
delta 较小:
  有效 q 轴电流充足，转矩方向正确

delta 接近 90 电角度:
  转矩接近最大，但系统已经很紧张

delta 超过稳定范围:
  转子跟不上虚拟磁场
  电流分解到错误轴上
  有效转矩下降或变成脉动
  表现为抖动、反抽、停转
```

因此 I/F 参数本质是在平衡三件事：

```text
启动电流 Iq_if 要足够大，能提供加速转矩
速度斜坡 d(rpm)/dt 要足够慢，转子能跟上
交接速度要足够高，SMO/PLL 已经看得清反电势
```

如果启动失败，现象和原因通常对应如下：

| 现象 | 更可能的原因 | 参数方向 |
|------|--------------|----------|
| 原地抖，不明显加速 | 启动电流不足或 ALIGN 没对齐 | 增大 `START_IQ` / `ALIGN_ID` |
| 刚动一下就失步 | RAMP 太快，负载角过大 | 降低 `RAMP_RPM_PER_S` |
| RAMP 能转，交接后卡住 | SMO/PLL 未锁定或角度方向错 | 提高交接条件、查方向 |
| 启动冲击很大 | 电流太大或 ALIGN 太猛 | 降低 `START_IQ` / `ALIGN_ID` |

### 3.5 触发 I/F 的条件

在 `FocContorl()` 中，无感速度环满足以下条件时启动 I/F：

```text
sensorMode == SENSORLESS
ctrolmode == FOC_SPEED_LOOP
abs(tar_speed) > FOC_SENSORLESS_IF_MIN_TARGET_RPM
sensorlessIfState == OFF
abs(speed) < FOC_SENSORLESS_IF_START_MAX_SPEED_RPM
```

配置来自 `foc_config.h`：

```c
#define FOC_SENSORLESS_IF_MIN_TARGET_RPM       10.0f
#define FOC_SENSORLESS_IF_START_MAX_SPEED_RPM  150.0f
```

也就是说：

- 目标速度太小，不启动 I/F
- 实际速度已经高于 150rpm，不重新启动 I/F
- 只有无感速度环才自动 I/F

启动前会清理状态：

```text
SMO_Reset()
uq = 0
ud = 0
speedPID.out = 0
iqPID.out = 0
idPID.out = 0
lastBias 全部清零
```

原因是：低速、堵转、失步时 SMO 角度和 PI 积分器可能已经污染。直接在脏状态上启动，会造成错误角度下的大电流冲击。

### 3.6 ALIGN：固定角度 + Id 对齐

入口函数：

```c
FOC_StartSensorlessIF()
```

进入 ALIGN 时：

```text
sensorlessIfState = ALIGN
sensorlessIfAngle = 当前 correctedAngle
sensorlessIfSpeed = 0
sensorlessIfIq = 0
sensorlessIfId = FOC_SENSORLESS_IF_ALIGN_ID
```

配置：

```c
#define FOC_SENSORLESS_IF_ALIGN_ID     0.50f
#define FOC_SENSORLESS_IF_ALIGN_COUNT  6000U
```

在 `FOC_UpdateSensorlessIF()` 中，ALIGN 阶段每个 PWM 周期都把控制角度固定为：

```c
pFOC->correctedAngle = pFOC->sensorlessIfAngle;
```

同时 `CurrentPIControlID()` 会把 Id 目标改成：

```c
if (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_ALIGN) {
    pFOC->idPID.tar = pFOC->sensorlessIfId;
}
```

物理意义：

```text
固定电角度 + 正 Id
  -> 定子产生一个固定方向磁场
  -> 转子磁极被吸到这个方向
  -> 后续 RAMP 从已知相位开始拖动
```

默认 6000 个 PWM 周期，若 20kHz：

```text
6000 * 50us = 300ms
```

从 dq 坐标看，`d` 轴定义为转子磁链方向，`q` 轴定义为超前 `d` 轴 90 电角度的力矩方向。但 ALIGN 时还不知道真实转子在哪，所以代码先人为指定一个虚拟电角度 `sensorlessIfAngle`，再在这个虚拟 `d` 轴上注入 `Id`。这会在定子上产生一个固定方向的磁场。

转子永磁体会被这个定子磁场吸过去，最终真实转子磁链方向和虚拟 `d` 轴方向基本重合。这样 RAMP 开始时，虚拟角度和真实转子角度之间不会差太多，开环拖动才不容易一开始就失步。

这里用 `Id` 而不是 `Iq`，是因为 ALIGN 的目标不是产生持续转矩，而是“定向吸住”。如果一开始就给 `Iq`，在未知转子角度下可能产生正转、反转或抖动。

从能量角度看，ALIGN 阶段是在给转子建立一个势能最低点。定子磁场固定不动，转子磁链会转到和定子磁场同向的位置，这个位置电磁能量最低、稳定性最好。等转子被吸到这个位置后，控制器就获得了一个“人为定义的起始角度”。

ALIGN 的几个细节：

```text
ALIGN 时间太短:
  转子还没完全吸到虚拟 d 轴
  RAMP 一开始角度误差大，容易反抽或失步

ALIGN 电流太小:
  磁场吸力不够，静摩擦/齿槽转矩下对不齐

ALIGN 电流太大:
  机械冲击明显，电机发热，可能听到咔的一声
```

当前实现中，ALIGN 只在 `FOC_SENSORLESS_IF_ALIGN` 阶段让 `idPID.tar = sensorlessIfId`，进入 RAMP 后 `sensorlessIfId = 0`，回到常规的 `Id ≈ 0` 控制。

### 3.7 RAMP：虚拟角度旋转 + Iq 拖动

ALIGN 结束后进入 RAMP：

```text
sensorlessIfSpeed = sign(tar_speed) * START_SPEED_RPM
sensorlessIfIq = sign(tar_speed) * START_IQ
sensorlessIfId = 0
```

配置：

```c
#define FOC_SENSORLESS_IF_START_IQ             1.00f
#define FOC_SENSORLESS_IF_START_SPEED_RPM      20.0f
#define FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM   1000.0f
#define FOC_SENSORLESS_IF_RAMP_RPM_PER_S       1500.0f
```

每个 PWM 周期速度按斜坡增加：

```c
rampStep = FOC_SENSORLESS_IF_RAMP_RPM_PER_S * FOC_SMO_TS;
speedAbs += rampStep;
```

20kHz 下：

```text
rampStep = 1500 rpm/s * 0.00005s = 0.075 rpm/周期
```

虚拟机械速度换算成电角速度：

```c
elecSpeed = sensorlessIfSpeed * 2pi / 60
            * pole_pairs * speedDir;
```

然后积分出虚拟电角度：

```c
sensorlessIfAngle =
    NormalizeAngle(sensorlessIfAngle + elecSpeed * FOC_SMO_TS);
```

此时 `correctedAngle` 使用虚拟角度，电流环跟随这个虚拟坐标系：

```text
correctedAngle = sensorlessIfAngle
iqPID.tar = sensorlessIfIq
idPID.tar = 0
```

物理意义：

```text
给一个旋转的 q 轴电流
  -> 定子磁场按虚拟角度旋转
  -> 转子被开环拖动
  -> 转速升高后产生反电势
  -> SMO/PLL 逐渐有可靠输入
```

RAMP 的本质是人为规定一个“假想转子角度”：

```text
theta_if[k+1] = theta_if[k] + omega_if * Ts
```

电流环并不知道它是假角度，它只负责在这个坐标系下把 `Iq` 调到目标值。因此逆 Park 后，三相电压会形成一个旋转磁场。只要 RAMP 斜率不过快、`Iq` 足够克服负载和摩擦，真实转子就会被这个旋转磁场拖着走。

为什么 RAMP 速度不能一下给很高：

```text
虚拟磁场加速太快
  -> 转子机械惯量跟不上
  -> 真实转子角度落后虚拟角度越来越多
  -> q 轴电流不再是纯力矩电流
  -> 有效转矩下降甚至反向
  -> 失步、抖动、卡住
```

所以 `FOC_SENSORLESS_IF_RAMP_RPM_PER_S` 是一个很关键的机械参数，和电机转动惯量、负载、启动电流有关。启动电流越大、负载越轻，允许的 RAMP 斜率越大；反之应降低斜率。

I/F RAMP 和普通开环电压拖动有一个重要区别：这里不是简单给 `Uq`，而是通过电流环给 `Iq`。也就是说，启动阶段仍然利用了电流闭环：

```text
sensorlessIfIq
  -> iqPID.tar
  -> CurrentPIControlIQ()
  -> iqPID.out
  -> uq
```

这样即使电机反电势逐渐升高，电流环也会自动提高或降低 `Uq` 来尽量维持启动电流。相比纯电压开环，I/F 电流启动对负载和母线波动更稳。

但是它依然是“角度开环”：电流环只能保证在虚拟坐标系里 `Iq` 接近目标，不能保证真实转子角度一定跟上。因此 RAMP 的成败最终取决于机械系统能否跟上虚拟频率。

### 3.8 RAMP 到 DONE 的交接条件

RAMP 阶段不会按时间盲目切闭环，而是检查 SMO/PLL 是否可靠。当前条件在 `FOC_UpdateSensorlessIF()` 中：

```text
1. I/F 虚拟速度 >= FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM
2. PLL 机械速度方向与目标方向一致，且 > 交接速度的一半
3. PLL 机械速度绝对值 > 交接速度的一半
4. g_smoObserver.eMag > FOC_SENSORLESS_IF_MIN_EMAG
5. abs(g_smoObserver.pllError) < FOC_SENSORLESS_IF_MAX_PLL_ERROR
6. 上述条件连续满足 FOC_SENSORLESS_IF_LOCK_COUNT 个 PWM 周期
```

配置：

```c
#define FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM   1000.0f
#define FOC_SENSORLESS_IF_MIN_EMAG             0.15f
#define FOC_SENSORLESS_IF_MAX_PLL_ERROR        0.35f
#define FOC_SENSORLESS_IF_LOCK_COUNT           200U
```

交接成功后：

```text
sensorlessIfState = DONE
speedPID.out = sensorlessIfIq
speedPID.lastBias = tar_speed - speed
iqPID.lastBias = 0
```

这样速度环接管时，初始输出等于 I/F 启动电流，减少从开环拖动切到闭环速度环时的电流跳变。

从原理上看，交接条件必须同时确认三件事：

```text
1. 转速足够高:
   反电势幅值和角度信噪比够用

2. SMO/PLL 方向正确:
   PLL 速度方向要和目标方向一致

3. PLL 已经锁相:
   pllError 小，并且连续稳定一段时间
```

如果只看虚拟速度达到 1000rpm 就交接，可能转子其实已经失步，SMO 看到的是错误反电势或噪声；如果只看 `eMag`，可能反电势幅值够了但 PLL 角度还没追上；如果只看瞬时 `pllError`，可能只是某一拍碰巧小。所以代码用了速度方向、反电势幅值、PLL 误差、连续计数多个条件一起判断。

交接瞬间最危险的是角度跳变：

```text
交接前 correctedAngle = sensorlessIfAngle
交接后 correctedAngle = g_smoObserver.angle
```

如果这两个角度相差很大，同样的 `Iq` 参考在真实坐标里会突然变成错误方向，电机就会“咯噔”一下甚至卡住。因此调试时要重点比较 I/F 末期的虚拟角度、SMO/PLL 角度和编码器电角度。没有编码器时，则看交接后 `iq/id` 是否突变、`pllError` 是否飙升、速度是否掉下去。

### 3.9 RAMP 超时重试

如果电机被卡住，虚拟速度会爬到交接速度，但真实转子没跟上，SMO 条件永远不满足。当前 `FocContorl()` 中有 RAMP 超时保护：

```text
RAMP 超过 40000 个 PWM 周期 -> FOC_ResetSensorlessIF()
```

20kHz 下：

```text
40000 * 50us = 2s
```

复位到 OFF 后，如果目标速度仍然有效，下个周期会重新 ALIGN -> RAMP。

### 3.10 低速、零速、堵转保护

无感在低速区没有可靠角度。当前实现做了两层保护。

第一层是限流：

```text
无感 + 速度环 + 非 ALIGN/RAMP
且 tar_speed 接近 0 或 speed 低于 I/F 启动阈值
  -> tariq = 0.05A
  -> speedPID.outMax = 0.05A
```

第二层是冻结角度：

```text
低速/零速/堵转
  -> correctedAngle = sensorlessOpenLoopAngle
```

这样做的原因：

- SMO 角度低速时主要是噪声
- 如果继续用噪声角度做 Park/逆 Park，即使电流很小也会产生交变转矩
- 冻结一个固定角度至少不会高速乱跳
- 同时把电流限制到很小，避免明显抖动

注意：如果目标是“完全不输出一点力”，理论上应让速度环输出、电流目标和 PWM 输出都归零或关断桥臂。当前代码的低速保护是 `0.05A` 微小限流，不是严格零力矩。

---

## 4. SMO 滑模观测器

SMO 实现在 `smo_observer.c`，全局对象是：

```c
SmoObserver g_smoObserver;
```

初始化在 `comm_task_func()`：

```c
const SmoObserverConfig smoConfig = {
    .rs = g_pMotor->rs,
    .ls = (g_pMotor->lq + g_pMotor->ld) * 0.5f,
    .ts = FOC_SMO_TS,
    .k_slide = FOC_SMO_K_SLIDE,
    .e_lpf_alpha = FOC_SMO_E_LPF_ALPHA,
    .pll = {
        .kp = FOC_SMO_PLL_KP,
        .ki = FOC_SMO_PLL_KI,
        .ts = FOC_SMO_TS,
    },
};
SMO_Init(&g_smoObserver, &smoConfig);
```

### 4.1 SMO 输入输出

每个 PWM 周期，无感模式下在 `FocContorl()` 末尾调用：

```c
SMO_Update(&g_smoObserver,
           pFOC->uAlpha, pFOC->uBeta,
           pFOC->iAlpha, pFOC->iBeta);
```

输入：

| 输入 | 单位 | 来源 |
|------|------|------|
| `uAlpha/uBeta` | V | 电流 PI 输出经逆 Park 后得到的电压指令 |
| `iAlpha/iBeta` | A | ADC 电流经 Clarke 后得到的实际电流 |

输出：

| 输出 | 单位 | 用途 |
|------|------|------|
| `g_smoObserver.angle` | rad | 无感电角度，供 FOC 使用 |
| `g_smoObserver.speed` | rad/s | 电角速度，换算为 rpm 供速度环使用 |
| `eAlpha/eBeta` | V | 估算反电势 |
| `eMag` | V | 反电势幅值，判断是否可观测 |
| `pllError` | normalized | PLL 锁相误差，判断是否锁定 |
| `rawAngle` | rad | 反电势直接 atan2 角度，诊断用 |
| `rawSpeed` | rad/s | rawAngle 差分速度，PLL 高速捕获前馈 |

### 4.2 为什么反电势里包含电角度

永磁同步电机的转子可以理解为一个旋转磁链矢量。设转子磁链幅值为 `psi_f`，电角度为 `theta`，那么在 αβ 静止坐标系里，转子磁链可以写成：

```text
psi_alpha = psi_f * cos(theta)
psi_beta  = psi_f * sin(theta)
```

绕组中的反电势来自磁链变化率，按常用电机方程符号可写成：

```text
e_alpha = d(psi_alpha)/dt
e_beta  = d(psi_beta)/dt
```

对上面的磁链求导：

```text
e_alpha = d/dt [psi_f * cos(theta)]
        = -psi_f * sin(theta) * dtheta/dt
        = -omega_e * psi_f * sin(theta)

e_beta = d/dt [psi_f * sin(theta)]
       =  psi_f * cos(theta) * dtheta/dt
       =  omega_e * psi_f * cos(theta)
```

所以反电势矢量是：

```text
E = [e_alpha, e_beta]
  = omega_e * psi_f * [-sin(theta), cos(theta)]
```

这个式子有三个非常重要的结论：

```text
1. 反电势幅值 |E| = |omega_e| * psi_f
   -> 转速越高，反电势越强；零速反电势为 0。

2. 正转时，反电势矢量比转子磁链矢量超前 90 电角度。
   磁链: [cos(theta), sin(theta)]
   反电势: [-sin(theta), cos(theta)]

3. 只要知道反电势方向，并知道旋转方向，就能反推出 theta。
```

正电角速度时：

```text
e_alpha = -E_mag * sin(theta)
e_beta  =  E_mag * cos(theta)

theta = atan2(-e_alpha, e_beta)
```

这就是当前代码里：

```c
smo->rawAngle = NormalizeAngle(atan2f(-eAlphaFlux, eBetaFlux));
```

为什么反电势能解析电角度：不是因为反电势“等于角度”，而是反电势方向和转子磁链方向存在固定 90 度几何关系。SMO 估计出反电势矢量以后，`atan2(-eAlpha, eBeta)` 就把这个 90 度关系反解回磁链角度，也就是 FOC 需要的电角度。

这里要注意“电角度”和“机械角度”的关系：

```text
theta_e = pole_pairs * theta_m * dir
```

无感 FOC 控制电流时需要的是电角度 `theta_e`，因为三相电流产生的是电角度空间里的旋转磁场。速度环给用户看的通常是机械转速 rpm，因此代码里还要把 PLL 的电角速度除以极对数再换成 rpm。

### 4.3 电机电压方程

在 αβ 静止坐标系中，简化 PMSM 电流模型：

```text
u_alpha = Rs * i_alpha + Ls * di_alpha/dt + e_alpha
u_beta  = Rs * i_beta  + Ls * di_beta/dt  + e_beta
```

整理：

```text
di/dt = (u - Rs*i - e) / Ls
```

SMO 不知道真实 `e`，于是构造一个电流观测器：

```text
d(i_hat)/dt = (u - Rs*i_hat - e_hat - z) / Ls
```

其中：

- `i_hat` 是估算电流
- `e_hat` 是估算反电势
- `z` 是滑模注入量，用来逼迫 `i_hat` 跟上真实电流 `i`

更完整地看，真实电机和观测器分别是：

```text
真实电机:
di/dt = (u - Rs*i - e) / Ls

观测器:
d(i_hat)/dt = (u - Rs*i_hat - e_hat - z) / Ls
```

把两式相减，定义电流估计误差：

```text
err = i_hat - i
```

可以得到误差动态：

```text
d(err)/dt
  = d(i_hat)/dt - di/dt
  = [-Rs*(i_hat - i) - (e_hat - e) - z] / Ls
  = [-Rs*err + (e - e_hat) - z] / Ls
```

如果没有 `z`，观测器只能靠电机参数 `Rs/Ls` 和输入电压去猜电流，参数一错、电压模型一错，`i_hat` 就会漂。滑模注入 `z` 的作用就是根据 `err` 的符号主动修正观测器，使 `err` 被压回 0 附近。

当误差被滑模作用压到很小并在 0 附近高频切换时，可以近似认为：

```text
err ≈ 0
d(err)/dt ≈ 0
```

代入误差动态：

```text
0 ≈ (e - e_hat - z) / Ls
z ≈ e - e_hat
```

而代码又用低通滤波让 `e_hat` 跟随 `z`：

```text
e_hat += alpha * (z - e_hat)
```

不同 SMO 写法的符号约定会不完全一样，但核心思想一致：滑模注入量中包含了“为了让电流模型贴住真实电流所必须补偿的未知电压项”。这个未知电压项主要就是反电势。因此，把注入量的高频切换滤掉，就可以得到反电势估计。

直观类比：

```text
已知输入电压 u
已知电机 Rs/Ls
测得真实电流 i
观测器预测电流 i_hat

如果预测电流和真实电流不一致，
说明电压方程里有一个未知项没解释掉。
SMO 用 z 把这个未知项补进去，
最终 z 的低频部分就是这个未知项，也就是反电势。
```

这也是为什么 SMO 对参数敏感：

- `Rs` 错，会把电阻压降误认为反电势
- `Ls` 错，会把电流动态误差放大或变慢
- `uAlpha/uBeta` 不准，会把电压指令误差混进反电势
- 电流零偏没校好，会让观测器长期注入错误补偿

### 4.4 电流误差和滑模注入

源码：

```c
errAlpha = smo->iAlphaHat - iAlpha;
errBeta = smo->iBetaHat - iBeta;
```

滑模注入：

```c
smo->zAlpha = smo->cfg.k_slide * smo_sat(errAlpha / band);
smo->zBeta  = smo->cfg.k_slide * smo_sat(errBeta  / band);
```

`smo_sat()` 是连续饱和函数：

```text
sat(x) = clamp(x, -1, 1)
```

所以：

```text
z = K_slide * sat((i_hat - i) / band)
```

`band` 来自：

```c
#define FOC_SMO_CURRENT_ERR_BAND 10.0f
```

如果用硬 `sign()`，注入量会在 `+K_slide/-K_slide` 之间硬切换，抖振明显。当前用饱和带把误差较小时的切换变成线性过渡，牺牲一点锐度换平滑性。

滑模观测器名字里的“滑模”，可以理解为让系统状态被强制吸到一个滑模面上。这里的滑模面就是：

```text
errAlpha = iAlphaHat - iAlpha = 0
errBeta  = iBetaHat  - iBeta  = 0
```

当估计电流高于真实电流时，`err > 0`，注入量按一个方向修正；当估计电流低于真实电流时，`err < 0`，注入量反向修正。只要 `K_slide` 足够覆盖未知反电势和模型误差，误差就会被压在 0 附近。误差在 0 附近来回切换时，注入量的平均值就携带了反电势信息。

`FOC_SMO_CURRENT_ERR_BAND` 决定“靠近 0 的区域有多软”：

```text
band 小:
  err 稍微一点就饱和到 ±K_slide
  收敛强，但开关噪声和抖振大

band 大:
  err 在较大范围内线性变化
  平滑，但滑模强度变弱，反电势估计可能钝
```

### 4.5 电流观测器积分

源码：

```c
invLs = 1.0f / smo->cfg.ls;
smo->iAlphaHat += smo->cfg.ts * invLs
                  * (uAlpha - smo->cfg.rs * smo->iAlphaHat
                     - smo->eAlpha - smo->zAlpha);
smo->iBetaHat  += smo->cfg.ts * invLs
                  * (uBeta  - smo->cfg.rs * smo->iBetaHat
                     - smo->eBeta  - smo->zBeta);
```

对应离散公式：

```text
i_hat[k+1] = i_hat[k]
           + Ts/Ls * (u[k] - Rs*i_hat[k] - e_hat[k] - z[k])
```

这里 `Ts` 必须与实际 PWM/ADC 中断周期一致。当前配置：

```c
#define FOC_SMO_TS 0.00005f
```

如果实际中断频率不是 20kHz，SMO 和 PLL 的带宽、速度估算都会偏。

### 4.6 反电势低通滤波

源码：

```c
smo->eAlpha += smo->cfg.e_lpf_alpha * (smo->zAlpha - smo->eAlpha);
smo->eBeta  += smo->cfg.e_lpf_alpha * (smo->zBeta  - smo->eBeta);
```

公式：

```text
e_hat[k] = e_hat[k-1] + alpha * (z[k] - e_hat[k-1])
```

直观理解：

- `z` 是滑模注入量，包含大量高频开关成分
- `e_hat` 是 `z` 的低通滤波结果，被当作反电势估计

截止频率近似：

```text
fc ≈ alpha / (2*pi*Ts)
```

当前：

```text
alpha = 0.02
Ts = 0.00005
fc ≈ 63.7 Hz
```

调参含义：

| `e_lpf_alpha` | 效果 |
|---------------|------|
| 太小 | 反电势很平滑，但相位滞后大，高速角度落后 |
| 太大 | 响应快，但噪声大，PLL 速度抖 |

### 4.7 K_slide 的物理约束

`K_slide` 是滑模注入最大幅值：

```text
z ∈ [-K_slide, +K_slide]
```

稳态下，SMO 需要用 `z` 和滤波后的 `e_hat` 去覆盖真实反电势。经验上：

```text
可估计最大反电势 ≈ 2 * K_slide
```

所以 `K_slide` 必须大于电机最高运行速度下的反电势幅值。当前配置：

```c
#define FOC_SMO_K_SLIDE 20.0f
```

24V 母线下设置到 20V 级别是为了避免高速反电势超过观测器能力。如果 `K_slide` 太小，典型现象是：

```text
低速能转
速度升高后 SMO 反电势饱和
角度突然偏离
FOC 力矩方向错误
电机抽搐、掉速
掉速后又重新锁定
```

`K_slide` 也不能无限大。太大时，`z` 的高频成分会很强，反电势 LPF 前的信号抖动会变大；如果 `e_lpf_alpha` 又偏大，PLL 看到的就是很吵的角度。工程上通常先保证 `K_slide` 不小于最高速反电势，再通过 `band`、`e_lpf_alpha` 和 PLL 带宽处理噪声。

### 4.8 反转时为什么要处理反电势方向

PMSM 反电势矢量与磁链角关系：

```text
E = omega_e * psi * [-sin(theta), cos(theta)]
```

当 `omega_e > 0` 时：

```text
theta = atan2(-eAlpha, eBeta)
```

但当 `omega_e < 0` 时，整个反电势矢量会翻转 `pi`。如果不处理，反转时 SMO/PLL 角度会和真实磁链角差 180 度，切闭环后 q 轴力矩方向会错。

当前代码用 `SMO_GetElectricalDirectionSign()` 得到电角速度方向：

```text
优先使用 I/F 阶段 sensorlessIfSpeed * speedDir
其次使用 tar_speed * speedDir
最后使用 PLL/rawSpeed
```

然后：

```c
bemfDir = SMO_GetElectricalDirectionSign(smo);
eAlphaFlux = bemfDir * smo->eAlpha;
eBetaFlux = bemfDir * smo->eBeta;
smo->rawAngle = NormalizeAngle(atan2f(-eAlphaFlux, eBetaFlux));
```

这一步的本质是：反转时先把反电势矢量翻回与正转相同的磁链角定义，再交给 PLL。

如果不做这一步，反转时会出现一个非常迷惑的现象：SMO 的反电势波形看起来是正常的，PLL 速度也可能有数，但角度相对真实磁链差 `pi`。FOC 用这个角度做 Park 变换后，原本应该是正向力矩的 `Iq` 会被解释到相反方向或错误轴上，表现为 I/F 阶段能拖动，切到 DONE 后立刻卡住或反向抽动。

---

## 5. PLL 锁相环

PLL 实现在 `pll_observer.c`。SMO 先得到反电势 `eAlpha/eBeta`，PLL 再从反电势中提取平滑角度和速度。

### 5.1 为什么不用 atan2 直接做控制角度

`atan2(-eAlpha, eBeta)` 可以直接算角度，但有几个问题：

- 低速时反电势小，角度噪声很大
- 角度跨 0/2pi 时差分求速度容易跳变
- 速度需要差分，差分会放大噪声
- 反电势低通带来的相位滞后会直接体现在角度上

PLL 的作用：

```text
用反电势方向构造角度误差
PI 调节出 omega
积分 omega 得到 theta
```

这样速度 `omega` 是 PLL 的状态量，不是简单角度差分，通常更平滑。

### 5.2 PLL 误差构造

反电势理论方向：

```text
E = E_mag * [-sin(theta), cos(theta)]
```

估计角度为 `theta_hat` 时，构造误差：

```text
epsilon = -eAlpha * cos(theta_hat) - eBeta * sin(theta_hat)
```

源码：

```c
pllErr = -eAlphaFlux * cosf(smo->pll.theta)
         - eBetaFlux  * sinf(smo->pll.theta);
```

这个误差等价于：

```text
epsilon = E_mag * sin(theta - theta_hat)
```

小角度误差时：

```text
sin(theta - theta_hat) ≈ theta - theta_hat
```

因此 PLL 可以把它当成角度误差来闭环。

完整展开如下。假设真实反电势为：

```text
eAlpha = -E * sin(theta)
eBeta  =  E * cos(theta)
```

PLL 内部估计角度为 `theta_hat`。代码构造：

```text
epsilon = -eAlpha * cos(theta_hat) - eBeta * sin(theta_hat)
```

代入：

```text
epsilon
  = -[-E*sin(theta)] * cos(theta_hat)
    -[ E*cos(theta)] * sin(theta_hat)

  = E*sin(theta)*cos(theta_hat)
    - E*cos(theta)*sin(theta_hat)

  = E * [sin(theta)*cos(theta_hat)
         - cos(theta)*sin(theta_hat)]

  = E * sin(theta - theta_hat)
```

所以：

```text
theta > theta_hat  -> epsilon > 0 -> PLL 增大 omega -> theta_hat 追上去
theta < theta_hat  -> epsilon < 0 -> PLL 减小 omega -> theta_hat 慢下来
```

这就是锁相环“锁相”的含义：它不是每次直接把 `theta_hat` 设成 `atan2` 角度，而是通过一个连续的误差信号不断调节角速度，让内部角度自动追上反电势矢量。

再换一个几何解释：估计角度 `theta_hat` 对应的估计反电势方向是：

```text
E_hat_dir = [-sin(theta_hat), cos(theta_hat)]
```

真实反电势方向与估计方向之间的叉积，正比于两者夹角的正弦。代码里的 `epsilon` 本质上就是这个夹角误差信号。夹角小的时候，`sin(delta) ≈ delta`，所以 PLL 可以像普通 PI 一样调。

### 5.3 幅值归一化

源码：

```c
smo->eMag = sqrtf(smo->eAlpha * smo->eAlpha + smo->eBeta * smo->eBeta);

if (smo->eMag > FOC_EPSILON) {
    pllErr /= smo->eMag;
}
```

原因：

```text
未归一化误差 = E_mag * sin(angle_error)
```

如果不除以 `E_mag`，转速越高反电势越大，相同角度误差产生的 PLL 输入越大，等效 PLL 增益会随转速变化。归一化后：

```text
pllErr ≈ sin(angle_error)
```

PLL 增益更容易整定。

### 5.4 PLL 初始化

首次有效更新时：

```c
PLL_SetInitialAngle(&smo->pll, smo->rawAngle);
smo->pll.integral = smo->rawSpeed;
smo->pll.omega = smo->rawSpeed;
```

这里 `rawAngle` 来自反电势 `atan2`，`rawSpeed` 来自 rawAngle 差分。

意义：

- 角度初值直接放到当前反电势角度附近，避免 PLL 从 0 角度慢慢追
- 速度积分项直接设置到 rawSpeed，避免高速捕获时 omega 从 0 慢慢爬

### 5.5 PLL 更新公式

`PLL_Update()`：

```c
pll->integral += pll->cfg.ki * error * pll->cfg.ts;
pll->omega = pll->cfg.kp * error + pll->integral;
pll->theta += pll->omega * pll->cfg.ts;
pll->theta = NormalizeAngle(pll->theta);
```

对应：

```text
integral[k] = integral[k-1] + Ki * error * Ts
omega[k] = Kp * error + integral[k]
theta[k] = theta[k-1] + omega[k] * Ts
```

配置：

```c
#define FOC_SMO_PLL_KP 800.0f
#define FOC_SMO_PLL_KI 80000.0f
```

调参直觉：

| 参数 | 太小 | 太大 |
|------|------|------|
| `PLL_KP` | 角度跟踪慢，高速滞后 | 角度抖，噪声放大 |
| `PLL_KI` | 速度爬升慢，稳态速度误差大 | 速度振荡，积分过猛 |

### 5.6 rawSpeed 前馈

在 `SMO_Update()` 中还有一行：

```c
smo->pll.integral += FOC_SMO_PLL_SPEED_FF_ALPHA
                     * (smo->rawSpeed - smo->pll.integral);
```

这相当于慢慢把 PLL 的速度积分状态拉向 SMO 原始角度差分速度。

配置：

```c
#define FOC_SMO_PLL_SPEED_FF_ALPHA 0.05f
```

作用：

- 帮助 PLL 高速捕获
- 避免 `omega` 完全靠误差积分慢慢追上真实速度

代价：

- `rawSpeed` 是差分量，本身比 PLL 速度噪
- 前馈系数太大会把噪声带进 PLL

### 5.7 反电势 LPF 相位补偿

反电势经过一阶低通后会滞后。当前代码估算滞后角并加到 PLL 角度上：

```c
smo->phaseComp = FOC_SMO_PHASE_COMP_GAIN
                 * atanf(smo->pll.omega * smo->cfg.ts / smo->cfg.e_lpf_alpha);

smo->angle = NormalizeAngle(smo->pll.theta + smo->phaseComp);
smo->speed = smo->pll.omega;
```

近似原理：

```text
一阶 LPF 在角频率 omega 处的相位滞后
phi_lag ≈ atan(omega * Ts / alpha)
```

所以输出角度：

```text
theta_out = theta_pll + phaseComp
```

配置：

```c
#define FOC_SMO_PHASE_COMP_GAIN 1.0f
```

如果高速时 SMO 角度总是落后编码器角度，可以关注这项；如果补偿太多，可能表现为角度超前、力矩效率下降或抖动。

---

## 6. 电流环实现

电流环在 `current_control.c`，每个 PWM 周期由 `FocContorl()` 调用。

### 6.1 电流采样和换算

`FocContorl()` 中：

```c
pFOC->Ia = (adcA - offsetA) / 4096.0f
           * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
```

同理计算 `Ib/Ic`。

配置：

```c
#define FOC_ADC_REF_VOLTAGE 3.3f
#define FOC_GAIN            10
#define FOC_SHUNT_R         0.01f
```

因此：

```text
1 ADC count ≈ 3.3 / 4096 / 10 / 0.01 = 0.00806 A
```

注意：本项目有硬件约束，Ia ADC 通道不可靠，应使用 Ib/Ic 重构 Ia。当前代码中有电流重构函数 `CurrentReconstruction()`，并在 Clarke 前有注释说明 Ia 采样异常。

### 6.2 Clarke 变换

源码：

```c
void clarke_transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (1.0f / FOC_SQRT3) * (Ia + 2.0f * Ib);
}
```

公式：

```text
iAlpha = Ia
iBeta = (Ia + 2*Ib) / sqrt(3)
```

`FocContorl()` 调用时：

```c
clarke_transform(-pFOC->Ia, -pFOC->Ib, &pFOC->iAlpha, &pFOC->iBeta);
```

这里的负号是电流方向定义修正，表示 ADC/功率级采样方向与 FOC 数学正方向相反。

### 6.3 Park 变换

源码：

```c
Id = Ialpha * cos(theta) + Ibeta * sin(theta)
Iq = -Ialpha * sin(theta) + Ibeta * cos(theta)
```

公式：

```text
id = iAlpha*cos(theta) + iBeta*sin(theta)
iq = -iAlpha*sin(theta) + iBeta*cos(theta)
```

这里 `theta` 就是 `correctedAngle`。有感时来自编码器，无感 I/F 时来自虚拟角度，无感闭环时来自 SMO/PLL。

### 6.4 Id/Iq 电流 PI

Id 环：

```c
pFOC->idPID.pre = pFOC->id;
pFOC->idPID.tar = pFOC->tarid;

if (sensorlessIfState == ALIGN) {
    pFOC->idPID.tar = pFOC->sensorlessIfId;
}
```

Iq 环：

```c
pFOC->iqPID.pre = pFOC->iq;
pFOC->iqPID.tar = pFOC->tariq;
```

在速度环/位置环中：

```c
pFOC->iqPID.tar = pFOC->speedPID.out;
```

在 I/F ALIGN/RAMP 中：

```c
pFOC->iqPID.tar = pFOC->sensorlessIfIq;
```

PI 算法：

```c
out += ki * (bias - lastBias) + kp * bias;
```

当前电流 PI 是增量式写法。输出限幅：

```c
iqPID.out = clamp(iqPID.out, -abs(iqPID.outMax), +abs(iqPID.outMax));
idPID.out = clamp(idPID.out, -abs(idPID.outMax), +abs(idPID.outMax));
```

初始化默认值在 `freertos_app.c`：

```c
SetCurrentPIDParams(g_pMotor, 0.0005f, 0.5f, 0.0f, 12.0f);
```

含义：

- `iqPID.out` 的单位是 V，对应 `uq`
- `iqPID.outMax = 12.0f` 是 q 轴电压限幅，不是电流限幅

### 6.5 三层电流/电压限制

当前有三层限制：

| 参数 | 单位 | 作用 |
|------|------|------|
| `tariq` | A | 电流环模式下是 Iq 目标；速度/位置环下是速度环输出上限 |
| `tariqMax` | A | 绝对 Iq 参考上限，所有模式生效 |
| `iqPID.outMax` | V | Uq 电压输出限幅 |

速度环下：

```text
speedPID.out -> iqPID.tar
iqPID.tar 再被 tariq 和 tariqMax 限幅
iqPID.out 再被 iqPID.outMax 限压
```

所以速度环想限制力矩，应设置 `CMD_SETIQ` 为非零电流上限；想限制最大输出电压，才设置 `CMD_SETIQPIDOUT`。

### 6.6 逆 Park 和 SVPWM

逆 Park：

```c
Ualpha = -Uq*sin(theta) + Ud*cos(theta)
Ubeta  =  Uq*cos(theta) + Ud*sin(theta)
```

源码：

```c
inv_park_transform(pFOC->uq, pFOC->ud, pFOC->correctedAngle,
                   &(pFOC->uAlpha), &(pFOC->uBeta));
```

然后：

```c
SVpwm(PSVpwm, pFOC->uAlpha, pFOC->uBeta);
setSVpwm(PSVpwm);
```

SVPWM 把 αβ 电压矢量转换成三相 PWM 占空比，最终写入 TMR1 CH1/CH2/CH3。

---

## 7. 速度环实现

速度环在 `speed_control.c`，由 `control_task_func()` 每 2ms 调用。

### 7.1 速度反馈来源

`CalculateSpeed()` 按 `sensorMode` 分两种情况。

有感：

```c
angle_diff = mechanicalAngle - mechanicalAngle_last;
处理 0/2pi 跨周期
speed = speedDir * angle_diff / dt * 60 / 2pi;
LPF_Speed_Update(...)
```

单位是 rpm。

无感：

```c
pFOC->speed = pFOC->sensorlessMechanicalSpeed;
LPF_Speed_Update(...)
```

`sensorlessMechanicalSpeed` 在 `FocContorl()` 高频中断中由 PLL 电角速度换算：

```c
pFOC->sensorlessMechanicalSpeed =
    g_smoObserver.speed / pole_pairs * speedDir * 60 / 2pi;
```

因此：

```text
g_smoObserver.speed: 电角速度 rad/s
sensorlessMechanicalSpeed: 机械转速 rpm
speed: 速度环实际反馈 rpm
tar_speed: 速度环目标 rpm
```

这是项目里的关键约定：速度环输入输出的速度单位是 rpm，不是 rad/s。

### 7.2 速度 PI

速度目标：

```c
pFOC->speedPID.tar = pFOC->tar_speed;
```

反馈：

```c
pFOC->speedPID.pre = pFOC->speed;
```

误差：

```c
bias = tar_speed - speed;
```

无感模式下使用标准增量式 PI：

```c
speedPID.out += kp * (bias - lastBias)
                + ki * bias * FOC_SPEED_LOOP_TS;
```

有感模式沿用旧参数语义：

```c
speedPID.out += ki * (bias - lastBias)
                + kp * bias;
```

这点非常重要：有感和无感速度 PI 参数含义不同。无感分支的 `ki` 乘了 `dt`，更接近连续 PI 的离散形式。

输出限幅：

```c
speedPID.out = clamp(speedPID.out, -outMax, +outMax);
```

再用 `tariq` 限幅：

```c
if (tariq > EPSILON) {
    speedPID.out = clamp(speedPID.out, -tariq, +tariq);
}
```

所以 `speedPID.out` 的单位是 A，是给 q 轴电流环的目标/参考。

### 7.3 I/F 阶段速度环冻结

在 `SpeedPIControl()` 开头：

```c
if (sensorlessIfState == ALIGN || sensorlessIfState == RAMP) {
    speedPID.out = sensorlessIfIq;
    speedPID.lastBias = 0.0f;
    return;
}
```

含义：

- I/F 阶段不让速度 PI 根据错误速度反馈乱调
- RAMP 阶段的 q 轴电流由 `sensorlessIfIq` 固定给定
- 等 SMO/PLL 锁定后再让速度 PI 接管

### 7.4 速度环到电流环

`CurrentPIControlIQ()` 中：

```c
if (ctrolmode == FOC_SPEED_LOOP || ctrolmode == FOC_POSITION_LOOP) {
    iqPID.tar = speedPID.out;
}
```

这就是速度环和电流环的连接点。

最终链路：

```text
tar_speed (rpm)
  - speed (rpm)
  -> speedPID.out (A)
  -> iqPID.tar (A)
  - iq (A)
  -> iqPID.out (V)
  -> uq (V)
  -> uAlpha/uBeta
  -> SVPWM
```

---

## 8. 有感/无感切换

切换命令在 `protocol.h`：

```c
CMD_SENSOR_SENSORED   = 0x58
CMD_SENSOR_SENSORLESS = 0x59
```

处理在 `protocol.c`：

```c
FOC_SetSensorMode(g_pMotor, FOC_SENSOR_MODE_SENSORED);
FOC_SetSensorMode(g_pMotor, FOC_SENSOR_MODE_SENSORLESS);
```

`FOC_SetSensorMode()` 做的事：

无感：

```text
sensorMode = SENSORLESS
sensorlessOpenLoopAngle = correctedAngle
SMO_Reset()
```

有感：

```text
sensorMode = SENSORED
```

共同清理：

```text
speedPID.out = 0
speedPID.lastBias = 0
positionPID.lastBias = 0
sensorlessIfState = OFF
sensorlessIf counters = 0
sensorlessIf speed/current = 0
```

这样切换模式时不会把旧速度环积分、旧 I/F 状态、旧 SMO 状态带到新模式。

当前 `FocContorl()` 末尾还有一个重要优化：

```c
if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORLESS) {
    SMO_Update(...);
}
```

也就是说，有感模式下不再更新 SMO，避免 SMO 的 `atan2f/atanf/sqrtf/cosf/sinf` 等浮点计算占用 PWM 中断预算。这个和“加了无感后有感也抖”的排查方向直接相关。

---

## 9. 命令使用顺序

典型无感速度环启动流程：

```text
CMD_SENSOR_SENSORLESS      切无感角度来源
CMD_SPEED_LOOP             切速度环
CMD_SETIQ                  设置速度环 Iq 上限，例如 1.0A~2.0A
CMD_SETSPEEDTAR            设置目标速度，例如 1000rpm
```

如果要看 SMO 状态：

```text
CMD_SMO_ANGLE              PLL 后角度
CMD_SMO_RAW_ANGLE          反电势 atan2 原始角度
CMD_SMO_SPEED              PLL 速度，最终会换算为 rpm 遥测
CMD_SMO_BACKEMF            eAlpha/eBeta
CMD_SMO_DIAG               pllError/eMag
CMD_ELECTRICALANGLE        编码器电角度，用于对比
```

典型有感调 SMO 的方法：

```text
1. 切有感模式，确保电机用编码器稳定运行
2. 观察 SMO 角度、SMO 速度、反电势、PLL 误差
3. 对比编码器电角度和 SMO/PLL 角度
4. 确认方向、幅值、相位都合理后，再切无感
```

但要注意：当前代码有感模式下跳过 `SMO_Update()`，所以如果想“有感运行时后台观测 SMO”，需要临时允许有感模式也更新 SMO，或者专门加调试开关。否则有感模式下 SMO 遥测不会持续更新。

---

## 10. 参数表

### 10.1 SMO/PLL 参数

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `FOC_SMO_RS` | `0.198f` | 默认相电阻，实际初始化优先用 Flash 中 `g_pMotor->rs` |
| `FOC_SMO_LS` | `0.000057f` | 默认等效电感 |
| `FOC_SMO_TS` | `0.00005f` | SMO 周期，20kHz |
| `FOC_SMO_K_SLIDE` | `20.0f` | 滑模增益，需覆盖最大反电势 |
| `FOC_SMO_E_LPF_ALPHA` | `0.02f` | 反电势低通系数 |
| `FOC_SMO_CURRENT_ERR_BAND` | `10.0f` | 滑模饱和带 |
| `FOC_SMO_PLL_KP` | `800.0f` | PLL 比例增益 |
| `FOC_SMO_PLL_KI` | `80000.0f` | PLL 积分增益 |
| `FOC_SMO_PLL_SPEED_FF_ALPHA` | `0.05f` | PLL rawSpeed 前馈系数 |
| `FOC_SMO_PHASE_COMP_GAIN` | `1.0f` | 反电势 LPF 相位补偿强度 |

### 10.2 I/F 启动参数

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `FOC_SENSORLESS_IF_ALIGN_ID` | `0.50f` | ALIGN 对齐 Id |
| `FOC_SENSORLESS_IF_ALIGN_COUNT` | `6000U` | ALIGN 持续周期数 |
| `FOC_SENSORLESS_IF_START_IQ` | `1.00f` | RAMP 起动 Iq |
| `FOC_SENSORLESS_IF_START_SPEED_RPM` | `20.0f` | RAMP 初始速度 |
| `FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM` | `1000.0f` | SMO 交接目标速度 |
| `FOC_SENSORLESS_IF_RAMP_RPM_PER_S` | `1500.0f` | RAMP 速度斜率 |
| `FOC_SENSORLESS_IF_START_MAX_SPEED_RPM` | `150.0f` | 低于该速度才自动 I/F |
| `FOC_SENSORLESS_IF_MIN_TARGET_RPM` | `10.0f` | 目标小于该值不启动 |
| `FOC_SENSORLESS_IF_MIN_EMAG` | `0.15f` | 交接最小反电势 |
| `FOC_SENSORLESS_IF_MAX_PLL_ERROR` | `0.35f` | 交接最大 PLL 误差 |
| `FOC_SENSORLESS_IF_LOCK_COUNT` | `200U` | 连续满足交接条件周期数 |

### 10.3 环路参数

| 参数 | 当前默认 | 含义 |
|------|----------|------|
| 电流环 `kp` | `0.0005f` | Id/Iq PI 比例 |
| 电流环 `ki` | `0.5f` | Id/Iq PI 积分/增量项 |
| 电流环 `outMax` | `12.0f` | Uq/Ud 电压限幅 |
| 速度环 `kp/ki` | Flash 参数 | 启动时从 Flash 读取 |
| 速度环 `outMax` | `10.0f` | 速度 PI 输出 Iq 限幅 |
| `tariqMax` | `40.0f` | Iq 参考绝对安全上限 |

---

## 11. 常见现象和对应原因

### 11.1 1000rpm 正常，0rpm 或低速抖

根因：

```text
低速反电势弱
SMO/PLL 角度不可观测
速度反馈噪声大
速度 PI 输出正负来回打
电流环在错误角度下输出交变力矩
```

当前代码用低速限流和冻结角度缓解，但无感本身不能真正做到高质量零速闭环。想完全静止且不输出力，应清零电流目标或关断 PWM；想零速保持力矩，则需要有感编码器或高频注入等专用低速观测方法。

### 11.2 I/F RAMP 能拖起来，DONE 后卡住

可能原因：

- SMO 角度方向错
- 反转时反电势差 `pi` 没处理好
- `speedDir/dir/pole_pairs` 配置错误
- PLL 未真正锁定，但交接条件过松
- `K_slide` 太小，高速反电势饱和

重点看：

```text
CMD_SMO_ANGLE vs CMD_ELECTRICALANGLE
CMD_SMO_RAW_ANGLE
CMD_SMO_SPEED
CMD_SMO_DIAG: pllError/eMag
```

### 11.3 高速抽搐、掉速、再恢复

优先怀疑 `K_slide` 太小或相位补偿/PLL 带宽不合适。

判断：

```text
eMag 增大后角度突然偏离
pllError 长期变大
SMO 角度与编码器角度差快速扩大
```

处理顺序：

```text
1. 增大 K_slide
2. 检查 Rs/Ls 是否接近真实电机
3. 检查 PLL Kp/Ki
4. 检查相位补偿是否过度或不足
```

### 11.4 加上无感后，有感运行也抖

这类问题通常不是“无感角度参与了有感控制”，而是“中断计算量或中断抖动影响了电流环时序”。

当前代码已经做了关键优化：

```text
有感模式不更新 SMO
```

如果仍然抖，应继续检查：

- ADC/PWM 中断执行时间是否接近 50us
- USART/DMA/TMR2 遥测是否抢占或阻塞
- 是否在 ISR 中做了过多浮点、打印、发送
- 中断优先级是否让通信类中断打断了 ADC 电流环
- `FOC_ENABLE_DEBUG` 是否在生产运行中关闭

最直接验证方法：

```text
ADC 中断入口拉高 GPIO
ADC 中断出口拉低 GPIO
示波器测高电平宽度和抖动
```

20kHz 下周期只有 50us，FOC 计算最好明显低于这个周期，并且不能有偶发长尾。

---

## 12. 一句话总结

当前无感实现的完整闭环是：

```text
I/F 用虚拟角度和固定电流把电机拖到 1000rpm 附近
  -> SMO 根据 uAlpha/uBeta 和 iAlpha/iBeta 估算反电势
  -> PLL 从反电势提取电角度和电角速度
  -> FOC 用 PLL 电角度做 Park/逆 Park
  -> 速度环用 PLL 速度换算的 rpm 做反馈
  -> 速度 PI 输出 Iq 参考
  -> 电流 PI 输出 Uq
  -> SVPWM 输出三相 PWM
```

关键限制是：

```text
SMO/PLL 依赖反电势
反电势依赖转速
所以无感天然不擅长零速和极低速
```

工程上必须靠 I/F 启动、交接判据、低速保护、限流、角度冻结，以及必要时切回有感编码器来覆盖这些盲区。

---

## 13. 与知乎 FOC 技术博客的概念对照

参考链接：`https://zhuanlan.zhihu.com/p/416224632`

说明：该知乎链接在当前环境中返回 403，无法直接抓取全文。因此本节先按该类 FOC 技术博客常见的讲解主线做概念对照：三相量 → αβ 坐标 → dq 坐标 → PI 电流环 → 逆变换 → SVPWM → 无感角度来源。若后续拿到博客原文标题或截图，可再把本节细化成“博客第 N 节/第 N 张图 ↔ 本项目代码”的逐条对照。

### 13.1 三相电流是空间矢量的三个投影

博客常见讲法：

```text
三相电流 Ia/Ib/Ic 幅值相等、相位相差 120 度。
它们可以看作同一个旋转电流空间矢量在 a/b/c 三个相轴上的投影。
```

本项目对应：

```c
pFOC->Ia = ...
pFOC->Ib = ...
pFOC->Ic = ...
```

位置：

```text
FOC.c -> FocContorl() -> ADC 值换算 Ia/Ib/Ic
```

工程含义：

```text
ADC 采到的是三相绕组电流。
FOC 算法不直接在 abc 三相坐标里控制，
而是先把它们变换到更容易控制的 αβ/dq 坐标。
```

注意本项目的硬件约束：

```text
Ia ADC 通道有问题，需要用 Ib/Ic 重构 Ia。
```

所以文档和代码中要特别关注：

```text
CurrentReconstruction()
clarke_transform(-pFOC->Ia, -pFOC->Ib, ...)
```

这里的负号来自采样方向和数学正方向的约定差异。

### 13.2 abc 到 αβ：把三相投影还原成二维静止坐标矢量

博客常见讲法：

```text
三相坐标轴互差 120 度，不方便直接做矢量控制。
Clarke 变换把三相量投影到 αβ 直角坐标系。
αβ 坐标系固定在定子上，所以叫静止坐标系。
```

本项目代码：

```c
void clarke_transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (1.0f / FOC_SQRT3) * (Ia + 2.0f * Ib);
}
```

对应公式：

```text
i_alpha = i_a
i_beta  = (i_a + 2*i_b) / sqrt(3)
```

这和博客里“投影/矩阵计算”的关系是：

```text
abc 三个 120 度相轴
  -> 投影到 α 轴和 β 轴
  -> 得到同一个电流空间矢量的二维坐标
```

为什么可以只用 `Ia/Ib`：

```text
三相平衡时 Ia + Ib + Ic = 0
Ic = -(Ia + Ib)
```

因此两相信息已经足够还原三相空间矢量。本项目还利用这个关系处理 Ia 硬件异常。

### 13.3 αβ 到 dq：把静止矢量投影到转子同步坐标系

博客常见讲法：

```text
αβ 坐标固定不动，电流矢量会随电机旋转而转。
dq 坐标跟着转子磁链一起转。
把 αβ 投影到 dq 后，正弦交流量会变成近似直流量。
```

本项目代码：

```c
void park_transform(float Ialpha, float Ibeta, float angle_el,
                    float *Id, float *Iq)
{
    *Id = Ialpha * fast_cos(angle_el) + Ibeta * fast_sin(angle_el);
    *Iq = -Ialpha * fast_sin(angle_el) + Ibeta * fast_cos(angle_el);
}
```

对应矩阵：

```text
[id]   [ cos(theta)   sin(theta)] [i_alpha]
[iq] = [-sin(theta)   cos(theta)] [i_beta ]
```

博客中如果用“向 d 轴和 q 轴做投影”解释 Park 变换，对应到代码就是：

```text
id = 电流矢量在转子磁链方向上的投影
iq = 电流矢量在转子磁链垂直方向上的投影
```

控制意义：

```text
Id: 励磁/磁链方向电流
Iq: 力矩方向电流
```

对表贴式 PMSM，常用 `Id = 0`：

```text
不额外增磁/弱磁
主要用 Iq 控制转矩
```

本项目中：

```text
正常运行: pFOC->ud 初始给很小值，Id 环维持接近 0
I/F ALIGN: 临时给 Id，用来对齐转子
I/F RAMP: Id 回到 0，给 Iq 拖动
```

### 13.4 电角度 theta 是 Park 变换的核心输入

博客常见讲法：

```text
Park 变换需要知道转子电角度。
角度准，dq 解耦才成立；角度错，Id/Iq 就不是真正的磁链/力矩分量。
```

本项目对应变量：

```text
pFOC->correctedAngle
```

它是当前控制真正使用的电角度：

```text
有感模式:
  MT6701 机械角度 -> 乘极对数 -> 减零偏 -> correctedAngle

无感 I/F 阶段:
  sensorlessIfAngle -> correctedAngle

无感闭环阶段:
  g_smoObserver.angle -> correctedAngle
```

对应代码位置：

```text
FOC.c -> FocContorl() -> 步骤 1：获取并选择角度
```

这一点可以和博客的 FOC 总框图对应：

```text
角度 theta
  -> Park 变换
  -> 电流 PI
  -> 逆 Park 变换
```

如果 `theta` 错了，后面的电流 PI 仍然会努力控制 `Id/Iq`，但这两个量已经不是物理上的真实 d/q 电流，最终就会抖动、发热或转矩方向错误。

### 13.5 dq 电流 PI：把“想要的 Id/Iq”变成“需要的 Ud/Uq”

博客常见讲法：

```text
FOC 把交流电机变成类似直流电机来控制。
在 dq 坐标下，Id/Iq 接近直流量，所以可以用 PI 控制。
```

本项目对应：

```text
CurrentPIControlID()
CurrentPIControlIQ()
```

控制链：

```text
idPID.tar - id -> idPID.out -> ud
iqPID.tar - iq -> iqPID.out -> uq
```

电流环 PI 输出为什么是电压：

```text
电流不是 MCU 能直接输出的量。
MCU 只能通过 PWM 改变电机端电压。
电压作用在 Rs/Ls 和反电势上，最终改变电流。
所以电流 PI 的控制量自然是 Ud/Uq 电压指令。
```

对应本项目代码：

```c
pFOC->uq = pFOC->iqPID.out;
pFOC->ud = pFOC->idPID.out;
```

### 13.6 逆 Park：把 dq 电压转回 αβ 电压

博客常见讲法：

```text
PI 算出来的是转子同步坐标系下的 Ud/Uq。
但逆变器输出的是定子三相电压。
所以要先做逆 Park，从 dq 转回 αβ。
```

本项目代码：

```c
static void inv_park_transform(float Uq, float Ud, float corr_angle,
                               float *Out_Ualpha, float *Out_Ubeta)
{
    *Out_Ualpha = -Uq * fast_sin(corr_angle) + Ud * fast_cos(corr_angle);
    *Out_Ubeta  =  Uq * fast_cos(corr_angle) + Ud * fast_sin(corr_angle);
}
```

对应矩阵：

```text
[u_alpha]   [cos(theta)  -sin(theta)] [ud]
[u_beta ] = [sin(theta)   cos(theta)] [uq]
```

按本项目变量顺序写就是：

```text
u_alpha = ud*cos(theta) - uq*sin(theta)
u_beta  = ud*sin(theta) + uq*cos(theta)
```

### 13.7 SVPWM：把 αβ 电压矢量变成三相 PWM

博客常见讲法：

```text
SVPWM 根据目标电压空间矢量所在扇区，计算相邻两个有效矢量和零矢量的作用时间。
最终生成三相桥臂 PWM 占空比。
```

本项目对应：

```text
SVpwm(PSVpwm, pFOC->uAlpha, pFOC->uBeta)
setSVpwm(PSVpwm)
```

数据流：

```text
uAlpha/uBeta
  -> SVpwm()
  -> PSVpwm->Ta/Tb/Tc
  -> TMR1 CH1/CH2/CH3
```

这对应博客 FOC 框图最后一段：

```text
Ualpha/Ubeta -> SVPWM -> 三相逆变器 -> 电机
```

### 13.8 有感 FOC 和无感 FOC 对博客框图的区别

如果博客画的是标准有感 FOC 框图，一般是：

```text
编码器/霍尔 -> theta
Ia/Ib/Ic -> Clarke -> Park
Id/Iq PI -> 逆 Park -> SVPWM
```

本项目有感模式完全对应这条链路：

```text
MT6701 -> AngleGetCorrectedElec() -> correctedAngle
```

无感模式只替换“角度来源”这一块：

```text
有感:
  编码器给 theta

无感:
  I/F 启动先给虚拟 theta
  SMO/PLL 锁定后给估算 theta
```

其余 FOC 主体不变：

```text
Clarke/Park/PI/逆 Park/SVPWM 仍然是同一套
```

这也是调试时的关键判断：

```text
有感能跑，说明 Clarke/Park/PI/SVPWM 主链路大概率没问题。
无感抖，重点看 theta 来源：I/F、SMO、PLL、交接条件。
```

### 13.9 博客公式和本项目变量速查

| 博客概念 | 常见符号 | 本项目变量/函数 |
|----------|----------|-----------------|
| A/B/C 三相电流 | `ia/ib/ic` | `pFOC->Ia/Ib/Ic` |
| αβ 电流 | `i_alpha/i_beta` | `pFOC->iAlpha/iBeta` |
| dq 电流 | `id/iq` | `pFOC->id/iq` |
| 电角度 | `theta_e` | `pFOC->correctedAngle` |
| Clarke 变换 | `abc -> alpha beta` | `clarke_transform()` |
| Park 变换 | `alpha beta -> dq` | `park_transform()` |
| 逆 Park | `dq -> alpha beta` | `inv_park_transform()` |
| q 轴电压 | `Uq` | `pFOC->uq` / `iqPID.out` |
| d 轴电压 | `Ud` | `pFOC->ud` / `idPID.out` |
| αβ 电压 | `Ualpha/Ubeta` | `pFOC->uAlpha/uBeta` |
| SVPWM 占空比 | `Ta/Tb/Tc` | `PSVpwm->Ta/Tb/Tc` |
| 速度目标 | `speed_ref` | `pFOC->tar_speed`，单位 rpm |
| 速度反馈 | `speed` | `pFOC->speed`，单位 rpm |
| 速度 PI 输出 | `Iq_ref` | `pFOC->speedPID.out` |
| 无感反电势 | `e_alpha/e_beta` | `g_smoObserver.eAlpha/eBeta` |
| 无感角度 | `theta_hat` | `g_smoObserver.angle` |
| 无感电角速度 | `omega_hat` | `g_smoObserver.speed`，单位 rad/s |

### 13.10 用博客框图看本项目无感速度环

可以把本项目无感速度环压缩成一条和博客 FOC 框图对应的数据链：

```text
速度外环:
tar_speed - speed
  -> SpeedPIControl()
  -> speedPID.out = Iq_ref

电流内环:
Iq_ref - iq
  -> CurrentPIControlIQ()
  -> Uq

坐标变换:
Ia/Ib/Ic
  -> Clarke
  -> iAlpha/iBeta
  -> Park(theta)
  -> id/iq

电压输出:
Ud/Uq
  -> inverse Park(theta)
  -> Ualpha/Ubeta
  -> SVPWM
  -> PWM

无感角度:
Ualpha/Ubeta + iAlpha/iBeta
  -> SMO
  -> eAlpha/eBeta
  -> PLL
  -> theta/speed
```

其中 `theta` 在不同阶段来自：

```text
I/F ALIGN/RAMP:
  theta = sensorlessIfAngle

SMO/PLL DONE:
  theta = g_smoObserver.angle

有感模式:
  theta = sensoredCorrectedAngle
```

所以这篇博客如果是在讲 FOC 的数学基础，那么它主要对应本项目的：

```text
clarke_transform()
park_transform()
inv_park_transform()
SVpwm()
CurrentPIControlID/IQ()
```

而本文档额外展开的是博客通常不会细讲的工程部分：

```text
I/F 启动
SMO 反电势观测
PLL 角度/速度提取
低速保护
有感/无感切换
```
