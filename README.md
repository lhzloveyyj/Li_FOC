# Li_FOC

`Li_FOC` 是基于 **AT32F403A/407** 的 FOC 电机控制固件工程，采用 **FreeRTOS** 做任务调度，支持通过串口协议与上位机进行参数配置、模式切换与实时数据回传。

该仓库目前主工程位于：
- `AT32F403ACCT7_WorkBench/`

---

## 1. 项目定位

本项目面向 BLDC/PMSM 的 FOC 控制实验与工程调试，核心能力包括：
- 三环控制链路：电流环 / 速度环 / 位置环
- ADC 中断驱动的电流采样 + FOC 主循环
- SVPWM 输出
- 串口命令控制与遥测回传
- 参数掉电保存（Flash + CRC32）
- FreeRTOS 任务化管理

---

## 2. 工程结构

```text
Li_FOC/
├─ LICENSE
├─ keil_build.log
└─ AT32F403ACCT7_WorkBench/
   ├─ BUILD.md                 # 构建与烧录说明（脚本）
   ├─ build-script.ps1         # Keil 构建脚本
   ├─ flash-script.ps1         # Keil 烧录脚本
   ├─ build.cmd / flash.cmd
   ├─ project/
   │  ├─ Hardware/             # FOC、协议、驱动增强模块
   │  │  ├─ FOC.c/.h
   │  │  ├─ SVPWM.c/.h
   │  │  ├─ current_control.c/.h
   │  │  ├─ speed_control.c/.h
   │  │  ├─ position_control.c/.h
   │  │  ├─ protocol.c/.h
   │  │  ├─ usart3.c/.h
   │  │  ├─ flash_ops.c/.h
   │  │  ├─ mt6701.c/.h
   │  │  └─ ...
   │  ├─ src/                  # 启动、外设配置、中断、RTOS入口
   │  │  ├─ main.c
   │  │  ├─ freertos_app.c
   │  │  ├─ at32f403a_407_int.c
   │  │  └─ ...
   │  ├─ inc/
   │  └─ MDK_V5/               # Keil 工程与输出
   ├─ libraries/               # CMSIS / AT32 drivers
   └─ middlewares/             # FreeRTOS 源码
```

---

## 3. 控制架构（按当前代码）

### 3.1 控制模式

`FOC.h` 中定义 4 种模式：
- `FOC_OPEN_LOOP`（开环）
- `FPC_CURRENT_LOOP`（电流环）
- `FOC_SPEED_LOOP`（速度-电流串级）
- `FOC_POSITION_LOOP`（位置-速度-电流串级）

### 3.2 运行链路

1. `ADC1_2_IRQHandler` 中读取三相电流 ADC 和 MOS 温度 ADC
2. 调用 `FocContorl(g_pMotor, PSVpwm)` 执行 FOC 核心计算
3. 在 `FocContorl` 内进行：
   - 机械角读取（MT6701）
   - 电角度计算与零偏修正
   - 电流重构 + Clarke/Park
   - 电流滤波
   - 电流环 PI（并串接速度/位置环输出）
   - 逆 Park + SVPWM
4. `TMR2_GLOBAL_IRQHandler` 按开关回传实时数据

### 3.3 FreeRTOS 任务

在 `freertos_app.c`：
- `comm_task`：命令处理、参数加载、初始化 PID 与滤波
- `control_task`：周期计算速度/位置与外环控制
- `Monitor_task`：周期发送 MOS 温度
- `TMR2_GLOBAL_IRQHandler`：按开关回传实时波形数据，包括母线电压 ADC 原始值

---

## 4. 串口协议

协议定义见 `project/Hardware/protocol.h` 与 `usart3.c`。

### 4.1 帧格式

- 帧头：`0xA5`
- 帧尾：`0x49`

#### 下位机 -> 上位机（可变长）

`USART3_SendPacket(cmd, values, count)` 发送：

```text
[HEAD][CMD][LEN][PAYLOAD...][CHECKSUM][TAIL]
```

- `LEN = count * 4`
- `PAYLOAD` 为 float 字节流
- `CHECKSUM = HEAD + CMD + LEN + PAYLOAD(逐字节累加, uint8)`

#### 上位机 -> 下位机（固定 8 字节）

`USART3_ParseFixedCommand` 解析：

```text
[HEAD][CMD][DATA0][DATA1][DATA2][DATA3][CHECKSUM][TAIL]
```

- 数据区固定为 1 个 float（4 字节）
- `CHECKSUM = HEAD + CMD + DATA0 + DATA1 + DATA2 + DATA3`

### 4.2 命令能力（节选）

主要命令与 `LiJointMaster` 上位机侧保持一致，包含：
- 电机连接、机械角打印开关、零点校准
- Uabc / ADC / Tabc / Iabc / Uαβ / Iαβ / IqId 打印开关
- `CMD_ADCVBUS / CMD_ADCVBUS_CLOSE`：打开/关闭母线电压 ADC 原始值曲线回传
- Uq/Ud、Iq/Id 目标设置
- 模式切换（开环/电流环/速度环/位置环）
- 速度目标、位置目标、PID 参数与输出限幅设置

命令枚举详见：`project/Hardware/protocol.h`。

### 4.3 母线电压 ADC 原始值回传

ADC1 预注入通道 4 的采样值保存到全局变量 `adcvbus`。上位机发送 `CMD_ADCVBUS` 后，`protocol.c` 将 `adcvbus_Enabled` 置 1；`TMR2_GLOBAL_IRQHandler` 在该开关打开时通过 `CMD_ADCVBUS` 回传 1 个 float，内容为 `(float)adcvbus`。上位机发送 `CMD_ADCVBUS_CLOSE` 后关闭该回传。

`CMD_MOSTEMP` 保持用于 MOS 温度，当前只回传 `GetMosTemp()` 的 1 个 float，不再携带 `getVbus()`。

---

## 5. 参数持久化（Flash）

`flash_ops.c/.h` 实现 FOC 参数掉电保存：
- 存储地址：`0x0803F800`
- 数据结构：`foc_params_t`
- 校验：`CRC32`

保存流程：
1. 组包参数 + CRC
2. 擦除目标扇区
3. 半字写入
4. 上电读取后 CRC 校验，不通过则视为无效参数

---

## 6. 外设与关键中断

- `ADC1_2_IRQHandler`：电流采样与 FOC 主计算
- `TMR2_GLOBAL_IRQHandler`：周期遥测发送
- `USART3_IRQHandler`：串口命令接收
- `DMA1_Channel1_IRQHandler`：USART3 DMA 发送完成

`main.c` 中初始化了 ADC、DMA、USART1/3、SPI1、TMR1/2、CAN1，并启动 FreeRTOS。

---

## 7. PWM 与驱动映射（仓库备注）

`AT32F403ACCT7_WorkBench/readme.txt` 中记录：

- `TIM1 CH1  -> HIN3 -> C相上管`
- `TIM1 CH1N -> LIN3 -> C相下管`
- `TIM1 CH2  -> HIN2 -> B相上管`
- `TIM1 CH2N -> LIN2 -> B相下管`
- `TIM1 CH3  -> HIN1 -> A相上管`
- `TIM1 CH3N -> LIN1 -> A相下管`

---

## 8. 构建与烧录

项目以 **Keil MDK5 (UV4)** 为主，脚本在：
- `AT32F403ACCT7_WorkBench/build-script.ps1`
- `AT32F403ACCT7_WorkBench/flash-script.ps1`

快速命令（PowerShell）：

```powershell
cd AT32F403ACCT7_WorkBench
.\build
.\flash
```

常用组合：

```powershell
.\flash -BuildFirst
.\flash -BuildFirst -ForceCloseUV4
```

详细说明见：`AT32F403ACCT7_WorkBench/BUILD.md`。

---

## 9. 配套上位机

推荐与仓库 `lhzloveyyj/LiJointMaster` 配套使用。
上位机命令与本固件 `protocol.h` 的命令枚举保持对应，可直接进行参数整定与波形观测。当前配套上位机已包含“母线ADC”曲线按钮，对应 `CMD_ADCVBUS / CMD_ADCVBUS_CLOSE`。

---

## 10. 注意事项

- 当前仓库包含大量构建产物（`project/MDK_V5/objects`、`listings` 等）
- 如需长期维护，建议补充 `.gitignore` 统一管理构建输出
- 代码中含硬编码 Keil 路径（见 `build-script.ps1`），跨机器使用前请先修改工具链路径

---

## 11. License

本仓库根目录已提供 `LICENSE`（MIT）。
