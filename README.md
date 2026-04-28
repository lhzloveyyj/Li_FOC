# Li_FOC

`Li_FOC` 是基于 **AT32F403A/407** 的无刷直流电机 FOC（Field Oriented Control）固件工程，采用 **FreeRTOS** 做任务调度，支持通过串口协议与上位机（QT）进行参数配置、模式切换与实时数据回传。

**当前代码已实现 SMO（滑模观测器）+ PLL（锁相环）无感角度观测，与 MT6701 磁编码器并行运行。**

---

## 1. 项目定位

本项目面向 BLDC/PMSM 的 FOC 控制实验与工程调试，核心能力包括：

| 功能 | 说明 |
|------|------|
| 三环控制链路 | 电流环 / 速度环 / 位置环串联控制 |
| ADC 中断驱动 | ADC 转换完成即触发 FOC 主循环（10kHz~20kHz） |
| SVPWM 调制 | 七段式空间矢量调制，含过调制处理 |
| 磁编码器 | MT6701（14-bit，SPI 接口） |
| 滑模观测器 | SMO + PLL 锁相环无感角度/速度估算 |
| 串口通信 | 自定义帧协议，支持命令/遥测双向传输 |
| 参数持久化 | Flash 存储 + CRC32 校验，兼容旧版升级 |
| FreeRTOS | 静态任务分配，实时多任务调度 |
| 快速三角函数 | Remez 多项式拟合 sin/cos，精度 ~1e-8 |

---

## 2. 工程结构

```text
Li_FOC/
├─ README.md
├─ LICENSE
├─ docs/                         # 文档（本工程）
│  └─ architecture.md
└─ AT32F403ACCT7_WorkBench/
   ├─ BUILD.md                   # 构建与烧录说明
   ├─ build.cmd / flash.cmd      # Windows 脚本
   ├─ build-script.ps1 / flash-script.ps1  # PowerShell 脚本
   ├─ project/
   │  ├─ Hardware/               # 核心硬件抽象层
   │  │  ├─ FOC.c/.h             # FOC 主控制逻辑
   │  │  ├─ foc_config.h         # 全局配置宏
   │  │  ├─ SVPWM.c/.h           # 空间矢量调制
   │  │  ├─ current_control.c/.h # 电流环 PI
   │  │  ├─ speed_control.c/.h   # 速度环 PI
   │  │  ├─ position_control.c/.h# 位置环 PD
   │  │  ├─ smo_observer.c/.h    # SMO+PLL 滑模观测器
   │  │  ├─ protocol.c/.h        # 串口通信协议
   │  │  ├─ usart3.c/.h          # USART3 驱动（DMA 发送）
   │  │  ├─ flash_ops.c/.h       # Flash 参数存取
   │  │  ├─ mt6701.c/.h          # MT6701 磁编码器驱动
   │  │  ├─ led.c/.h             # LED 驱动
   │  │  ├─ mostemp.c/.h         # NTC 温度 / 母线电压检测
   │  │  └─ delay.c/.h           # SysTick 延时
   │  ├─ src/                    # 主程序、外设配置、中断、RTOS
   │  │  ├─ main.c               # 主入口（外设初始化 + RTOS 启动）
   │  │  ├─ freertos_app.c       # FreeRTOS 任务实现
   │  │  ├─ commtask.c           # （精简版 comm_task 备选）
   │  │  ├─ at32f403a_407_int.c  # 全部中断服务函数
   │  │  └─ ...                  # 外设初始化（wk_*.c）
   │  ├─ inc/                    # 头文件
   │  ├─ tools/Math/             # 数学工具
   │  │  ├─ fast_sin.h           # 快速 sin/cos（Remez 多项式）
   │  │  ├─ filter.c/.h          # 一阶低通滤波器
   │  │  └─ my_math.c/.h         # 限幅等工具函数
   │  ├─ tools/LOG/              # 日志（保留未用）
   │  └─ MDK_V5/                 # Keil MDK 工程文件
   ├─ libraries/                 # CMSIS / AT32 标准驱动库
   └─ middlewares/               # FreeRTOS 源码
```

---

## 3. 控制架构

### 3.1 控制模式

`FOC.h` 中定义 4 种模式：

| 模式 | 值 | 说明 |
|------|-----|------|
| `FOC_OPEN_LOOP` | 0x01 | 开环：直接给定 Uq/Ud，无反馈 |
| `FPC_CURRENT_LOOP` | 0x02 | 电流环：Id/Iq PI 闭环 |
| `FOC_SPEED_LOOP` | 0x03 | 速度-电流串级控制 |
| `FOC_POSITION_LOOP` | 0x04 | 位置-速度-电流串级控制 |

### 3.2 控制链路

```
电流环（ADC 中断，10kHz~20kHz）：
  编码器角度 → 电角度修正 → 电流采样/重构
  → Clarke → Park → Id/Iq LPF → Id/Iq PI
  → 逆 Park → SVPWM → PWM 输出
  → SMO 同步观测（并行运行）

速度环（control_task，500Hz）：
  编码器角度差分 → 速度 LPF → 速度 PI → Iq 目标

位置环（control_task，100Hz）：
  角度积分 → 位置 PD → 速度目标
```

### 3.3 中断优先级与实时性

| 中断源 | 功能 | 频率 |
|--------|------|------|
| ADC1_2_IRQHandler | 电流采样 + FOC 主循环 | PWM 频率（10k~20kHz） |
| TMR2_GLOBAL_IRQHandler | 遥测数据发送 | 可配置 |
| USART3_IRQHandler | 命令帧接收 | 异步 |
| DMA1_Channel1_IRQHandler | USART3 DMA 发送完成 | 每次发送完成触发 |

### 3.4 FreeRTOS 任务

| 任务 | 栈大小 | 周期 | 功能 |
|------|--------|------|------|
| `comm_task` | 256 words | 5ms | 参数初始化 + 上位机命令处理 |
| `control_task` | 256 words | 2ms | 速度/位置外环控制 |
| `Monitor_task` | 128 words | 500ms | MOS 温度监控上传 |

---

## 4. SMO 滑模观测器（无传感器角度观测）

### 4.1 概述

SMO（Sliding Mode Observer）通过构建电流观测模型，利用滑模面来估计反电势，进而提取转子角度和速度。
当前实现将 SMO 作为**后台观测**（平行于编码器运行），不参与控制。可随时通过串口切换到 SMO 角度控制。

### 4.2 算法流程

```
电流估计误差计算：
  err = i_hat - i

滑模注入：
  z = k_slide * sat(err / band)

电流观测器（离散积分）：
  i_hat += Ts/Ls * (u - Rs*i_hat - e_hat - z)

反电势低通滤波：
  e += e_lpf_alpha * (z - e)

PLL 锁相环提取角度/速度：
  ε = (-eAlpha*cos(θ̂) - eBeta*sin(θ̂)) / |e|   （归一化误差）
  ω̂ = Kp*ε + ∫ Ki*ε*dt                         （PI 速度估计）
  θ̂ += ω̂*Ts                                    （角度积分）
```

### 4.3 PLL 相比 atan2 的优势

| 对比项 | atan2 + 差分求速 | PLL 锁相环 |
|--------|-----------------|------------|
| 角度提取 | 直接 atan2，对噪声敏感 | 闭环跟踪，天然滤波 |
| 速度计算 | 差分 → 低通滤波，有延迟 | PI 直接输出，延迟小 |
| 低速性能 | 信号幅值小，信噪比差 | 归一化后幅值无关 |
| 参数整定 | 速度 LPF 系数 | Kp/Ki 带宽控制 |

### 4.4 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Rs | 0.198 Ω | 电机相电阻 |
| Ls | 57 µH | 等效电感 (Lq+Ld)/2 |
| K_slide | 8.0 | 滑模增益 |
| e_lpf_alpha | 0.08 | 反电势低通滤波系数 |
| PLL_Kp | 200.0 | PLL 比例增益 |
| PLL_Ki | 50.0 | PLL 积分增益 |

参数通过上位机 `CMD_SETMOTORRS/LQ/LD` 在线调整，掉电保存至 Flash。

---

## 5. 串口通信协议

### 5.1 帧格式

**上位机 → 下位机（固定 8 字节）：**
```
[0xA5][CMD][DATA0][DATA1][DATA2][DATA3][CHECKSUM][0x49]
```
- 数据区固定为 1 个 float（4 字节）
- CHECKSUM = HEAD + CMD + DATA0~3

**下位机 → 上位机（可变长）：**
```
[0xA5][CMD][LEN][PAYLOAD...][CHECKSUM][0x49]
```
- LEN = count × 4（count 为 float 个数）
- PAYLOAD：float 字节流
- CHECKSUM：HEAD + CMD + LEN + PAYLOAD（逐字节累加）

### 5.2 命令分类

| 类别 | 命令举例 |
|------|---------|
| 连接与初始化 | `CMD_CONNECT_MOTOR` |
| 遥测开关 | `CMD_MECHANICALANGLE`, `CMD_UABC`, `CMD_SMO_ANGLE` 等 |
| 参数设置 | `CMD_SETPAIRS`, `CMD_SETDIR`, `CMD_SETUQ` 等 |
| 模式切换 | `CMD_OPEN_LOOP`, `CMD_CURRENT_LOOP`, `CMD_SPEED_LOOP`, `CMD_POSITION_LOOP` |
| PID 整定 | `CMD_SETIQPIDKP/KI`, `CMD_SETSPEEDPIDKP/KI`, `CMD_SETLOCALPIDKP/KD` |
| 标定校准 | `CMD_ZEROCALIBRATIO`（强拖 + 角度零偏标定）|
| SMO 调试 | `CMD_SMO_ANGLE`, `CMD_SMO_SPEED`, `CMD_SMO_BACKEMF` |
| 电机参数 | `CMD_SETMOTORRS/LQ/LD` |

完整的命令枚举见 `protocol.h`。

---

## 6. 参数持久化（Flash）

| 项目 | 值 |
|------|-----|
| 存储地址 | 0x0803F800 |
| 数据结构 | `foc_params_t`（含 Rs/Lq/Ld） |
| 校验方式 | CRC32 |
| 兼容性 | 支持旧版结构（无 Rs/Lq/Ld 字段）升级迁移 |

存储参数：
- `elec_offset`：零电角度偏移（标定值）
- `pole_pairs`：电机极对数
- `dir`：旋转方向（±1）
- `speeddir`：速度方向
- **`rs/lq/ld`**：电机模型参数（SMO 使用）

---

## 7. 外设与驱动映射

### 7.1 PWM 输出（TIM1）

| TIM1 通道 | 驱动信号 | 相 |
|-----------|---------|-----|
| CH1 | HIN3 | C 相上管 |
| CH1N | LIN3 | C 相下管 |
| CH2 | HIN2 | B 相上管 |
| CH2N | LIN2 | B 相下管 |
| CH3 | HIN1 | A 相上管 |
| CH3N | LIN1 | A 相下管 |
| CH4 | - | Brake / 附加输出 |

### 7.2 主要外设

| 外设 | 用途 |
|------|------|
| ADC1 | 三相电流 + 母线电压采样 |
| ADC2 | NTC 温度采样 |
| TMR1 | PWM 生成（6 路互补输出） |
| TMR2 | ADC 触发 + 遥测定时中断 |
| SPI1 | MT6701 磁编码器通信 |
| USART1 | printf 调试输出 |
| USART3 | 上位机通信（RX 中断 + TX DMA） |
| CAN1 | 预留扩展 |

---

## 8. 如何调试验证

### 8.1 基础调试

1. 连接 USART3 到 PC（推荐使用 USB 转串口模块）
2. 上位机发送 `CMD_CONNECT_MOTOR`（0x01）检查通信正常
3. 发送 `CMD_ZEROCALIBRATIO`（0x06）标定电角度零偏

### 8.2 SMO 观测调试

1. 电机运行后，发送 `CMD_SMO_ANGLE`（0x48）打开 SMO 角度遥测
2. 同时发送 `CMD_ELECTRICALANGLE`（0x51）对比编码器角度
3. 发送 `CMD_SMO_BACKEMF`（0x4C）查看反电势波形
4. 通过上位机调整 Rs/Lq/Ld 参数匹配电机模型

### 8.3 控制模式切换

1. 开环验证：`CMD_OPEN_LOOP` + 设置 Uq 观察电机转动
2. 电流闭环：`CMD_CURRENT_LOOP` + 设置目标 Iq
3. 速度闭环：`CMD_SPEED_LOOP` + 设置目标速度
4. 位置闭环：`CMD_POSITION_LOOP` + 设置目标位置

---

## 9. 快速三角函数

使用 Remez 算法生成的 5 阶多项式近似替代标准 `sinf`/`cosf`：

- 拟合精度：~1e-8（sin），~1e-8（cos）
- 性能：无分支预测惩罚，比标准库快数倍
- 原理：sin(x) = x + x³·f1(x²)，cos(x) = 1 + x²·f2(x²)

---

## 10. 构建与烧录

### Keil MDK5

```powershell
cd AT32F403ACCT7_WorkBench
.\build                    # 构建
.\flash                    # 烧录
.\flash -BuildFirst        # 构建 + 烧录
```

详细说明见：`AT32F403ACCT7_WorkBench/BUILD.md`。

### VS Code + CMake

项目同时支持 VS Code + CMake 构建（`atcmake` / `atmake` / `atflash` 脚本），
详情参考 `AT32F403ACCT7_WorkBench` 目录下的相关配置。

---

## 11. 配套上位机

推荐与仓库 [`lhzloveyyj/LiJointMaster`](https://github.com/lhzloveyyj/LiJointMaster) 配套使用。
上位机命令与本固件的 `protocol.h` 枚举保持对应，可进行：

- PID 参数在线整定
- 实时波形观测（角度、电流、反电势等）
- 电机参数设置
- SMO 角度对比观测

---

## 12. 注意事项

1. **SMO 参数整定**：Rs/Lq/Ld 必须与电机实际参数匹配，否则观测角度偏差大
2. **PLL 参数**：PLL Kp/Ki 根据转速范围和 PWM 频率调整
3. **启动流程**：先开环强拖到一定转速后，SMO 角速度稳定后方可切换闭环
4. **电流采样**：Ia 采样通道存在异常，当前使用 Ib/Ic 重构
5. **构建产物**：`MDK_V5/objects`、`listings` 等建议加入 `.gitignore`

---

## 13. License

MIT License
