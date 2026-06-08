# VESC FOC 架构深度分析

> 基于 VESC firmware v7.00 源码分析，聚焦有感/无感切换机制和多观测器架构，
> 为 Li_FOC 工程优化提供参考。

---

## 一、整体架构哲学：配置驱动 + 运行时切换

VESC 的核心设计原则是：**一切选择都是 `mc_configuration` 结构体的一个字段**，而非 `#ifdef` 编译时宏。

```c
// 运行时从 Flash 加载，也可以通过 VESC Tool 在线修改
volatile mc_configuration *m_conf;   // 指向当前配置

// 关键字段举例（来自 datatypes.h）
m_conf->foc_sensor_mode       // 传感器模式：0=Sensorless, 1=Encoder, 2=Hall, 3~8=HFI...
m_conf->foc_observer_type     // 观测器类型：0=Ortega, 1=MXLEMMING, 2~6=其他变体
m_conf->foc_observer_gain     // 观测器增益
m_conf->foc_observer_gain_slow // 低速增益缩放
m_conf->foc_pll_kp            // PLL Kp
m_conf->foc_pll_ki            // PLL Ki
m_conf->foc_sl_erpm           // 有感/无感切换阈值 ERPM
m_conf->foc_sl_erpm_hfi       // HFI/观测器切换阈值
// ... 共 60+ 个 FOC 相关可调参数
```

**对比你的 Li_FOC**：你用 `foc_config.h` 的 `#define` 宏来设置参数，每次改参数都需要重新编译。VESC 的做法是配置在 Flash 中持久化，可在运行时随时通过上位机修改并立即生效。

### 1.1 详细对比：宏 vs 运行时配置

这是两个项目最根本的架构差异，直接决定了扩展性和灵活性。

**你的 Li_FOC — 宏方式（编译时决定一切）**：

```c
// ============ foc_config.h ============
// 所有 FOC 参数都是预编译宏，改任何值必须重新编译固件
#define FOC_SMO_RS                  0.198f      // 电机相电阻
#define FOC_SMO_LS                  0.000057f   // 等效电感
#define FOC_SMO_K_SLIDE             30.0f       // 滑模增益
#define FOC_SMO_PLL_KP             800.0f       // PLL Kp
#define FOC_SMO_PLL_KI             80000.0f     // PLL Ki
#define FOC_SENSORLESS_IF_ALIGN_ID  1.50f       // I/F 对齐电流
#define FOC_SENSORLESS_IF_START_IQ  4.00f       // I/F 拖动电流
#define FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM 600.0f  // 切换转速
// ... 共 ~35 个 #define

// ============ FOC.c ============
// 传感器模式也只有 2 种，硬编码 if/else
if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORED) {
    // 读 MT6701 编码器 → 做 Park 变换
    pFOC->sensoredMechanicalAngle = Mt6701GetAngleWrapper();
    pFOC->sensoredCorrectedAngle = AngleGetCorrectedElec(...);
} else {
    // 跑 SMO 观测器 → 从 eAlpha/eBeta 提取角度
    SMO_Update(&g_smoObserver, ...);
}
```

问题：
- 换电机 → 改 R/L 宏 → 重编译 → 重烧录
- 加新传感器（如 Hall） → 改 FOC.c 主循环，嵌更多 if/else
- 想对比两种 PLL 参数 → 改宏 → 编译两次 → 烧两次
- 参数是代码的一部分，不是数据

**VESC — 结构体方式（运行时决定一切）**：

```c
// ============ datatypes.h ============
// FOC 所有参数是 mc_configuration 结构体的字段
typedef struct {
    float foc_motor_r;                    // 电机相电阻
    float foc_motor_l;                    // 电机电感
    float foc_motor_flux_linkage;         // 磁链
    float foc_motor_ld_lq_diff;           // Ld-Lq 差
    float foc_observer_gain;              // 观测器增益
    float foc_observer_gain_slow;         // 低速增益
    float foc_pll_kp;                     // PLL Kp
    float foc_pll_ki;                     // PLL Ki
    float foc_sl_erpm;                    // 有感/无感切换阈值
    float foc_sl_erpm_start;              // 启动切换阈值
    float foc_sl_erpm_hfi;                // HFI 切换阈值
    mc_foc_sensor_mode foc_sensor_mode;   // 传感器模式（枚举）
    mc_foc_observer_type foc_observer_type; // 观测器类型（枚举）
    SAT_COMP_MODE foc_sat_comp_mode;      // 饱和补偿模式
    bool foc_temp_comp;                   // 温度补偿开关
    float foc_fw_current_max;             // 弱磁最大电流
    MTPA_MODE foc_mtpa_mode;              // MTPA 模式
    // ... 共 60+ 个 FOC 字段，全部在运行时可变
} mc_configuration;

// ============ mcpwm_foc.c ============
// 传感器模式 9 种，switch 统一分发
switch (conf_now->foc_sensor_mode) {   // conf_now = motor->m_conf
case FOC_SENSOR_MODE_ENCODER:      // SPI 绝对编码器
    phase = foc_correct_encoder(obs, enc, speed, sl_erpm, motor);
    break;
case FOC_SENSOR_MODE_ENCODER_AB:   // ABI 增量编码器
case FOC_SENSOR_MODE_HALL:         // Hall 传感器
case FOC_SENSOR_MODE_SENSORLESS:   // 纯无感
case FOC_SENSOR_MODE_HFI:          // HFI V1
case FOC_SENSOR_MODE_HFI_V2:       // HFI V2
// ...
}

// 观测器类型 7 种，同样是 switch 分发
switch (conf_now->foc_observer_type) {
case FOC_OBSERVER_ORTEGA_ORIGINAL:  ...
case FOC_OBSERVER_MXLEMMING:        ...
case FOC_OBSERVER_MXV:              ...
// ...
}
```

好处：
- 改参数 → 上位机发一条命令 → 即刻生效，不动代码
- 加新传感器 → 加一个 `case` 分支 + 写一个新驱动文件即可，主循环结构不变
- 对比不同参数 → 上位机实时调参看效果，不需要反复编译
- 配置即是数据 → 序列化到 Flash（带 CRC），断电不丢失

### 1.2 配置如何传递到控制循环

```
┌─────────────────────────────────────────────────────────┐
│  VESC Tool (上位机)                                     │
│    用户选传感器模式、调参数                               │
└────────────┬────────────────────────────────────────────┘
             │ USB / CAN / UART
             ▼
┌─────────────────────────────────────────────────────────┐
│  comm/commands.c                                        │
│    解析命令包 → 写入 m_conf 对应字段                      │
│    同时触发 Flash 持久化（flash_helper.c）                │
└────────────┬────────────────────────────────────────────┘
             │
     ┌───────┴───────┐
     ▼               ▼
┌──────────┐  ┌──────────────────┐
│  Flash   │  │  m_conf 结构体    │
│  持久化   │  │  （RAM 中，      │
│  + CRC   │  │   运行时生效）    │
└──────────┘  └────────┬─────────┘
                       │ 每个 PWM 周期读取
                       ▼
┌─────────────────────────────────────────────────────────┐
│  mcpwm_foc.c  /  foc_math.c                             │
│    switch(m_conf->foc_sensor_mode)   选传感器            │
│    switch(m_conf->foc_observer_type) 选观测器            │
│    读 m_conf->foc_pll_kp 做 PLL 运算                    │
│    读 m_conf->foc_observer_gain 做增益调度               │
└─────────────────────────────────────────────────────────┘
```

### 1.3 三个层面"选择"的归属

容易混淆的是 VESC 中"谁决定用什么"，这里按自动化程度分三层：

```
层面 1：观测器算法选型（用户手动）
  ├── 7 种观测器 → 用户在 VESC Tool 下拉菜单选一个
  ├── 3 种饱和补偿 → 用户选
  ├── 2 种 MTPA 模式 → 用户选
  └── 存储在 m_conf->foc_observer_type，固件只负责 switch 执行
        → 固件不会自动换观测器算法

层面 2：传感器角度融合（固件自动，参数可配）
  ├── foc_correct_encoder() → 按转速 + 滞环自动切换编码器/观测器
  ├── foc_correct_hall()    → 按转速自动切换 Hall/观测器
  └── 切换阈值 sl_erpm 和滞环百分比用户可配，但切换动作是固件自动的
        → 这就是 VESC 真正"自动"的部分

层面 3：观测器增益（固件全自动）
  ├── m_gamma_now → 每个 PWM 周期根据占空比/电压自动计算
  ├── 温度补偿 → 根据温度传感器读数自动修正电阻
  └── 饱和补偿 → 根据电流大小自动修正电感/磁链
        → 用户只能开关和配基础参数，具体数值由固件实时计算
```

---

## 二、观测器架构：单入口 + switch 分发

### 2.1 观测器状态结构

观测器内部状态是统一的 `observer_state` 结构体（[foc_math.h:97-103](foc_math.h#L97-L103)）：

```c
typedef struct {
    float x1;               // α 轴磁链估计
    float x2;               // β 轴磁链估计
    float lambda_est;       // 磁链幅值估计（用于饱和补偿）
    float i_alpha_last;     // 上次 α 电流（MXLEMMING 观测器用）
    float i_beta_last;      // 上次 β 电流（MXLEMMING 观测器用）
} observer_state;
```

所有 7 种观测器共用同一状态结构体，只是内部更新公式不同。

### 2.2 观测器分发：运行时 switch，非编译时条件

每个 PWM 周期，`foc_observer_update()` 被调用一次（[foc_math.c:25](foc_math.c#L25)）：

```c
void foc_observer_update(float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float dt, observer_state *state,
                         float *phase, motor_all_state_t *motor) {

    mc_configuration *conf_now = motor->m_conf;

    // === 预处理：饱和补偿、温度补偿、凸极性 ===
    float R = conf_now->foc_motor_r;
    float L = conf_now->foc_motor_l;
    float lambda = conf_now->foc_motor_flux_linkage;

    // 1. 饱和补偿（3 种模式）
    switch(conf_now->foc_sat_comp_mode) { ... }

    // 2. 温度补偿
    if (conf_now->foc_temp_comp) {
        R = motor->m_res_temp_comp;    // 实时温度补偿后的电阻
    }

    // 3. 凸极性电感修正
    if (fabsf(id) > 0.1 || fabsf(iq) > 0.1) {
        L = L - ld_lq_diff/2.0 + ld_lq_diff * SQ(iq)/(SQ(id) + SQ(iq));
    }

    // === 观测器本体：switch 选择算法 ===
    switch (conf_now->foc_observer_type) {

    case FOC_OBSERVER_ORTEGA_ORIGINAL:
        // 原始 Ortega 磁链观测器
        // http://cas.ensmp.fr/...ObserverPermanentMagnet.pdf
        {
            float err = SQ(lambda) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));
            if (err > 0.0) err = 0.0;  // 强制误差项为非正，帮助收敛
            state->x1 += (v_alpha - R_ia + gamma_half*(state->x1 - L_ia)*err) * dt;
            state->x2 += (v_beta - R_ib + gamma_half*(state->x2 - L_ib)*err) * dt;
        }
        break;

    case FOC_OBSERVER_MXLEMMING:
        // MXLEMMING 观测器：用电流差替代复杂的非线性误差项
        state->x1 += (v_alpha - R_ia)*dt - L*(i_alpha - state->i_alpha_last);
        state->x2 += (v_beta - R_ib)*dt - L*(i_beta - state->i_beta_last);
        break;

    case FOC_OBSERVER_MXLEMMING_LAMBDA_COMP:
        // MXLEMMING + 在线磁链估计（λ补偿）
        // 磁链估计：λ_dot = 0.1 * γ/2 * λ * -err * dt
        break;

    case FOC_OBSERVER_MXV:           // MXV 观测器
    case FOC_OBSERVER_MXV_LAMBDA_COMP:
    case FOC_OBSERVER_MXV_LAMBDA_COMP_LIN:
        // MXV 变体：不同的数值方案，对低速/高速有不同的权衡
        break;
    }

    // === 后处理：从磁链提取角度 ===
    if (phase) {
        *phase = utils_fast_atan2(state->x2, state->x1);
    }

    state->i_alpha_last = i_alpha;  // MXLEMMING 保存电流用于下次
    state->i_beta_last = i_beta;
}
```

**关键设计**：
- 7 种观测器共用一个函数入口，参数完全相同
- `switch` 分发在函数内部，不是通过函数指针或多态
- 每种观测器更新的是同一个 `observer_state`（x1/x2），语义完全相同
- 上层调用者无需关心当前用的是哪种观测器

**对比你的 Li_FOC**：你只有一个 SMO 观测器（`SMO_Update`），可以直接拓展为类似的 switch 分发结构。

---

## 三、传感器切换机制

### 3.1 核心原则：观测器永远在跑

VESC 的观测器 (`foc_observer_update`) **在任何传感器模式下都在运行**。切换传感器模式只是改变 "用谁的相位做 FOC 控制"，而不是"跑不跑观测器"。

```
┌────────────────────────────────────────────────────┐
│                 每个 PWM 周期                        │
├────────────────────────────────────────────────────┤
│                                                    │
│  1. 更新观测器 m_phase_now_observer （始终运行）      │
│                            ↓                        │
│  2. 读编码器数据 m_phase_now_encoder （始终更新）     │
│  3. 读 Hall 传感器                  （始终更新）     │
│  4. 跑 HFI 高频注入                 （始终更新）     │
│                            ↓                        │
│  5. switch(foc_sensor_mode) → 选一个相位             │
│                            ↓                        │
│  6. Park / 逆 Park 变换使用选定的相位                │
│                                                    │
└────────────────────────────────────────────────────┘
```

### 3.2 传感器模式切换代码（[mcpwm_foc.c:3457](mcpwm_foc.c#L3457)）

```c
switch (conf_now->foc_sensor_mode) {

// ────── 绝对编码器 ──────
case FOC_SENSOR_MODE_ENCODER:
    // 索引找到：编码器 + 观测器混合，按 sl_erpm 切换
    state_now->phase = foc_correct_encoder(
        motor_now->m_phase_now_observer,   // 观测器角度（高速用）
        motor_now->m_phase_now_encoder,    // 编码器角度（低速用）
        motor_now->m_speed_est_fast,       // 当前速度
        conf_now->foc_sl_erpm,             // 切换阈值
        motor_now);
    break;

// ────── ABI 增量编码器（无索引信号） ──────
case FOC_SENSOR_MODE_ENCODER_AB:
    // 低速：开环拖动（observer_override）→ 速度上来后先与观测器同步
    // 索引建立后：同绝对编码器，低速编码器 → 高速观测器
    if (encoder_index_found()) {
        state_now->phase = foc_correct_encoder(...);   // 混合
    } else {
        state_now->phase = m_phase_now_observer;        // 纯观测器
    }
    break;

// ────── Hall 传感器 ──────
case FOC_SENSOR_MODE_HALL:
    state_now->phase = foc_correct_hall(
        motor_now->m_phase_now_observer,  // 观测器角度
        dt, motor_now,
        hall_val);                        // 当前 Hall 状态
    // 低于 sl_erpm 用 Hall 插值角度，高于 sl_erpm 用观测器
    break;

// ────── Sensorless 纯无感 ──────
case FOC_SENSOR_MODE_SENSORLESS:
    // 开环阶段用观测器 override，闭环后用观测器
    if (motor_now->m_phase_observer_override) {
        state_now->phase = m_phase_now_observer_override;
    } else {
        state_now->phase = motor_now->m_phase_now_observer;
    }
    break;

// ────── HFI 高频注入 ──────
case FOC_SENSOR_MODE_HFI:    // V1~V5 共 6 种 HFI
    // 低于 foc_sl_erpm_hfi：用 HFI 注入角度（foc_correct_encoder 混合）
    // 高于 foc_sl_erpm_hfi：切回观测器
    state_now->phase = foc_correct_encoder(
        motor_now->m_phase_now_observer,  // 高速
        motor_now->m_hfi.angle,           // HFI 角度（低速/零速）
        motor_now->m_speed_est_fast,
        conf_now->foc_sl_erpm_hfi,        // HFI/观测器切换阈值
        motor_now);
    break;

case FOC_SENSOR_MODE_HFI_START:
    // HFI 仅在启动阶段使用，启动成功后切回纯观测器
    break;
}
```

### 3.3 核心混合函数 `foc_correct_encoder`（[foc_math.c:572](foc_math.c#L572)）

这是 VESC 有感和无感之间切换的核心：

```c
float foc_correct_encoder(float obs_angle, float enc_angle, float speed,
                          float sl_erpm, motor_all_state_t *motor) {
    float rpm_abs = fabsf(RADPS2RPM_f(speed));

    // 5% 滞环防止抖动
    float hyst = sl_erpm * 0.05;

    if (motor->m_using_encoder) {
        // 当前在用编码器 → 速度超过 sl_erpm+hyst 才切到观测器
        if (rpm_abs > (sl_erpm + hyst)) {
            motor->m_using_encoder = false;
        }
    } else {
        // 当前在用观测器 → 速度低于 sl_erpm-hyst 才切回编码器
        if (rpm_abs < (sl_erpm - hyst)) {
            motor->m_using_encoder = true;
        }
    }

    return motor->m_using_encoder ? enc_angle : obs_angle;
}
```

**设计要点**：
1. **硬切换 + 滞环**：不是渐进融合（fade/blend），而是在 `sl_erpm` 附近用一个带滞环的状态机做二选一切换
2. **滞环防止抖动**：5% 滞环确保转速在阈值附近来回波动时不会反复切换
3. **状态记忆**：`m_using_encoder` 是持久状态，切换后保持直到条件再次满足
4. **观测器一直在跑**：切换到观测器的一瞬间，观测器早已收敛（因为一直在后台运行），不会有突变

### 3.4 Hall 传感器混合 `foc_correct_hall`（[foc_math.c:591](foc_math.c#L591)）

```c
float foc_correct_hall(float angle, float dt, motor_all_state_t *motor, int hall_val) {
    float rpm_abs = fabsf(RADPS2RPM_f(motor->m_pll_speed));
    motor->m_using_hall = rpm_abs < conf_now->foc_sl_erpm;
    // 低于 sl_erpm：用 Hall 表查表插值角度
    // 高于 sl_erpm：用观测器角度
}
```

Hall 模式比编码器模式多了一步：用 Hall 状态跃迁来修正观测器累积误差，具体是通过 `hall_table[hall_val]` 查表获取预设的 Hall 角度偏移。

---

## 四、无感开环启动（Sensorless Openloop Startup）

### 4.1 你的 I/F 启动 vs VESC 的定时开环

| 特性 | Li_FOC (I/F) | VESC (定时开环) |
|------|-------------|----------------|
| 对齐方式 | Id 电流注入定角度 | 同 |
| 拖动方式 | Iq 电流 + 虚拟角度积分 | Iq 电流 + 虚拟角度积分 |
| 拖动时长 | 固定周期数 ALIGN_COUNT | 可配置三段时间：Lock/Ramp/Const |
| 交接条件 | 5 条条件，全部满足且持续 LOCK_COUNT 周期 | 定时器到期自动切 |
| 超时重试 | RAMP 60k 周期超时重试 | 定时器设计保证平滑交接 |

### 4.2 VESC 的三段式开环定时

```c
// 来自 mcpwm_foc.c:4026
float t_lock = conf_now->foc_sl_openloop_time_lock;  // 锁定时间（slow rpm）
float t_ramp = conf_now->foc_sl_openloop_time_ramp;  // 斜坡上升时间
float t_const = conf_now->foc_sl_openloop_time;      // 恒定高速运行时间

// m_min_rpm_timer 从 (t_lock + t_ramp + t_const) 倒计时到 0
// 倒计时期间 phase_observer_override = true → 用预计算的虚拟角度
// 倒计时结束 → phase_observer_override = false → 切换到观测器
```

### 4.3 观测器初始化：离开开环时的交接

VESC 在开环期间就把观测器状态预置好了，保证切换到观测器时零冲击：

```c
// mcpwm_foc.c:4112-4116
// 将观测器 x1/x2 强制置为预先计算好的磁链分量
utils_fast_sincos_better(
    m_phase_now_observer_override + SIGN(duty_now) * M_PI/4.0,
    &s, &c);
m_observer_x1_override = c * conf_now->foc_motor_flux_linkage;
m_observer_x2_override = s * conf_now->foc_motor_flux_linkage;
```

**对比你的 Li_FOC**：你的 I/F 交接只设置 `speedPID.lastBias` 和清零 `iqPID.lastBias`，但没有预设 SMO 内部状态 (`iAlphaHat`, `iBetaHat`, `eAlpha`, `eBeta`)。这导致交接瞬间 SMO 需要若干周期才能收敛到正确的反电势，期间角度可能有偏差。

---

## 五、观测器增益调度（Gain Scheduling）

这是 VESC 的另一个关键设计：观测器增益不是常数，而是根据**母线电压和占空比动态调整**。

```c
// mcpwm_foc.c:4134-4143
// 观测器增益基于占空比缩放
float gamma_tmp = utils_map(
    fabsf(motor->m_motor_state.duty_now),    // 当前占空比
    0.0,                                       // 占空比低端
    40.0 / motor->m_motor_state.v_bus,         // 占空比高端（40V / 母线电压）
    0,                                          // 增益低端
    conf_now->foc_observer_gain);               // 增益高端

// 低速保底增益
if (gamma_tmp < (conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain)) {
    gamma_tmp = conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain;
}

motor->m_gamma_now = gamma_tmp * 4.0;
```

**物理意义**：
- 低占空比/低电压时，反电势信号弱，需要**低增益**避免噪声放大
- 高占空比/高电压时，反电势信号强，可以用**高增益**提高跟踪速度
- `foc_observer_gain_slow`（默认约 0.5）保证低速时增益不会过低导致跟踪不上

**对比你的 Li_FOC**：你的 SMO 增益 `FOC_SMO_K_SLIDE=30.0` 是固定值，全转速范围不变。

---

## 六、Field Weakening（弱磁调速）

VESC 的弱磁控制独立为一个函数 `foc_run_fw`（[foc_math.c:702](foc_math.c#L702)）：

```c
void foc_run_fw(motor_all_state_t *motor, float dt) {
    // 当调制深度超过 foc_fw_duty_start 阈值（如 0.95）时
    // 注入负的 Id 电流来降低反电势
    // 负 Id 用掉了一部分电压裕量，让正 Iq 能继续输出转矩
    // 带有 backoff 机制：当 Iq 失控时自动减少 FW 深度
}
```

**对比你的 Li_FOC**：你没有弱磁功能。你的电机在 12V 母线电压下，极对数 11，高速时反电势很容易超出母线电压，导致转速上不去。

---

## 七、电机状态结构

VESC 用 `motor_all_state_t` 维护每个电机的全部运行时状态（[foc_math.h:138](foc_math.h#L138)）：

```c
typedef struct {
    mc_configuration *m_conf;              // 指向配置
    mc_state m_state;                      // OFF/DETECTING/RUNNING/FULL_BRAKE
    mc_control_mode m_control_mode;        // DUTY/SPEED/CURRENT/POSITION/...

    motor_state_t m_motor_state;           // 电气瞬时量（vd,vq,id,iq,phase等）

    float m_phase_now_observer;            // ★ 观测器估算角度
    float m_phase_now_observer_override;   // ★ 开环覆盖角度
    bool  m_phase_observer_override;       // ★ 是否开环
    float m_phase_now_encoder;             // ★ 编码器角度
    float m_phase_now_encoder_no_index;    // 无索引时的开环编码器角度

    observer_state m_observer_state;       // 观测器内部状态
    float m_pll_phase;                     // PLL 锁定角度
    float m_pll_speed;                     // PLL 估计速度 (rad/s)

    hfi_state_t m_hfi;                     // HFI 内部状态

    float m_gamma_now;                     // 当前观测器增益
    float m_res_temp_comp;                 // 温度补偿后的电阻值
    float p_lq, p_ld;                      // 预计算的 d/q 轴电感（含凸极）
    // ...
} motor_all_state_t;
```

---

## 八、上位机通信与调试数据设计：为什么不需区分观测器类型

这是你问到的关键问题：不同观测器的内部变量不同（SMO 有 eAlpha/eBeta/zAlpha/slideError，Ortega 有 x1/x2/err/lambda_est），上位机怎么知道该显示什么？VESC 的答案是：**所有观测器向上位机暴露同一套接口，不需要区分**。

### 8.1 VESC 的上位机数据分两层

**第一层：固定字段的 `mc_values` 结构体（[datatypes.h:1359](datatypes.h#L1359)）**

```c
typedef struct {
    float v_in;              // 输入电压
    float temp_mos;          // MOS 温度
    float temp_motor;        // 电机温度
    float current_motor;     // 电机电流
    float current_in;        // 输入电流
    float id;                // d 轴电流
    float iq;                // q 轴电流
    float rpm;               // 转速
    float duty_now;          // 当前占空比
    float amp_hours;         // 安时
    float watt_hours;        // 瓦时
    int   tachometer;        // 里程计
    float position;          // 位置
    mc_fault_code fault_code; // 故障码
    float vd;                // d 轴电压
    float vq;                // q 轴电压
} mc_values;
```

这是每次上位机请求 `COMM_GET_VALUES` 时固件返回的数据。注意它**不包含任何观测器内部变量**——不区分观测器类型、不暴露 x1/x2/z/eMag。切换观测器对这条数据通路完全无影响，上位机显示永远一致。

**第二层：通用 Scope/Plot 系统（[commands.c:1987](comm/commands.c#L1987)）**

```c
// 初始化一个示波器窗口，指定 X/Y 轴标签
void commands_init_plot(const char *namex, const char *namey);

// 添加一条波形通道，指定通道名称
void commands_plot_add_graph(const char *name);

// 选中当前写入的通道
void commands_plot_set_graph(int graph);

// 发送一对 XY 数据点
void commands_send_plot_points(float x, float y);
```

使用示例（来自 [mcpwm_foc.c:4012](mcpwm_foc.c#L4012)）：

```c
// 初始化一个 "X1 vs X2" 的示波器窗口，包含两条曲线
commands_init_plot("X1", "X2");
commands_plot_add_graph("Observer");          // 通道 0
commands_plot_add_graph("Observer Mag");      // 通道 1

// 发送数据
commands_plot_set_graph(0);
commands_send_plot_points(m_observer_x1, m_observer_x2);   // X1-X2 Lissajous 图
commands_plot_set_graph(1);
commands_send_plot_points(0.0, NORM2_f(x1, x2));          // 磁链幅值
```

**关键设计**：Plot 系统的通道名称是**字符串**，上位机只负责把波形渲染出来，完全不关心数据来自哪个观测器。固件端想发什么就发什么，上位机不需要知道 "X1" 是什么物理量。

### 8.2 为什么观测器切换不需要改调试接口

根本原因是：**所有观测器输出的是同一个物理量——αβ 坐标系下的磁链矢量**。

```
         ┌──────────────────────────────────┐
         │  观测器（任意一种算法）             │
         │                                  │
         │  输入: v_alpha, v_beta,          │
         │        i_alpha, i_beta           │
         │                                  │
         │  输出: x1 (α 轴磁链 ≈ λ·cosθ)    │
         │        x2 (β 轴磁链 ≈ λ·sinθ)    │
         │        phase = atan2(x2, x1)     │
         │        mag   = sqrt(x1² + x2²)   │
         └──────────────────────────────────┘
```

不管你用 SMO 的 `eAlpha/eBeta` 还是 Ortega 的 `x1/x2`，它们都代表同一个东西：转子永磁体磁链在定子 αβ 坐标系下的投影。如果观测器工作正常，`(x1, x2)` 在 X-Y 坐标系下的轨迹应该是一个圆（Lissajous circle），圆的半径等于磁链幅值 λ。

所以调试接口只需要发两样：
1. **`x1-x2` Lissajous 图** → 看是不是圆（观测器是否收敛）
2. **磁链幅值** → 看是否稳定在预期值附近（约等于 `foc_motor_flux_linkage`）

这两样对所有观测器通用。

### 8.3 对比你的 Li_FOC

你目前的调试方式是 `printf` 打印，没有结构化的上位机数据通道。如果要借鉴 VESC 的设计：

```c
// 你的 SMO 输出：
smo->eAlpha, smo->eBeta       // 反电势（非磁链，差一个积分关系）
smo->zAlpha, smo->zBeta       // 滑模注入量（高频开关信号）
smo->iAlphaHat, smo->iBetaHat // 电流估计
smo->eMag                     // 反电势幅值
smo->pllError                 // PLL 误差
smo->rawAngle, smo->angle     // 原始/滤波角度
smo->speed                    // PLL 估计速度

// VESC 只发给上位机：
x1, x2, sqrt(x1²+x2²)         // 磁链矢量 + 幅值
```

把 SMO 的 `eAlpha/eBeta` 也组织成等价形式就能和 VESC 统一——它们的物理意义一致。加新观测器时只需保证输出同一个结构体，上位机完全不需要改。

### 8.4 调试数据的三种发送策略对比

| 策略 | VESC 用的 | 是否区分观测器 |
|------|----------|--------------|
| **通用实时数据** (`mc_values`) | ✓ | 否，固定 20 个字段 |
| **Scope/Plot 波形** (`COMM_PLOT_*`) | ✓ | 否，字符串通道名，通用 XY 协议 |
| **浓缩统计** (`setup_values`) | ✓ | 否，只发累积量 (Ah/Wh) |
| **观测器内部变量透传** | ✗ 不做 | — |

上位机 VESC Tool 里的示波器页面**不关心**当前是什么传感器模式或观测器类型。它只是把固件发来的 `(x, y)` 数据对画成波形。不同观测器的不同内部变量如果想看，需要固件端显式调用 `commands_send_plot_points` 把它们发出去——但 VESC 目前也只发了 x1/x2/mag，没有再细分。

---

## 九、电流采样架构详解

你的问题是"电流小电机可以，大电机波形很差，怀疑是反电动势造成的"。VESC 的电流采样针对这个问题做了多层处理。

### 9.1 你的电流采样 vs VESC 的电流采样

**你的 Li_FOC（[FOC.c:822](foc.c#L822)）**：

```c
// 步骤 2-3：读 ADC → 减零点偏置 → 直接算出三相电流
pFOC->current.adA = g_motorAdValues[0];
pFOC->Ia = (pFOC->current.adA - pFOC->current.voltageAOffset)
           / 4096.0f * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
// B, C 相同理...

// 步骤 4：单电阻采样时根据 SVPWM 扇区补全第三相
CurrentReconstruction(g_pMotor, PSVpwm, pFOC->Ia, pFOC->Ib, pFOC->Ic);
clarke_transform(-pFOC->Ia, -pFOC->Ib, ...);
```

问题：
1. **扇区重构逻辑有误**：你的 `CurrentReconstruction` 中，扇区 2/3 都重构 Ia，扇区 5/6 都重构 Ib/Ic，但没有考虑当前 PWM 占空比是否真的允许测到该相电流
2. **无低通滤波**：电流直接入 Park 变换，然后才在 Id/Iq 上做一道 LPF——此时噪声已经被 Park 变换"旋转"到 dq 坐标了，频率成分已改变
3. **单电阻采样 + 大电机高反电势**：大电机需要的调制深度接近 100%，此时某相 PWM 导通时间极短（接近 0 或 1），ADC 采样窗口太窄，读到的电流值不可靠

**VESC**：

```c
// 1. 读 ADC 原始值 → 减偏移（偏移值可在运行时校准）
curr0 = GET_CURRENT1();
curr1 = GET_CURRENT2();
curr2 = GET_CURRENT3();
curr0 -= conf_now->foc_offsets_current[0];  // 每相独立偏移
curr1 -= conf_now->foc_offsets_current[1];
curr2 -= conf_now->foc_offsets_current[2];

// 2. 三相不平衡检测（硬件传感器故障诊断）
motor_now->m_curr_unbalance = curr0 + curr1 + curr2;  // 应接近 0

// 3. 缩放为实际电流值（每相独立标定系数）
curr0 *= FAC_CURRENT1;
curr1 *= FAC_CURRENT2;
curr2 *= FAC_CURRENT3;

// 4. 双电阻板：补全第三相
curr2 = -(curr0 + curr1);

// 5. ★ 关键：根据 PWM 占空比选择可信任的电流相 ★
//    （见 9.2 节）
```

### 9.2 电流采样模式选择（反电动势问题的核心解决方案）

VESC 有 4 种电流采样模式，在 `mc_configuration.foc_current_sample_mode` 中配置，每个 PWM 周期都根据实际情况动态决定信任哪相电流。这是解决大电机高调制时电流波形差的关键机制。

#### 模式 1：`LONGEST_ZERO`（默认推荐，[mcpwm_foc.c:3096](mcpwm_foc.c#L3096)）

```c
#define SHUNT_PICK_THR 900  // PWM 周期内的阈值

// 检查哪相 PWM 最接近 0（低侧导通时间最长 = 采样窗口最宽）
// V7 硬件（相电流采样）：找 CCR 最小的相
if (tim->CCR1 < SHUNT_PICK_THR ||
    tim->CCR2 < SHUNT_PICK_THR ||
    tim->CCR3 < SHUNT_PICK_THR) {

    // 选 CCR 最小 → 该相电流采样时间最长 → 最可靠
    if (tim->CCR1 < tim->CCR2 && tim->CCR1 < tim->CCR3) {
        curr0 = -(curr1 + curr2);  // 用另外两相的值反推这一相
    } else if (tim->CCR2 < tim->CCR1 && tim->CCR2 < tim->CCR3) {
        curr1 = -(curr0 + curr2);
    } else {
        curr2 = -(curr0 + curr1);
    }
}

// V6 硬件（低侧采样）：找 CCR 最大的相
// (因为低侧导通时才有电流经过采样电阻)
if (tim->CCR1 > (tim->ARR - SHUNT_PICK_THR) || ...) {
    // 选 CCR 最大 → 该相低侧导通时间最长 → 最可靠
    ...
}
```

**物理含义**：当某相 PWM 占空比太极端（接近 0% 或接近 100%），该相的 ADC 采样窗口过窄，读值不可信。此时宁可丢掉该相直接用 Kirchhoff 定律从另两相推算，也不要用噪声数据。

#### 模式 2：`HIGH_CURRENT`（大电流用，[mcpwm_foc.c:3080](mcpwm_foc.c#L3080)）

```c
// 取幅值最小的两相电流，推算出幅值最大的那一相
// 目的：避免 ADC 饱和/运放非线性区
const float i0_abs = fabsf(curr0);
const float i1_abs = fabsf(curr1);
const float i2_abs = fabsf(curr2);

if (i0_abs > i1_abs && i0_abs > i2_abs) {
    curr0 = -(curr1 + curr2);  // 最大电流相用推算值替代
} else if (i1_abs > i0_abs && i1_abs > i2_abs) {
    curr1 = -(curr0 + curr2);
} else if (i2_abs > i0_abs && i2_abs > i1_abs) {
    curr2 = -(curr0 + curr1);
}
```

**物理含义**：当你推大电流时，电流最大的那相可能已经超出了运放的线性范围或接近 ADC 饱和。用另两个小电流相加推算它，虽然损失一点精度，但不会出现削顶失真。

#### 模式 3：`BEST_SENSOR`（实验性，[mcpwm_foc.c:3142](mcpwm_foc.c#L3142)）

利用当前相位角预测下一拍各相的期望电流值，选出测量值和预测值最接近的相。当前代码被 `#if 0` 注释掉了。

#### 模式 4：`ALL_SENSORS`（三电阻板）

三相全部直接使用，不做额外处理。仅在 `HW_HAS_3_SHUNTS` 的硬件上有效。

### 9.3 V0/V7 双向量采样（控制采样模式）

除了电流选择模式，还有控制采样模式 `foc_control_sample_mode`（[datatypes.h:72](datatypes.h#L72)）：

```
FOC_CONTROL_SAMPLE_MODE_V0          = 0   // 仅在 V0 向量采样
FOC_CONTROL_SAMPLE_MODE_V0_V7       = 1   // V0 和 V7 向量都采样（双倍频率）
FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL = 2 // V0+V7 + 角度插值
```

- **V0**：每个 PWM 周期采样一次（在 V0 零向量时），采样率 = 开关频率
- **V0_V7**：每个 PWM 周期采样两次（V0 和 V7 零向量），采样率 = 开关频率 × 2
- **V0_V7_INTERPOL**：V0+V7 双采样 + 第二个电机的角度插值（双电机用），保证两个电机的相位对齐到同一个时间点

对于大电机/高转速，V0_V7 模式能让电流采样频率翻倍，在有相位电流采样电阻的 V7 硬件上效果显著。

### 9.4 电流滤波器链

VESC 在信号链上布置了多层滤波（[mcpwm_foc.c:3239](mcpwm_foc.c#L3239)，[3777](mcpwm_foc.c#L3777)）：

```
ADC 原始值
  │
  ├─→ 偏移校准
  ├─→ 标定系数
  ├─→ 采样模式选择（丢弃不可信相）
  │
  ▼
i_alpha, i_beta ──→ Park 变换 ──→ id, iq
                                      │
  ┌───────────────────────────────────┤
  │ 第1层：LPF_FOC_FAST(id, iq, const=0.005)  ← 开关频率的 0.5%，非常轻
  │
  │ 第2层：id_filter, iq_filter   ← 上位机可读的滤波值
  │    UTILS_LP_FAST(state_now->vd, vd_tmp, 0.2)
  │    UTILS_LP_FAST(state_now->vq, vq_tmp, 0.2)
  │
  │ 第3层：i_abs_filter          ← 幅值滤波，用于增益调度和弱磁
  │    UTILS_LP_FAST(state_now->i_abs_filter, i_abs, 0.03)
  │
  │ 第4层：duty_filtered / mod_q_filter ← 调制深度滤波
  │    UTILS_LP_FAST(motor->m_duty_abs_filtered, duty_abs, 0.01)
  │    UTILS_LP_FAST(state_now->mod_q_filter, mod_q, 0.2)
  │
  │ 第5层：母线电压滤波
  │    UTILS_LP_FAST(state_now->v_bus, GET_INPUT_VOLTAGE(), 0.1)
```

LPF 宏的实现（[utils_math.h:100](util/utils_math.h#L100)）：
```c
#define UTILS_LP_FAST(value, sample, filter_constant) \
    (value -= (filter_constant) * ((value) - (sample)))
```

这是标准的一阶 IIR 低通滤波器：`y[n] = y[n-1] + α * (x[n] - y[n-1])`

### 9.5 电流偏移自动校准

VESC 在每次启动时会自动测量电流 ADC 的零点偏移（[mcpwm_foc.c:2617](mcpwm_foc.c#L2617)）：

```c
// 电机停转时多次采样取平均，作为三相电流零偏
m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;
```

**对比你的 Li_FOC**：你的 `getAdoffset()` 功能类似，但只做了 16 次采样的简单平均，而且是全局变量不是配置字段。

### 9.6 电流采样中的 i_alpha/i_beta 偏移补偿

VESC 在电机释放后重新开始驱动时，会对第一次采样做偏移补偿（[mcpwm_foc.c:3296](mcpwm_foc.c#L3296)）：

```c
// 电机释放（duty=0）后的第一次采样可能带有残余偏移
// 取当前采样和下一个采样的平均值来消除
if (motor_now->m_i_alpha_beta_has_offset) {
    state_now->i_alpha = 0.5 * (state_now->i_alpha + motor_now->m_i_alpha_sample_next);
    state_now->i_beta  = 0.5 * (state_now->i_beta  + motor_now->m_i_beta_sample_next);
    motor_now->m_i_alpha_beta_has_offset = false;
}
```

### 9.7 电流不平衡检测（三相传感器故障诊断）

```c
#ifdef HW_HAS_3_SHUNTS
    // Kirchhoff 定律：Ia + Ib + Ic 应恒为 0
    motor_now->m_curr_unbalance = curr0 + curr1 + curr2;
#endif
```

这个值持续偏离 0 意味着某相采样电路有硬件故障。

### 9.8 针对你的问题的具体建议

你大电机时电流波形差，根因很可能是：

1. **调制深度高时采样窗口窄**：大电机在 12V 母线电压下转速一高，调制深度就接近 100%，某相 PWM 导通时间极短（几微秒），你的 `CurrentReconstruction` 只看 SVPWM 扇区决定重构哪相，但实际上同一个扇区内某相也可能因为调制太深而无法可靠采样。**建议做 VESC 的 CCR 阈值判断**。

```c
// 改 CurrentReconstruction：不只看扇区，还要检查 PWM CCR 是否太极端
if (tmr_channel_value_get(TMR1, CH1) > PWM_PERIOD - 900) {
    // A 相采样窗口太窄，用 B/C 反推 A
    pFOC->Ia = -(pFOC->Ib + pFOC->Ic);
}
```

2. **缺少 ABC 域的电流滤波**：噪声在 Clarke 变换后混入 αβ 域，再做 Park 后已被"调制"成不同频率。**ABC 端先做轻量 LPF 或者在采样模式选择之前直接丢弃明显不可靠的数据点**。

3. **低调制时没有自动切回直接采样**：VESC 的 `SHUNT_PICK_THR=900` 是动态的——当所有相都在安全窗口内时，三相全部直接使用不做重构。



### 8.5 总结

VESC 的设计哲学是：**观测器对外的接口统一（都是磁链矢量），调试数据不随观测器类型变化**。这不是因为不同观测器的内部变量都一样，而是因为用统一的上层抽象（磁链 x1/x2）就足够判断观测器是否正常工作了。如果开发者需要深入调试某种特定观测器，可以通过 scope 系统手动添加临时通道，但这不在默认数据流中。

---

## 十、VESC 有感模式专项对比 —— 聚焦 MT6701 编码器场景

你说先把有感调完美。这里聚焦 VESC 在纯有感模式（编码器）下做得比你的 Li_FOC + MT6701 更好的地方，不涉及无感/观测器/HFI。

### 10.1 编码器配置链：你的 vs VESC

**你的 Li_FOC**：

```c
// 只有两个参数：
float zeroOffset;       // 零位偏移
int   dir;              // 方向（1 或 -1）

// 角度处理：
mechanicalAngle = Mt6701GetAngleWrapper();              // 读 SPI → 机械弧度
elecAngle = CalculateElectricalAngle(mechanicalAngle);   // 极对数换算
correctedAngle = elecAngle - zeroOffset;                 // 减零偏
```

**VESC**：

```c
// 三个独立参数，各司其职：
float foc_encoder_offset;     // 零位偏移（度）
float foc_encoder_ratio;      // 减速比（如 7:1 齿轮箱 → 编码器转 7 圈 = 电机转 1 圈）
bool  foc_encoder_inverted;   // 方向反转（编码器 CW 对应电机 CW？）

// 角度处理（每个 PWM 周期，mcpwm_foc.c:3260）：
phase_tmp = enc_ang;                          // 读编码器原始度数
if (foc_encoder_inverted) {
    phase_tmp = 360.0 - phase_tmp;            // 方向补偿
}
phase_tmp *= foc_encoder_ratio;               // 齿轮比补偿
phase_tmp -= foc_encoder_offset;              // 零位补偿

// 然后才做非线性误差修正：
if (enc_corr_en) {
    phase_tmp -= enc_corr[(int)phase_tmp];    // 每度查表修正
}

motor->m_phase_now_encoder = DEG2RAD_f(phase_tmp);
```

**关键差异**：
- VESC 的 `foc_encoder_ratio` 可以处理编码器装在减速器输出端的情况（电机转 7 圈，编码器转 1 圈），你的代码做不到
- VESC 有 360 点的非线性误差修正表，你的 MT6701 未做任何线性度补偿
- 所有参数是 `m_conf` 字段，运行时随时可调

### 10.2 编码器零位自动标定

#### 你的方式：单点强拖（[FOC.c:437](FOC.c#L437)）

```
原理：Ud 电压注入在 d 轴 0° 方向 → 转子被电磁力拖到 d 轴对齐 → 读编码器

步骤：
1. 关 ADC 中断
2. MotorApplyStrongDrag(Ud=1.0V) — 开环 Ud 电压，锁转子
3. delay 1 秒等稳定
4. 读 10 次编码器电角度 → 算术平均 = zeroOffset
5. 关强拖，开 ADC 中断
```

**问题**：

1. **单点 vs 多点**：只在 d 轴 0° 一个位置测量。如果该位置恰好有齿槽转矩卡住转子，或者摩擦导致转子不能完全对齐，误差就固定在那里，没有其他位置的数据来抵消。

2. **算术平均的 2π 环绕 bug**：

```c
sum += elecAngle;
*zeroOffset = sum / 10;
// 如果采样值恰好跨在 0/2π 边界附近（比如 359°, 1°, 358°, 2°...），
// 算术平均 ≈ (359+1+358+2)/4 = 180° → 完全错误！
```

3. **开环 Ud 电压**：`FOC_STRONGDRAG=1.0V` 是开环电压，实际电流取决于电阻和反电势，不可控。电压太小 → 锁不住；电压太大 → 可能过流。

4. **需要关 ADC 中断**：强拖期间 FOC 主循环停止，完全是一个离线过程。

#### VESC 方式：电流闭环多点扫描 + 复数平均（[mcpwm_foc.c:1687](mcpwm_foc.c#L1687)）

```
原理：用 Id 电流闭环控制磁场方向，扫过多个电气位置，
      在每个位置记录"命令角度 vs 编码器读回角度"的差值，
      正转一圈 + 反转一圈，复数平均得到零位偏移

步骤：
1. Id 电流闭环（Id=给定值 A，Iq=0，PI 精确控流）
2. 旋转过程：m_phase_override（命令电角度）匀速步进
3. 正转：扫过 it_ofs 个位置，每个位置停 100ms
   在每个位置记录：angle_diff = encoder_angle - override_angle
   复数累加：s_sum += sin(diff), c_sum += cos(diff)
4. 反转：同样扫一遍（消除齿轮回差/磁滞）
5. offset = RAD2DEG(atan2(s_sum, c_sum))
```

**为什么更好**：

**a) 复数平均天然处理 2π 环绕**：

```c
// 不存才算术平均 359°vs 1° = 180° 的问题
// 359° → (cos=0.9998, sin=-0.0175)
// 1°   → (cos=0.9998, sin=0.0175)
// 和    → (cos=1.9997, sin=0) → atan2(0, 1.9997) = 0°  ← 正确！
```

**b) 多点扫描消除单点误差**：如果某个位置因为齿槽转矩偏差了 3°，扫过 6-18 个位置的复数平均会把这个随机误差分散掉。

**c) 正转+反转消除回差**：齿轮减速器的齿隙、磁编码器本身的磁滞会导致正转和反转时读数不同。两次扫描取平均消除系统性回差。

**d) 电流闭环**：不是开环 Ud 电压，而是 Id 电流 PI 闭环，电流精确等于设定值，安全可控。

**e) 不关中断**：整个过程在正常运行模式下进行，PWM 和 FOC 都在跑。

#### 标定精度对比总结

| | 你的方式 | VESC 方式 |
|--|---------|----------|
| 采样位置 | 1 个（d 轴 0°） | 3~9 个 × 正反 = 6~18 个 |
| 角度平均 | 算术平均（2π 环绕 bug） | atan2(Σsin, Σcos)（数学正确） |
| 回差补偿 | 无 | 正转+反转消除 |
| 齿槽/摩擦 | 影响标定精度 | 多点扫描抵消 |
| 电流控制 | 开环 Ud 电压 | Id 电流闭环 PI |
| 中断 | 需关 ADC | 正常运行 |
| 自动程度 | 手动调用 | 一键完成 |

#### 对你最简可行的改进

只需改一行——把算术平均改成复数平均，就能消除 2π 环绕 bug：

```c
// 改前：
sum += CalculateElectricalAngle(mechanicalAngle);
*zeroOffset = sum / sampleCount;

// 改后：
float s = 0, c = 0;
for (int i = 0; i < sampleCount; i++) {
    float elecAngle = CalculateElectricalAngle(Mt6701GetAngleWrapper());
    s += sinf(elecAngle);
    c += cosf(elecAngle);
}
*zeroOffset = atan2f(s, c);
```

### 10.3 编码器错误检测和故障保护

**你的 Li_FOC**：MT6701 驱动中没有任何错误检测。

```c
// mt6701.c — 唯一的"保护"是 SPI 超时 10 次重试，超时返回 0
while (spi_i2s_flag_get(encoder->spix, SPI_I2S_TDBE_FLAG) == RESET) {
    if (++retry > 10) return 0;
}
```

**VESC**：每个编码器类型有专属的错误检测链（[encoder/encoder.c:655](encoder/encoder.c#L655)）：

```c
void encoder_check_faults(...) {
    // 只在 "正在用编码器 + 转速低于 sl_erpm" 时触发故障
    // （高于 sl_erpm 时用观测器角度，编码器故障不致命）
    bool is_foc_encoder = 
        foc_sensor_mode == ENCODER && mcpwm_foc_is_using_encoder();

    if (is_foc_encoder) {
        switch (m_sensor_port_mode) {
        case AS5047:
            if (spi_error_rate > 0.05) → FAULT_CODE_ENCODER_SPI
            if (!diag.is_connected)   → FAULT_CODE_ENCODER_SPI
            if (diag.is_Comp_high)    → FAULT_CODE_ENCODER_NO_MAGNET
            if (diag.is_Comp_low)     → FAULT_CODE_ENCODER_MAGNET_TOO_STRONG
            break;
        case MT6816:
            if (no_magnet_error_rate > 0.05) → FAULT_CODE_ENCODER_NO_MAGNET
            break;
        // ... 每种编码器有专属的故障判断逻辑
        }
    }
}
```

对应到你的 MT6701 应该检测：
- SPI 通信错误率（CRC 失败、超时）
- 磁场强度诊断（MT6701 有磁场强弱标志位）
- 数据更新超时（编码器是否还在响应）

### 10.4 编码器错误率跟踪（每个读操作都会更新）

VESC 的 AS5047 驱动（[enc_as504x.c](encoder/enc_as504x.c)）在每次 SPI 读取时：

```c
// 伪代码
spi_error_rate = spi_error_cnt / spi_total_cnt;  // 滑动窗口内的错误率
last_update_time = current_time;                  // 用于超时检测
```

这给了两个维度的健康监控：
- **错误率**：瞬时/平均通信质量
- **超时**：编码器完全断连的检测

### 10.5 编码器非线性误差修正表

VESC 支持一个 360 点的角度误差修正表 `enc_corr[360]`（[mcpwm_foc.c:3269](mcpwm_foc.c#L3269)）：

```c
if (g_backup.enc_corr_en == 1) {
    int corr_ind = (int)enc_ang;                   // 取整到度数
    phase_tmp -= (float)g_backup.enc_corr[corr_ind]; // 减去该度的修正值
}
```

这是针对磁编码器普遍存在的 1-3 度谐波误差的补偿。你可以用激光干涉仪或高精度编码器对比标定出这张表。

### 10.6 编码器/观测器混合策略

即使在纯编码器模式下，VESC 也做了自动切换（[foc_math.c:572](foc_math.c#L572)）：

```
转速 < sl_erpm - 5%: 用编码器角度（编码器分辨率高，低速准）
转速 > sl_erpm + 5%: 用观测器角度（观测器在高转速时角度更新频率高于编码器读取频率）
```

你目前是有感/无感完全隔离——编码器模式永远用编码器，观测器模式永远用观测器。在高速时编码器的角度更新延迟（SPI 读取耗时）会引入相位滞后，VESC 的混合策略自动规避了这个问题。

### 10.7 有感的 I/F 启动不需要

VESC 在编码器模式下完全不需要开环启动流程——索引找到后直接做正常 FOC 闭环运行。你的代码在切换 sensorMode 时会重置 I/F 状态机，但如果你一直用有感模式，那段代码就一直是无用分支。VESC 的做法更干净：sensorMode 决定了是否需要走 openloop override 流程。

### 10.8 针对你的 MT6701 有感模式的优先级建议

| 优先级 | 改进项 | 收益 |
|--------|--------|------|
| **P0** | MT6701 SPI 错误率 + 超时检测 | 保护有感模式不受编码器故障影响 |
| **P1** | `foc_correct_encoder` 风格混合切换 | 高速时自动切观测器，消除 SPI 读取延迟引起相位滞后 |
| **P2** | 在线零位偏移标定（不关 ADC 中断） | 标定过程可控、可重复 |
| **P3** | `foc_encoder_ratio` 齿轮比参数 | 支持编码器装在输出端的场景 |
| **P4** | 360 点误差修正表 | 补偿磁编码器谐波误差 |

---

## 十一、串口/上位机通信对比

### 11.1 你的通信架构

**帧格式**（[usart3.c:44](usart3.c#L44)）：

```
┌──────┬──────┬──────┬──────────┬────────┬──────┐
│ 帧头  │ 命令 │ 长度 │ 数据区    │ 校验和 │ 帧尾 │
│ 0xA5 │ 1 B  │ 1 B  │ N×4B     │ 1 B    │ 0x49 │
└──────┴──────┴──────┴──────────┴────────┴──────┘
校验和 = 逐字节累加和（非 CRC）
数据区 = count 个 float32（原始 4 字节浮点）
```

**命令处理**（[protocol.c:48](protocol.c#L48)）：

```c
void Comm_CommandHandler(void) {
    switch (g_commCmd) {
        case CMD_CONNECT_MOTOR:   // 返回所有参数
        case CMD_SETSPEEDPIDKP:   // 设置速度环 Kp
        case CMD_SPEED:           // 开启速度遥测 → speed_Enabled = 1
        case CMD_SPEED_CLODE:     // 关闭速度遥测 → speed_Enabled = 0
        // ... 90 个 case
    }
}
```

**遥测方式**：每个变量需要独立开/关命令对（如 `CMD_SPEED` / `CMD_SPEED_CLODE`），使能后固件在 ISR 或 Task 中持续发送 raw float 数据。

### 11.2 VESC 的通信架构

**帧格式**（[packet.c:41](comm/packet.c#L41)）：

```
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│ 长度类型  │ 长度     │ 数据区    │ CRC16    │ 帧尾     │
│ 1 B      │ 1~3 B   │ N B      │ 2 B      │ 0x03     │
└──────────┴──────────┴──────────┴──────────┴──────────┘
长度类型：2=8bit长度, 3=16bit长度, 4=24bit长度
CRC16：真 CRC，不是简单校验和
```

**命令处理**（[commands.c:384](comm/commands.c#L384)）：

```c
// 一次性返回 20+ 个变量的统一数据包（COMM_GET_VALUES）
case COMM_GET_VALUES:
    buffer_append_float16(send_buffer, temp_fet, 1e1, &ind);     // float16 压缩
    buffer_append_float32(send_buffer, avg_motor_current, 1e2, &ind); // float32
    buffer_append_float32(send_buffer, rpm, 1e0, &ind);
    // ... 一次调用返回所有实时数据

// 选择性获取 — 上位机传 bitmask，只返回需要的字段
case COMM_GET_VALUES_SELECTIVE:
    mask = buffer_get_uint32(data, &ind2);
    if (mask & (1 << 0)) { buffer_append_float16(..., temp_fet, ...); }
    if (mask & (1 << 7)) { buffer_append_float32(..., rpm, ...); }
    // 上位机只要温度+转速 → bitmask = 0x81 → 只传 2 个值
```

**数据压缩**：
```c
// float16：2 字节有损压缩（适合温度、电压等精度要求不高的量）
buffer_append_float16(buf, temp, 1e1, &ind);  // 温度 × 10 → int16 → 2 字节

// float32_auto：自动选择最优 scale factor
buffer_append_float32_auto(buf, value, &ind);

// float32：4 字节（指定 scale）
buffer_append_float32(buf, current, 1e2, &ind); // 电流 × 100 → int32 → 4 字节
```

**Plot/Scope 系统**（[commands.c:1987](comm/commands.c#L1987)）：
```c
// 示波器接口：上位机不关心数据类型，只管画波形
commands_init_plot("X 轴名", "Y 轴名");
commands_plot_add_graph("通道名");
commands_plot_set_graph(0);
commands_send_plot_points(x, y);
```

### 11.3 核心差异对比

| 维度 | Li_FOC | VESC |
|------|--------|------|
| **帧校验** | 1 字节累加和（弱） | 2 字节 CRC16（强） |
| **数据传输** | raw float32（4 字节/值） | float16(2B) / float32(4B) / float32_auto 可选 |
| **遥测方式** | 每变量独立 ON/OFF 命令对 | 一条 `COMM_GET_VALUES` 返回所有；`COMM_GET_VALUES_SELECTIVE` + bitmask 选择性返回 |
| **协议扩展** | 加一个变量 → 加一对命令字 → 改上位机 | 加一个字段 → 在 `COMM_GET_VALUES` 末尾追加一个 `if(mask)` → 上位机更新 bitmask 定义 |
| **示波器** | 无通用 scope | 通用 Plot 系统（任意 X/Y 通道 + 字符串命名） |
| **包大小** | 固定 4B/值，无压缩 | 可压缩到 2B/值（节省 50% 带宽） |
| **多传输层** | 仅 USART3 | USB + CAN + UART，同一套 packet 层 |
| **调试输出** | printf 定向到串口 | `commands_printf`（也走 packet 协议，可在线开关） |
| **命令个数** | 90 个独立命令 | ~30 个核心命令 + 参数全通过配置接口读写 |
| **配置读/写** | 每个参数单独命令 | 统一的 `COMM_GET_MCCONF` / `COMM_SET_MCCONF` 读写整个配置结构体 |

### 11.4 最关键的架构差异：命令 vs 配置

**你**：每个可调参数都是一个独立命令。

```c
CMD_SETSPEEDPIDKP   → g_pMotor->speedPID.kp = g_cmdData;  // 单独命令
CMD_SETSPEEDPIDKI   → g_pMotor->speedPID.ki = g_cmdData;  // 单独命令
CMD_SETIQPIDKP       → g_pMotor->iqPID.kp = g_cmdData;    // 单独命令
CMD_SETMOTORRS       → g_pMotor->rs = g_cmdData;           // 单独命令
// ... 每加一个参数就要加一个命令枚举 + case
```

**VESC**：所有配置是一个序列化的结构体，一条命令读写全部。

```c
// 读配置：一条命令返回整个 mc_configuration 的二进制 blob（confgenerator.c 序列化）
case COMM_GET_MCCONF:
    confgenerator_serialize_mcconf(send_buffer, m_conf);
    // 一包返回 400+ 参数，上位机解析到对应的 UI 控件

// 写配置：一条命令写入整个 mc_configuration
case COMM_SET_MCCONF:
    confgenerator_deserialize_mcconf(data, m_conf);
    // 所有 400+ 参数一次性更新 + Flash 持久化
```

这意味着：
- 加新参数：在 `mc_configuration` 结构体加一个字段 → 在 `confgenerator.c` 的序列化/反序列化函数里各加一行 → 上位机加一个 UI 控件。**不需要加新命令**。
- 你的方式：加新参数 → 加命令枚举 → 加 case 分支 → 上位机加对应命令 → 两边要同步命令字

### 11.5 哪种更好

**对于你的场景（个人项目、快速迭代）**，说实话两种方式各有优劣：

- **VESC 的优势**：成熟稳定，协议扩展性好，带宽利用率高，多传输层统一。适合多硬件变体、多人协作、长期维护的项目。
- **VESC 的代价**：`confgenerator.c` 序列化层、`commands.c` 的 bitmask 管理、packet 层的 CRC 和变长帧都增加了复杂度。对于一个只有你一个人维护的项目，这套体系的维护成本可能超过收益。

**对你最实用的改进**（不一定要照搬 VESC 全部）：

1. **校验和 → CRC16**：你的累加和检错能力弱（交换两个字节检测不到），换成 CRC16 只多了 1 个字节
2. **用 bitmask 替换 ON/OFF 命令对**：遥测不用每变量两个命令，上位机发一个 32-bit mask 指定要哪些数据，固件一包返回
3. **数据压缩**：温度、电压等慢变量用 2 字节 int16 传输，节省带宽
4. **printf 走协议**：不直接往串口打 ASCII，用 `commands_printf` 封装，上位机能在线开关调试输出

---

## 十二、深度对比：电流环、解耦、死区、制动等细节处理

前面几章覆盖了架构层面的差异。这一章深入到具体的算法实现细节，每一项 VESC 都比你多做了处理。

### 12.1 电流环交叉解耦（Cross-coupling Decoupling）

**物理背景**：永磁同步电机的 dq 轴电压方程是耦合的：

```
vd = Rs·id + Ld·did/dt − ωe·Lq·iq           ← d 轴受 q 轴电流影响
vq = Rs·iq + Lq·diq/dt + ωe·Ld·id + ωe·ψm  ← q 轴受 d 轴电流和反电势影响
```

如果只对 id/iq 分别做独立 PI 控制，那交叉项 ωe·Lq·iq、ωe·Ld·id 和反电势 ωe·ψm 会被 PI 当作"扰动"来对抗。低速时 PI 带宽够高，能压住；高速时 PI 跟不上，id/iq 就会偏离目标。

**你的 Li_FOC**：没有解耦，只有独立 PI。

```c
// FOC.c:849 — Id 目标恒为 0，Iq 目标由外环给
CurrentPIControlIQ(pFOC);  // 独立 PI，无前馈
CurrentPIControlID(pFOC);  // 独立 PI，无前馈
```

**VESC**（[mcpwm_foc.c:4642](mcpwm_foc.c#L4642)）：4 种解耦模式可选。

```c
switch (conf_now->foc_cc_decoupling) {
case FOC_CC_DECOUPLING_CROSS:             // 仅交叉项
    dec_vd = iq * ωe * Lq;                // 抵消 ωe·Lq·iq 对 d 轴的影响
    dec_vq = id * ωe * Ld;                // 抵消 ωe·Ld·id 对 q 轴的影响
    break;

case FOC_CC_DECOUPLING_BEMF:              // 仅反电势
    dec_bemf = ωe * ψm;                   // 抵消 ωe·ψm 对 q 轴的影响
    break;

case FOC_CC_DECOUPLING_CROSS_BEMF:        // 全解耦（推荐）
    dec_vd = iq * ωe * Lq;
    dec_vq = id * ωe * Ld;
    dec_bemf = ωe * ψm;
    break;
}

// 前馈叠加到 PI 输出上
state_m->vd -= dec_vd;
state_m->vq += dec_vq + dec_bemf;
```

你的电机 Lq=74μH, Ld=40μH, 11 对极，在 1000rpm 机械（=11000 ERPM = 1150 rad/s 电气）时：
- 交叉项 vd_offset = iq × 1150 × 0.000074。若 iq=10A → 0.85V，这在 12V 总线上已经占 7%
- 反电势 = 1150 × ψm。若 ψm≈0.003 → 3.5V，占 29%

没有解耦的话，PI 要出额外的电压来对抗这些项，高速时明显吃力。

### 12.2 死区补偿（Dead-time Compensation）

**物理背景**：MOSFET 上下管切换时需要插入死区时间（通常 200-800ns），防止上下管同时导通短路。死区期间两管都关断，输出电压由续流二极管决定，方向和负载电流方向相同。这导致实际输出电压偏离指令值，在低电感电机中尤其明显——小电流时波形严重畸变。

**你的 Li_FOC**：没有死区补偿。

**VESC**（[mcpwm_foc.c:5097](mcpwm_foc.c#L5097)）：

```c
// 死区时间 → 占空比误差
const float mod_comp_fact = conf_now->foc_dt_us * 1e-6 * conf_now->foc_f_zv;

// 根据电流方向确定每相补偿符号
// 钳位效应：sign(ia)×2/3 - sign(ib)×1/3 - sign(ic)×1/3
const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

// 补偿到调制量上
mod_alpha -= mod_alpha_comp;
mod_beta -= mod_beta_comp;
```

**影响**：如果你的死区是 500ns，PWM 频率 20kHz，则 `mod_comp_fact = 500e-9 × 20000 = 0.01 = 1% 占空比`。在小电流时，1% 占空比误差导致电压指令不准，电流波形在过零点附近出现明显的"零位钳位"（zero-crossing clamping）畸变。

### 12.3 电压饱和时的 d 轴优先反饱和（Anti-windup）

**你的 Li_FOC**：

```c
// FOC.c:854 — 非开环模式下直接用 PI 输出
pFOC->uq = pFOC->iqPID.out;
pFOC->ud = pFOC->idPID.out;
// 然后直接进入 SVPWM，没有电压限幅检查
```

**VESC**（[mcpwm_foc.c:4667](mcpwm_foc.c#L4667)）：

```c
// 最大不失真电压矢量长度
float max_v_mag = ONE_BY_SQRT3 * max_duty * v_bus * foc_overmod_factor;

// ★ d 轴优先反饱和：d 轴控制弱磁和效率，优先级高于 q 轴
utils_truncate_number_abs(&vd, max_v_mag * foc_mag_vd_max);    // d 轴限到上限
utils_truncate_number_abs(&vd_int, max_v_mag * foc_mag_vd_max); // 积分器也限
float max_vq = sqrtf(SQ(max_v_mag) - SQ(vd));                   // q 轴用剩余空间
utils_truncate_number_abs(&vq, max_vq);
utils_truncate_number_abs(&vq_int, max_vq);
```

**为什么 d 轴优先**：当电压饱和时（高速/重载），如果放任 PI 积分器随意饱和，退出饱和时会有很大的超调。VESC 做了两件事：
1. 同时限幅 PI 输出和积分器（你的代码只限了输出）
2. d 轴优先（q 轴拿剩下的），因为 d 轴控制弱磁——弱磁不够会导致转速上不去甚至失控

### 12.4 Duty 控制模式的 PI 降速保护

**物理背景**：在占空比控制模式下，如果突然降低目标占空比（比如从 80% 降到 20%），直接截断电压会导致电流突变和转矩冲击。

**VESC**（[mcpwm_foc.c:3385](mcpwm_foc.c#L3385)）：

```c
if (fabsf(duty_set) < (duty_abs - 0.01) && ...) {
    // 目标占空比 < 实际占空比 → 不能直接截断，用 PI 平滑降下来
    float error = duty_set - duty_now;
    float p_term = error * foc_duty_dowmramp_kp / v_bus;
    m_duty_i_term += error * (foc_duty_dowmramp_ki * dt) / v_bus;
    // 积分器防饱和，限制到 [-1, 1]
    float output = p_term + m_duty_i_term;
    iq_set = output * current_max;
} else {
    // 占空比已经低于目标 → 直接用最大电流，自然限幅
    state_now->max_duty = duty_set;
    iq_set = SIGN(duty_set) * current_max;
}
```

这会用一个专用的 PI 控制器把占空比平滑降下来，同时在 duty 模式下复位同向积分器（防止降速时积分器助推）。

### 12.5 制动时的方向切换保护

**VESC**（[mcpwm_foc.c:3331](mcpwm_foc.c#L3331)）：

```c
if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
    // 检测到方向反转 OR 电压符号反转 OR 占空比降到零
    if ((SIGN(speed) != SIGN(m_br_speed_before) ||
         SIGN(vq) != SIGN(m_br_vq_before) ||
         fabsf(duty_filtered) < 0.001 ||
         m_br_no_duty_samples < 10) &&
        i_abs_filter < fabsf(iq_set)) {
        // → 短接全部三相（duty=0），等电流降到设定值再恢复
        duty_set = 0.0;
        m_br_no_duty_samples = 0;
    } else if (m_br_no_duty_samples < 10) {
        // 维持 duty=0 至少 10 个周期
        duty_set = 0.0;
        m_br_no_duty_samples++;
    }
}
```

制动时电机转速过零的瞬间，如果不做这个处理，电流控制器会试图维持制动电流，但这会变成反向驱动。VESC 在检测到方向变化时先短接三相，等电流自然衰减，再恢复制动控制。

### 12.6 电阻在线辨识（Resistance Observer）

**VESC**（[mcpwm_foc.c:4145](mcpwm_foc.c#L4145)）：

```c
// 基于自适应磁链观测器的电阻在线估计
// 参考: "An adaptive flux observer for the PMSM" (doi:10.1002/acs.2587)
float res_est_gain = 0.00002;
m_res_est = m_r_est_state - 0.5 * res_est_gain * L * i_abs_sq;
float res_dot = -res_est_gain * (m_res_est * i_abs_sq + ωe * x1*iq - ωe*x2*id + ...);
m_r_est_state += res_dot * dt;
// 限制在标称值的 25%~300%
utils_truncate_number(&m_r_est_state, foc_motor_r * 0.25, foc_motor_r * 3.0);
```

虽然没有直接的温度传感器，但通过观测器的自适应机制，实时跟踪电阻变化，用于温度补偿。

### 12.7 温度补偿的电流环增益

**你的代码**：电流环 Ki 固定，不随温度变化。

**VESC**（[mcpwm_foc.c:3928](mcpwm_foc.c#L3928)）：当温度补偿开启时，电流环 Ki 也随电阻变化同步调整：

```c
m_res_temp_comp = foc_motor_r * comp_fact;           // 电阻温度补偿
m_current_ki_temp_comp = foc_current_ki * comp_fact;  // 电流环 Ki 同步补偿

// 电流环中使用：
float ki = conf_now->foc_current_ki;
if (conf_now->foc_temp_comp) {
    ki = motor->m_current_ki_temp_comp;   // 用温度补偿后的 Ki
}
```

**原因**：电机电气时间常数 τ = L/R 随温度变化。如果 R 升高 30% 但 Ki 不变，电流环带宽也会变，控制性能不一致。

### 12.8 硬件相位滤波器补偿

某些 VESC 硬件版本在 MOSFET 输出端有 RC 滤波器来抑制 EMI，但这会衰减和延迟高频电压分量。

**VESC**（[mcpwm_foc.c:5143](mcpwm_foc.c#L5143)）：

```c
#ifdef HW_HAS_PHASE_FILTERS
if (foc_phase_filter_enable && abs_rpm < foc_phase_filter_max_erpm) {
    float mod_mag = NORM2_f(mod_alpha, mod_beta);
    // 用滤波器测量到的电压幅值替代理论值
    // 但保留调制量的方向（角度）
    if (mod_mag > 0.04) {
        v_alpha = mod_alpha / mod_mag * v_mag_filter;   // 方向用理论值，幅值用实测值
        v_beta = mod_beta / mod_mag * v_mag_filter;
    }
    // 电压幅值偏差过大 → 触发故障（滤波器硬件损坏？）
    if (fabsf(v_mag_mod - v_mag_filter) > l_max_vin * 0.05) {
        mc_interface_fault_stop(FAULT_CODE_PHASE_FILTER, ...);
    }
}
#endif
```

### 12.9 电机自动释放机制

**VESC**（[mcpwm_foc.c:3946](mcpwm_foc.c#L3946)）：

```c
// 当所有电流设定值都低于 cc_min_current 时，释放电机
if (fabsf(m_iq_set) < min_current &&
    fabsf(m_id_set) < min_current &&
    m_i_fw_set < min_current &&
    m_current_off_delay < dt) {
    m_control_mode = CONTROL_MODE_NONE;
    m_state = MC_STATE_OFF;
    stop_pwm_hw(motor);    // 关 PWM → 电机惯性滑行
}
```

这比手动设占空比=0 更安全——PWM 完全关闭，MOSFET 全部高阻，电机完全自由旋转。

### 12.10 位置控制的角度分频器

**VESC**（[mcpwm_foc.c:3879](mcpwm_foc.c#L3879)）：当电机有减速器时，输出轴转一圈对应电机转 N 圈。位置控制需要在输出轴侧做闭环，但 FOC 换相需要在电机侧做：

```c
if (p_pid_ang_div > 0.98 && p_pid_ang_div < 1.02) {
    // 无减速器：位置 = 电机角度
    m_pos_pid_now = angle_now;
} else {
    // 有减速器：电机角度 / 减速比，并跟踪跨圈
    m_pos_pid_now = m_pid_div_angle_accumulator + angle_now / ang_div;
    // 跨圈检测（0°↔360° 边界）
    if (angle_now < 90 && last_angle > 270) {
        accumulator += 360.0 / ang_div;  // 正转跨圈
    } else if (angle_now > 270 && last_angle < 90) {
        accumulator -= 360.0 / ang_div;  // 反转跨圈
    }
}
```

你的代码没有这个——位置直接就是电机机械角度乘以极对数，不支持输出轴减速器。

### 12.11 电流滤波的双轨设计

**你的代码**：Id/Iq 经过一个 LPF 后同时用于反馈和显示。

**VESC**（[mcpwm_foc.c:4612](mcpwm_foc.c#L4612)）：

```c
// 电流反馈 → 直接用 raw id/iq（无滤波，最小延迟）
// Park 变换的输出直接进 PI 反馈

// 滤波后的电流 → 仅用于慢速任务
UTILS_LP_FAST(id_filter, id, foc_current_filter_const);  // 默认 0.005
UTILS_LP_FAST(iq_filter, iq, foc_current_filter_const);
// id_filter/iq_filter 用于：弱磁计算、过流保护、增益调度、上位机显示
```

这是双轨设计：**反馈通路零延迟，监控通路做滤波**。你的代码把滤波值也送进 PI，引入了不必要的延迟。

### 12.12 总结：这些细节如何影响实际性能

| 细节 | 没做时的后果 | VESC 怎么处理的 |
|------|------------|---------------|
| 交叉解耦 | 高速大电流时 id 偏离 0，效率下降 | 4 种解耦模式前馈 |
| 死区补偿 | 小电流时电流波形畸变，低速抖动 | 根据电流方向补偿调制量 |
| d 轴优先反饱和 | 电压饱和时 q 轴抢电压，弱磁失效 | d 轴先限幅，q 轴用剩余空间 |
| duty 降速 PI | 占空比突变导致电流冲击 | 专用 PI 平滑降占空比 |
| 制动力向切换 | 过零时反向驱动 | 先短接三相，等电流衰减再恢复 |
| 电阻在线辨识 | 温度变化导致观测器模型失配 | 自适应观测器实时跟踪 |
| 电流环温度补偿 | R 变 30% 但 Ki 不变，带宽偏移 | Ki 随 R 同步缩放 |
| 相位滤波器 | 高速时 v_alpha/v_beta 幅值不准 | 硬件滤波器建模补偿 |
| 电流双轨 | PI 反馈也有滤波延迟 | raw id/iq 直送反馈，滤波值给慢任务 |
| 位置角度分频 | 减速器输出位置不正确 | 支持 N:1 角度分频 + 跨圈跟踪 |

### 12.13 多层保护机制

VESC 有完整的保护体系，你的代码基本没有。

**超时保护（Timeout）**（[timeout.c:192](timeout.c#L192)）：

```c
// 后台线程每秒检查一次
// 如果超过 timeout_msec 没有收到任何命令（timeout_reset 未调用）
// → 自动施加制动电流，释放电机
if (timeout_msec != 0 && chVTTimeElapsedSinceX(last_update_time) > MS2ST(timeout_msec)) {
    mc_interface_release_motor_override();
    mc_interface_set_brake_current(timeout_brake_current);  // 制动停机
}
```

**硬件急停（Kill Switch）**（[timeout.c:200](timeout.c#L200)）：

支持 4 种硬件信号：PPM 低/高电平、ADC 低/高阈值。急停信号有效 → 立即制动 + 忽略遥控输入 20 秒。

**独立看门狗（IWDG）**：

12ms 的硬件看门狗，主控制循环必须每个 PWM 周期喂狗（`timeout_feed_WDT`），否则 MCU 硬件复位。

**故障码体系（34 种）**：覆盖电压、电流、温度、编码器、Flash、旋变、相滤波、驱动芯片、LV 输出、过速/欠速等。

**多级电流限制**：
```c
l_current_max       // 电机电流上限
l_current_min       // 制动电流上限
lo_current_max      // 占空比模式最大电流
lo_current_min      // 占空比模式最小电流
cc_min_current      // 低于此值自动释放电机
l_current_max_scale // 运行时缩放系数（用于温度降压等）
```

**输入电流限制（Bus Current）**：VESC 计算 `i_bus = duty * iq`（直流母线电流），用于保护电池不过流：

```c
// mc_interface.c 中限制 iq 使 i_bus 不超过 l_in_current_max
if (fabsf(i_bus) > l_in_current_max) {
    iq *= l_in_current_max / fabsf(i_bus);
}
```

### 12.14 电机参数自动检测

VESC 可以一键自动测量电机 R、L、λ（磁链）、Hall 表：

- `mcpwm_foc_measure_resistance()`：注入直流电流，测 Vd/Id → R = Vd/Id（[mcpwm_foc.c:1795](mcpwm_foc.c#L1795)）
- `mcpwm_foc_measure_inductance()`：注入 PWM 脉冲，测电流斜率 → L = V·dt/di（[mcpwm_foc.c:1907](mcpwm_foc.c#L1907)）
- `mcpwm_foc_measure_flux_linkage()`：开环旋转，测 BEMF → λ = BEMF/ωe
- `mcpwm_foc_hall_detect()`：开环旋转，记录每个 Hall 状态对应的电角度 → Hall 表

你的 Rs=0.198, Lq=0.000074, Ld=0.000040 全是手工填的。换电机就得改宏重编译。

### 12.15 模拟电机（Virtual Motor）

VESC 内置了一个虚拟电机（`virtual_motor.c`），可以在没有真实电机的情况下仿真运行：

```c
// 在 ADC ISR 中，每个 PWM 周期更新虚拟电机状态
virtual_motor_int_handler(v_alpha, v_beta);
// 虚拟电机会模拟反电势/电流响应，返回模拟的编码器角度
if (virtual_motor_is_connected()) {
    enc_ang = virtual_motor_get_angle_deg();
}
```

这对调试 FOC 算法、测试上位机通信非常有用——不需要接真实电机就能验证整个控制链路。

### 12.16 速度环的转速加速限制

VESC 的速度 PID 支持加速度限制，防止转速指令突变导致的电流冲击：

```c
// datatypes.h:534
float s_pid_min_erpm;   // 低速时不跑速度环
float s_pid_kp, s_pid_ki, s_pid_kd;
float s_pid_kd_filter;  // D 项滤波器
bool s_pid_allow_braking; // 是否允许再生制动
```

速度 PID 实现（[foc_math.c:505](foc_math.c#L505)）中还包含：
- D 项经低通滤波后再用（防噪声放大）
- 低于 `s_pid_min_erpm` 时自动关速度环
- 可选择禁止制动（只驱动不减速，适用于某些应用）

### 12.17 双电机 + 并联模式

VESC 原生支持双电机（`m_motor_1` + `m_motor_2`），共享一个 PWM ISR，交替处理。还支持双电机并联模式（`HW_HAS_DUAL_PARALLEL`），两个电机的电流相加一起控制。

### 12.18 MC 接口层次抽象

所有对外接口通过 `mc_interface.c` 统一访问，不直接操作 `m_motor_1`/`m_motor_2`：

```c
mc_interface_set_current(10.0);      // 自动选择当前线程对应的电机
mc_interface_set_duty(0.5);
mc_interface_get_rpm();
mc_interface_lock();  mc_interface_unlock();  // 线程安全锁
mc_interface_select_motor_thread(1);  // 切换当前线程操作的电机
```

你的代码直接读全局变量 `g_pMotor->xxx`，没有这种层次抽象。

### 12.19 滑行期间继续跟踪转子位置

**你的代码**：PWM 关闭（电机释放）后不再运行 FOC 主循环 → 观测器停止 → 转子位置丢失。下次重新驱动时需要从头开始（有感重新读编码器或重新做 I/F 启动）。

**VESC**（[mcpwm_foc.c:3687](mcpwm_foc.c#L3687)）：即使电机不驱动，照样跑观测器。

```c
// 电机释放/滑行时，PWM duty=0，但仍然在 ISR 中：
update_valpha_vbeta(motor_now, 0.0, 0.0, 1.5 / v_bus);  // v_alpha=v_beta=0

// 观测器仍然运行！用 v=0, i=0 继续积分
foc_observer_update(0, 0, 0, 0, dt, &observer_state, 0, motor);

// 从磁链估计中提取角度（即使在滑行中）
motor_now->m_phase_now_observer = utils_fast_atan2(
    motor_now->m_x2_prev + observer_state.x2,
    motor_now->m_x1_prev + observer_state.x1);

// 补偿半个开关周期 + observer_offset 的延迟
m_phase_now_observer += m_pll_speed * dt * foc_observer_offset;
```

**为什么这很重要**：如果电机在惯性滑行中重新上电，而观测器没有在滑行期间跟踪位置，那么重新驱动瞬间的角度可能是错的 → 电流冲击甚至反转。VESC 的做法保证了任何时候角度都是最新的。

### 12.20 温度降额（Temperature Derating）

**VESC**（[mc_interface.c:2337](mc_interface.c#L2337)）：MOS 温度和电机温度各自独立降额。

```c
// MOS 温度降额
if (temp_fet < l_temp_fet_start) {
    // 温度正常 → 满电流
} else if (temp_fet > l_temp_fet_end) {
    // 温度超限 → 电流 = 0 + 触发 FAULT_CODE_OVER_TEMP_FET
} else {
    // 中间区域 → 线性降额
    max_current = utils_map(temp_fet, l_temp_fet_start, l_temp_fet_end, max_current, 0);
}

// 电机温度降额（同样的逻辑，独立阈值）
// 取两者中更严格的限制
motor->m_lo_min_mos = lo_min_mos;
motor->m_lo_max_mos = lo_max_mos;
```

这是**平滑降额**而不是一刀切——超过起始温度就开始减小电流上限，到终止温度时才完全关断。你的代码没有这个保护。

### 12.21 CAN 总线多 VESC 协调

VESC 支持通过 CAN 总线连接多个 VESC，实现：
- `COMM_FORWARD_CAN`：上位机命令通过 USB VESC 转发到 CAN 总线上的其他 VESC
- `CAN_PACKET_PING_CAN`：CAN 设备发现
- `CAN_PACKET_SET_CURRENT_REL`：多个 VESC 统一用相对电流值协调（如差速转向）
- `can_status_msg`：每个 VESC 定期广播自身状态（电压、电流、转速、温度）
- PPM/ADC/NRF 输入可以通过 CAN 共享给总线上的所有 VESC
- CAN 总线支持 VESC 协议和 UAVCAN/Cyphal 协议两种模式

### 12.22 速度环加速度限制（隐式）

虽然 VESC 的速度 PID 本身没有显式的加速度限制，但它通过 `utils_step_towards` 实现了平滑过渡：

```c
// utils_math.h:129
static inline void utils_step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        *value += step;
        if (*value > goal) *value = goal;
    } else if (*value > goal) {
        *value -= step;
        if (*value < goal) *value = goal;
    }
}
```

在编码器检测和电流渐变中大量使用，保证状态切换平滑无冲击。

### 12.23 快速数学函数库

VESC 用查表+线性插值实现了快速的 `sin/cos` 和 `atan2`：

```c
float utils_fast_atan2(float y, float x);           // 查表 atan2，比标准库快 5-10 倍
void utils_fast_sincos_better(float angle, float *s, float *c);  // 查表 sin+cos 同时算
```

在 20kHz PWM ISR 中，标准 `atan2f` 调用一次就要 2-3μs，用查表版降到 0.3μs。你的代码用的是标准 `atan2f`，每个 PWM 周期至少调用一次（在 SMO/PLL 中）。

### 12.24 `current_off_delay` 机制

```c
// 释放电机时不立即关 PWM，而是等待一个短暂延迟
mcpwm_foc_set_current_off_delay(1.0);  // 设置 1 秒延迟

// 在主循环中：
utils_step_towards(&m_current_off_delay, 0.0, dt);  // 延迟到期
if (m_current_off_delay < dt) {
    stop_pwm_hw(motor);  // 安全关 PWM
}
```

这个延迟确保在故障或模式切换时，电流有足够的时间降到安全范围，然后再关 PWM。

---

## 十三、通用改进建议（不限有感模式）

### 优先级 0：将 #define 宏改为运行时配置结构体

这是最根本的改进，也是后续所有改进的基础：

```c
// ============ 改前：foc_config.h，全是 #define ============
#define FOC_SMO_RS        0.198f
#define FOC_SMO_LS        0.000057f
#define FOC_SMO_K_SLIDE   30.0f
// 改任何参数 → 重编译 → 重烧录

// ============ 改后：foc_config.h 定义的结构体，默认值从 Flash 或 EEPROM 加载 ============
typedef struct {
    float motor_r;           // 电机相电阻
    float motor_l;           // 电机电感
    float smo_k_slide;       // 滑模增益
    float pll_kp;            // PLL Kp
    float pll_ki;            // PLL Ki
    // ... 所有可调参数
    uint8_t sensor_mode;     // 传感器模式
    uint8_t observer_type;   // 观测器类型（预留扩展）
} foc_config_t;

// 控制循环中读取：
const foc_config_t *cfg = &g_foc_config;
float k_slide = cfg->smo_k_slide;  // 不再是 FOC_SMO_K_SLIDE 宏
```

这样做的好处：
- 上位机在线改参数，无需重编译
- 换电机只需改配置数据，同一份固件适配所有电机
- 加新参数只需在结构体末尾加字段（EEPROM 兼容）
- 调试时可以在上位机实时扫参

### 优先级 1：有感模式也让观测器后台运行 + foc_correct_encoder 风格混合切换

```c
// ============ 改前 ============
if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORLESS) {
    SMO_Update(...);   // 有感时不跑，观测器无输出
}
phase = (sensorMode == SENSORED) ? encoder_angle : smo_angle;

// ============ 改后 ============
SMO_Update(...);  // 始终运行，观测器始终有输出

// 带滞环的编码器/观测器混合切换
float rpm_abs = fabsf(speed);
if (using_encoder) {
    if (rpm_abs > sl_erpm + sl_erpm * 0.05f) using_encoder = false;
} else {
    if (rpm_abs < sl_erpm - sl_erpm * 0.05f) using_encoder = true;
}
phase = using_encoder ? encoder_angle : observer_angle;
```

### 优先级 2：编码器错误检测

```c
// 在 mt6701.c 中加入健康监控
struct {
    uint32_t spi_error_cnt;
    uint32_t spi_total_cnt;
    float spi_error_rate;       // 错误率超过 5% → 报警/降级
    uint32_t last_update_time;
    float last_valid_angle;
} encoder_health;
```

### 优先级 3：观测器 switch 分发（为多观测器预留）

```c
// 统一的观测器更新入口，内部 switch 分发
void observer_update(float v_alpha, float v_beta,
                     float i_alpha, float i_beta, ...) {
    switch (cfg->observer_type) {
    case OBSERVER_SMO:
        SMO_Update(...); break;
    case OBSERVER_ORTEGA:
        Ortega_Update(...); break;  // 未来扩展
    // ...
    }
}
```

### 优先级 4：增益调度

```c
// 根据占空比缩放观测器增益，而非固定值
float gamma = utils_map(fabsf(duty), 0.0f, 1.0f, gain_min, gain_max);
motor->gamma_now = gamma;
```

### 优先级 5：凸极性利用

你的电机 Ld=40μH, Lq=74μH，凸极率 1.85。可以加入：
- MTPA（最大转矩电流比）：给一点负 Id 来利用磁阻转矩
- 观测器中用 Ld/Lq 差异修正等效电感，提高大负载时角度精度

---

## 十四、总体比较表

| 维度 | Li_FOC | VESC |
|------|--------|------|
| **配置方式** | `#define` 编译时宏，改参数需重编译烧录 | 运行时结构体 + Flash 持久化，上位机在线改 |
| **传感器选择** | 2 种，`if/else` 硬编码 | 9 种，`switch(配置字段)` 分发 |
| **传感器切换** | 完全隔离，切换需清零全部 PI/SMO 状态 | 观测器始终运行，带滞环无缝切换 |
| **观测器种类** | 1 种 SMO | 7 种，`switch(配置字段)` 分发 |
| **观测器选型** | 无选择（只此一种） | 用户手动选，固件不自动换 |
| **观测器增益** | 固定常数 | 按占空比/电压自动调度 |
| **电机状态管理** | 单一 `FocState` | `motor_all_state_t` 双电机 |
| **饱和补偿** | 无 | 3 种模式 |
| **温度补偿** | 无 | 电阻温度系数实时补偿 |
| **凸极处理** | 无（单一电感值） | Ld/Lq 差异 + MTPA |
| **弱磁调速** | 无 | 完整实现 |
| **开环启动** | I/F 条件交接 | 三阶段定时 + 观测器状态预置 |
| **编码器容错** | 无 | SPI 错误率 + 超时 + 多级错误码 |
| **配置持久化** | 无 | Flash + CRC 校验，断电不丢 |
| **电流采样模式** | 固定扇区重构 | 4 种模式，按 PWM 占空比动态选择可信相 |
| **电流滤波** | Id/Iq 单层 LPF | 5 层 LPF (ABC→αβ→dq→幅值→调制深度) |
| **V0/V7 双采样** | 无 | 支持，频率翻倍 + 角度插值 |
| **多硬件适配** | 一份代码一个硬件 | 同一份固件 30+ 硬件变体 |

---

*文档生成于 2026-06-05，基于 VESC firmware v7.00 源码*
