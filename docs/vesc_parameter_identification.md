# VESC 电机参数辨识专题

> 基于 VESC firmware v7.00，分析电阻、电感、磁链、Hall 表、编码器五项自动检测的原理和代码实现

---

## 一、总览：一键检测的完整流程

VESC Tool 中点击 "Detect Motor Parameters" → 固件依次执行：

```
1. mcpwm_foc_dc_cal()          电流/电压 ADC 零点偏移校准
2. mcpwm_foc_measure_resistance()    测相电阻 R（Ω）
3. mcpwm_foc_measure_inductance_current()  测电感 Ld, Lq（μH）
4. conf_general_measure_flux_linkage_openloop()  测磁链 λ（Wb）
5. mcpwm_foc_encoder_detect()     测编码器 offset/ratio/inverted
6. mcpwm_foc_hall_detect()         测 Hall 表
7. 所有结果写入 m_conf → 自动保存到 Flash
```

上位机也可以单独执行其中某一步（如只测电阻电感）。

---

## 二、相电阻测量

### 2.1 物理原理

```
电机 d 轴电压方程（静止时 ωe=0）：
  vd = Rs · id + Ld · did/dt

通入恒定的直流 id 电流，稳态时 did/dt = 0：
  vd = Rs · id

因此：Rs = vd / id
```

给电机注入一个固定方向的直流电流（磁场锁在固定角度），转子静止后测量 Vd 和 Id，比值 = 相电阻。

### 2.2 代码实现（[mcpwm_foc.c:1795](mcpwm_foc.c#L1795)）

```c
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance) {
    // ======== 步骤 1：配置 FOC 进入电流闭环模式 ========
    motor->m_phase_override = true;
    motor->m_phase_now_override = 0.0;       // 固定角度 0°（d 轴对齐）
    motor->m_id_set = 0.0;                   // Id 目标 = 0
    motor->m_control_mode = CONTROL_MODE_CURRENT;
    motor->m_state = MC_STATE_RUNNING;

    // ======== 步骤 2：平滑爬升电流到目标值 ========
    // 不是突然跳到目标电流，而是每 ms 爬升 current/200
    while (fabsf(m_iq_set - current) > 0.001) {
        utils_step_towards(&m_iq_set, current, fabsf(current) / 200.0);
        // 检查故障
        chThdSleepMilliseconds(1);
    }

    // ======== 步骤 3：等待电流稳定 + 转子锁定 ========
    chThdSleepMilliseconds(50);

    // ======== 步骤 4：采样电压和电流 ========
    motor->m_samples.avg_current_tot = 0.0;
    motor->m_samples.avg_voltage_tot = 0.0;
    motor->m_samples.sample_num = 0;

    while (motor->m_samples.sample_num < samples) {
        chThdSleepMilliseconds(1);   // 1ms 间隔采样
        // 采样在 PWM ISR 中自动累加到 m_samples 中
    }

    // ======== 步骤 5：计算电阻 ========
    float current_avg = avg_current_tot / sample_num;
    float voltage_avg = avg_voltage_tot / sample_num;
    *resistance = voltage_avg / current_avg;   // R = V / I
}
```

**关键设计细节**：

**a) 为什么用 Iq 而不是 Id？**

虽然注入的电流是直流"锁轴"电流，但 VESC 选择注入在 q 轴（`m_iq_set`）而不是 d 轴。原因是 FOC 电流控制器中的 decoupling 项在 d 轴有 ωe·Lq·iq 的交叉项，静止时不影响。实际测到的是 `vq / iq`（q 轴电压/电流），对于表面贴磁电机，R_dq ≈ R_phase。

**b) 电流平滑爬升**（`utils_step_towards`）：

```c
// 不是 m_iq_set = current;  —— 这是阶跃，会触发过流保护
// 而是每次增加 current/200，耗时约 200ms 到达目标
utils_step_towards(&m_iq_set, current, fabsf(current) / 200.0);
```

**c) 采样机制**：

采样不是在本函数中手动完成的——它在 PWM ISR（每个 50μs 周期）中自动累加到 `m_samples.avg_current_tot` 和 `m_samples.avg_voltage_tot` 中。主循环每 1ms 检查一次是否积累了足够的样本数。

### 2.3 对 Li_FOC 的借鉴

你的电阻是手动填的 `#define FOC_SMO_RS 0.198f`。可以用同样的原理自动测量：

```c
// 伪代码
FOC_SetPhaseOverride(0.0);   // 固定 d 轴 0°
FOC_SetIqTarget(2.0);        // 注入 2A q 轴电流
delay(200);                   // 等待稳定
float vq_avg = 0, iq_avg = 0;
for (int i = 0; i < 500; i++) {
    vq_avg += g_pMotor->uq;
    iq_avg += g_pMotor->iq;
    delay(1);
}
float Rs = vq_avg / iq_avg;
```

---

## 三、电感测量（含 Ld/Lq 差异）

### 3.1 物理原理

这是 VESC 参数检测中最精妙的部分——**利用 HFI（高频注入）硬件来测量电感**。

```
原理：高频方波电压注入 → 测量 di/dt → L = V · dt / di

HFI（高频注入）的正常用途：向 d 轴注入高频电压方波，
  检测电流响应的斜率来提取转子位置（利用 Ld≠Lq 的凸极性）

测量电感的思路：复用 HFI 的硬件和信号处理链路，
  但不用它来提取位置，而是直接计算电流变化率 →
  反算电感值
```

为什么不用更简单的方法？常规测电感需要高速 ADC 捕捉电流上升沿，但 STM32 的 ADC 在每个 PWM 周期只采样一次。HFI 模式用了特殊的 ADC 采样时序，在一个 PWM 周期内采样多次（32 个点），可以捕捉到电流对高频电压的完整响应波形。

### 3.2 FFT 提取法

HFI 注入的是方波电压，电流响应中也含有方波频率的基波和奇次谐波。VESC 对采样的电流波形做 FFT（在 bin0 和 bin2 上），从中提取电感信息。

**关键公式**（[mcpwm_foc.c:2014](mcpwm_foc.c#L2014)）：

```c
// 从 FFT 结果中提取 Ld 和 Lq
float offset   = real_bin0;                    // 平均导纳 1/L_avg
float amplitude = NORM2_f(real_bin2, imag_bin2) * 2.0;  // 2 次谐波幅值

float Ld_est = 1.0 / (offset + amplitude);     // 1/(avg + amp)
float Lq_est = 1.0 / (offset - amplitude);     // 1/(avg - amp)

// 平均电感
L = (Ld_est + Lq_est) / 2.0;

// Ld-Lq 差异
ld_lq_diff = Lq_est - Ld_est;
```

**物理直觉**：
- bin0（DC 分量）= 平均电流变化率 = 1/L_avg（导纳的平均值）
- bin2（2 倍注入频率分量）= 响应中的 2 倍频分量幅值 → 反映 Ld 和 Lq 的不对称程度
- 如果 Ld=Lq（无凸极），bin2 幅值 = 0，Ld = Lq = 1/offset
- 如果 Ld<Lq（有凸极），电流响应不是对称的正弦，含 2 倍频分量，可从其幅值反算 Ld/Lq

### 3.3 代码实现（[mcpwm_foc.c:1907](mcpwm_foc.c#L1907)）

```c
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr,
                                  float *ld_lq_diff, float *inductance) {
    // ======== 步骤 1：保存当前 FOC 配置 ========
    save_all_foc_config_to_locals();

    // ======== 步骤 2：切换到 HFI 测量模式 ========
    motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI;
    motor->m_conf->foc_hfi_voltage_start = duty * Vbus * 2/3 * √3/2;
    motor->m_conf->foc_hfi_voltage_run  = 同上;
    motor->m_conf->foc_hfi_voltage_max  = 同上;
    motor->m_conf->foc_sl_erpm_hfi = 20000;          // 高阈值让 HFI 一直跑
    motor->m_conf->foc_hfi_samples = HFI_SAMPLES_32;  // 每周期 32 点采样
    motor->m_conf->foc_f_zv = min(f_zv, 30kHz);       // PWM 频率不超 30k

    // ======== 步骤 3：等待 HFI 准备好（ADC 缓冲区填满） ========
    while (!motor->m_hfi.ready) {
        chThdSleepMilliseconds(1);
    }

    // ======== 步骤 4：多次采样取平均 ========
    for (int i = 0; i < (samples / 10); i++) {
        // 关闭驱动 → 清空 HFI 缓冲区 → 重新开始
        mcpwm_foc_set_duty(0.0);
        chThdSleepMilliseconds(10);

        // 对 HFI 采样的电压和电流波形做 FFT
        fft_bin0_func(buffer,         &real_bin0,  &imag_bin0);   // 电压 bin0
        fft_bin2_func(buffer,         &real_bin2,  &imag_bin2);   // 电压 bin2
        fft_bin0_func(buffer_current, &real_bin0_i, &imag_bin0_i); // 电流 bin0

        // 从 FFT 结果计算电感
        float offset = real_bin0;                       // 1/L_avg
        float amplitude = NORM2(real_bin2, imag_bin2) * 2.0;  // 2 倍频幅值
        float Ld_est = 1.0 / (offset + amplitude);
        float Lq_est = 1.0 / (offset - amplitude);

        l_sum += (Ld_est + Lq_est) / 2.0;
        ld_lq_diff_sum += (Lq_est - Ld_est);
    }

    // ======== 步骤 5：恢复配置 + 输出结果 ========
    restore_all_foc_config();

    // 乘以 0.9 补偿：硬件延迟 + 高电阻/低电感比会使测量值偏大
    float ind_scale_factor = 0.9;
    *ld_lq_diff = ld_lq_diff_avg * 1e6 * ind_scale_factor;   // → μH
    *inductance = L_avg * 1e6 * ind_scale_factor;             // → μH
}
```

### 3.4 自动选择测量电流的版本

`mcpwm_foc_measure_inductance_current()`（[mcpwm_foc.c:2084](mcpwm_foc.c#L2084)）在上面的基础上加了自动搜索合适的 duty cycle：

```c
// 从 2% duty 开始，每次 ×1.5，直到测量电流 ≥ 目标电流
for (float i = 0.02; i < 0.5; i *= 1.5) {
    measure_inductance(i, 10, &i_tmp, 0, 0);
    if (i_tmp >= curr_goal) {
        duty_last = i;
        break;
    }
}
// 找到合适的 duty 后，用这个 duty 做完整精度测量
measure_inductance(duty_last, samples, curr, ld_lq_diff, inductance);
```

### 3.5 为什么乘 0.9

```c
// mcpwm_foc.c:2044
// 观测器对电感低估比对高估更稳定，所以乘 0.9
// 硬件延迟 + 高电阻/低电感比会使测量值偏高
float ind_scale_factor = 0.9;
*inductance = L_avg * 1e6 * ind_scale_factor;
```

### 3.6 对 Li_FOC 的借鉴

你的 `Lq=0.000074, Ld=0.000040` 是手动填的。如果你没有 HFI 硬件支持（多数 DIY 硬件没有），可以用更简单的方法测电感：

**方法：PWM 脉冲法**

```
1. 同测电阻一样，锁轴在固定角度
2. 施加短时 PWM 脉冲（如 10% duty，持续 1ms）
3. 测量电流从 0 爬升到峰值的斜率
4. L = V · Δt / Δi
5. 在 d 轴 0° 测一次 → Ld
6. 在 d 轴 90° 测一次 → Lq
```

这不需要 HFI 硬件，但需要一个足够快的电流采样（你的 20kHz PWM 够了）。

---

## 四、磁链测量

### 4.1 物理原理

```
BLDC 模式下，电机空载旋转时：
  反电势幅值 = 磁链 · 电角速度
  BEMF_amplitude = λ · ωe

在占空比 d 下：
  v_phase = d · Vbus - I · Rs · 2  （输出电压 = 占空比×母线 - 电阻压降）
  v_phase ≈ BEMF_amplitude （忽略电感压降）

因此：
  λ = (d · Vbus - I · Rs · 2) / (√3 · ωe)
```

VESC 的做法是：**临时切换到 BLDC 模式**，用低速开环拖动电机旋转，测 BEMF 电压和转速，反算磁链。

### 4.2 为什么用 BLDC 模式而不是 FOC 模式？

因为 BLDC 模式下反电势可以通过导通相的电压直接读出来（两相导通时，第三相悬浮，其电压就是反电势）。比 FOC 模式中需要从 dq 反电势中间接计算更直接、更准。

### 4.3 代码实现（[conf_general.c:742](conf_general.c#L742)）

```c
bool conf_general_measure_flux_linkage(float current, float duty,
        float min_erpm, float res, float *linkage) {

    // ======== 步骤 1：切换到 BLDC 有感/无感模式 ========
    mcconf->motor_type = MOTOR_TYPE_BLDC;
    mcconf->sensor_mode = SENSOR_MODE_SENSORLESS;   // 无感 BLDC
    mcconf->comm_mode = COMM_MODE_INTEGRATE;         // 积分换向
    mcconf->sl_min_erpm = min_erpm;                  // 最低转速
    mc_interface_set_configuration(mcconf);

    // ======== 步骤 2：最多 4 次尝试启动电机 ========
    bool started = false;
    for (int i = 0; i < 4; i++) {
        if (i == 1) {
            // 尝试 1 失败 → 调整参数：更高的积分限制
            mcconf->sl_cycle_int_limit = 250;
        } else if (i == 2) {
            // 尝试 2 失败 → 更高最低转速 + 更小积分限制
            mcconf->sl_min_erpm = 2 * min_erpm;
            mcconf->sl_cycle_int_limit = 20;
        } else if (i == 3) {
            // 尝试 3 失败 → 再翻倍转速 + 延迟换向模式
            mcconf->sl_min_erpm = 4 * min_erpm;
            mcconf->comm_mode = COMM_MODE_DELAY;
        }
        mc_interface_set_current(current);

        // 等待占空比达到目标 d（电机加速到所需转速）
        while (mc_interface_get_duty_cycle_now() < duty) {
            if (duty_now >= duty/2) {
                mcpwm_switch_comm_mode(COMM_MODE_DELAY);  // 半程切延迟换向
            }
            if (超时 5 秒) { started = false; break; }
        }
        if (started) break;
    }

    // ======== 步骤 3：采样 2000 次（2 秒） ========
    mc_interface_set_duty(duty);    // 固定占空比
    float avg_voltage = 0, avg_rpm = 0, avg_current = 0;
    for (int i = 0; i < 2000; i++) {
        avg_voltage += GET_INPUT_VOLTAGE() * mc_interface_get_duty_cycle_now();
        avg_rpm     += mc_interface_get_rpm();
        avg_current += mc_interface_get_tot_current();
        chThdSleepMilliseconds(1);
    }

    // ======== 步骤 4：计算磁链 ========
    avg_voltage /= 2000;
    avg_current /= 2000;
    avg_voltage -= avg_current * res * 2.0;  // 减去电阻压降
    avg_rpm /= 2000;

    // λ = (V - 2·I·R) / (√3 · ωe)
    *linkage = avg_voltage / (sqrtf(3.0) * RPM2RADPS_f(avg_rpm));
}
```

### 4.4 Openloop 磁链测量（改进版）

新版本使用 `conf_general_measure_flux_linkage_openloop()`（[conf_general.c:967](conf_general.c#L967)），在 FOC 模式下直接开环旋转，使用观测器的反电势输出来计算磁链：

```c
// FOC 模式下开环旋转电机
motor_type = MOTOR_TYPE_FOC;
foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;

// 固定占空比旋转，读 R 测出的电阻 + L 测出的电感
// 观测器反电势幅值 / 电角速度 = 磁链
```

这种方式比 BLDC 模式更准（直接利用 FOC 观测器），而且**不需要依赖电阻和电感参数的先验知识**——因为观测器本身就是基于电压/电流模型来估计反电势的。

### 4.5 对 Li_FOC 的借鉴

磁链是你完全没有的测量。用 FOC 开环测量法最直接：

```c
// 伪代码
FOC_SetSensorMode(SENSORLESS);   // 用 SMO 观测器
FOC_SetOpenloopDuty(0.2, 500);  // 20% duty, 目标 500rpm
delay(3000);                     // 等电机稳定旋转

float emag_sum = 0, speed_sum = 0;
for (int i = 0; i < 500; i++) {
    emag_sum  += g_smoObserver.eMag;   // 反电势幅值
    speed_sum += g_smoObserver.speed;  // 电角速度 rad/s
    delay(10);
}
float linkage = emag_sum / speed_sum;  // λ = E / ωe
```

前提是你的 SMO 先跑稳了。

---

## 五、Hall 传感器表检测

### 5.1 原理

Hall 传感器输出 3 位数字信号，共 8 种组合（但很多组合不会出现，最终通常 6 种有效）。需要在**已知电角度**的情况下，记录每种 Hall 组合对应的角度。

### 5.2 代码实现（[mcpwm_foc.c:2368](mcpwm_foc.c#L2368)）

```c
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result) {

    // ======== 步骤 1：电流闭环 + 固定磁场方向 ========
    motor->m_phase_override = true;
    motor->m_control_mode = CONTROL_MODE_CURRENT;
    motor->m_phase_now_override = 0;

    // 平滑升流到目标值
    for (int i = 0; i < 1000; i++) {
        motor->m_id_set = (float)i * current / 1000.0;
        chThdSleepMilliseconds(1);
    }

    // ======== 步骤 2：正转 3 圈 → 复数平均 ========
    float sin_hall[8] = {0}, cos_hall[8] = {0};
    int   hall_iterations[8] = {0};

    for (int i = 0; i < 3; i++) {       // 扫 3 圈
        for (int j = 0; j < 360; j++) {  // 1°步进
            motor->m_phase_now_override = DEG2RAD_f(j);
            chThdSleepMilliseconds(5);

            int hall = utils_read_hall(...);  // 读 Hall 端口

            float s, c;
            sincosf(m_phase_now_override, &s, &c);
            sin_hall[hall] += s;
            cos_hall[hall] += c;
            hall_iterations[hall]++;
        }
    }

    // ======== 步骤 3：反转 3 圈 → 同上（消除滞后） ========
    for (int i = 0; i < 3; i++) {
        for (int j = 360; j >= 0; j--) {
            // ... 同正转
        }
    }

    // ======== 步骤 4：对每个 Hall 状态求平均角度 ========
    for (int i = 0; i < 8; i++) {
        float ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));

        if (hall_iterations[i] > 30) {
            // 有效 Hall 状态 → 编码为 0~200 的整数存表
            hall_table[i] = (uint8_t)(ang * 200.0 / 360.0);
        } else {
            // 无效 Hall 状态（罕见组合，实际不存在）
            hall_table[i] = 255;
        }
    }

    *result = true;
}
```

**关键设计**：
- 正转 3 圈 + 反转 3 圈 = 消除磁滞
- 复数平均（`atan2(Σsin, Σcos)`）处理每个 Hall 状态内的角度分布
- `hall_iterations[i] > 30` 过滤掉不存在的 Hall 组合（如 111/000 在某些传感器中不存在）

---

## 六、编码器检测（已在文档第十章详述）

简要回顾三步骤：

1. **Index 查找**：绝对编码器直接返回 true
2. **方向+减速比检测**：对比 override 角度 vs 编码器角度，用复数平均拟合比例和方向
3. **零位偏移标定**：旋转扫多个位置，`atan2(Σsin(diff), Σcos(diff))`

详见文档第十章 §10.2。

---

## 七、一键全自动检测的调度

`conf_general_detect_apply_all_foc()`（[conf_general.c:1738](conf_general.c#L1738)）把以上步骤串联起来，支持双电机：

```c
int conf_general_detect_apply_all_foc(float max_power_loss, ...) {
    // 0. DC 偏移校准（电流 ADC 零点）
    mcpwm_foc_dc_cal(false);

    // 1-2. 测 R + L（多档电流迭代找到不饱和的测量点）
    fault = measure_r_l_imax(current_min, current_max, max_power_loss,
                              &r, &l, &ld_lq_diff, &i_max);

    // 3. 测磁链（用 FOC 开环法）
    fault = conf_general_measure_flux_linkage_openloop(
        i_max / 2.5, 0.3, 1800, r, l, &linkage, ...);

    // 4. Hall 表（如果有 Hall 传感器）
    if (m_sensor_port_mode == SENSOR_PORT_MODE_HALL) {
        mcpwm_foc_hall_detect(current, hall_table, &res);
    }

    // 5. 编码器（如果有编码器）
    if (encoder_is_configured()) {
        mcpwm_foc_encoder_detect(current, false, &offset, &ratio, &inverted);
    }

    // 6. 所有结果写入配置 → 保存到 Flash
    mcconf->foc_motor_r = r;
    mcconf->foc_motor_l = l / 1e6;
    mcconf->foc_motor_ld_lq_diff = ld_lq_diff / 1e6;
    mcconf->foc_motor_flux_linkage = linkage;
    mcconf->foc_encoder_offset = offset;
    mcconf->foc_encoder_ratio = ratio;
    mcconf->foc_encoder_inverted = inverted;
    memcpy(mcconf->foc_hall_table, hall_table, 8);
    conf_general_store_mcconf(mcconf);
}
```

---

## 八、总结

| 参数 | 方法 | 物理原理 | 耗时 |
|------|------|---------|------|
| R | 直流注入，V/I | 静止时 did/dt=0，vd=R·id | ~2s |
| Ld, Lq | HFI 高频注入 + FFT | L = V·dt/di，FFT bin0+bin2 分离平均和2倍频分量 | ~5s |
| λ | BLDC 空载旋转测 BEMF 或 FOC 开环观测器 | λ = BEMF / ωe | ~5s |
| Hall 表 | 360°旋转，每度记录 Hall vs 电角度 | 复数平均 | ~15s |
| 编码器 | 旋转扫描，复数平均角度差 | offset = atan2(Σsin_diff, Σcos_diff) | ~10s |

**与你的 Li_FOC 最大的差距**：

你所有的电机参数（R=0.198, Lq=0.000074, Ld=0.000040）都是手动填写的 `#define` 宏。VESC 的这套自动检测让你：

1. **换电机不需要改代码** → 一键测完自动写入 Flash
2. **参数更准确** → 多次采样取平均 + 复数平均，比手动估算精确得多
3. **可以检测磁链 λ** → 你的代码完全没有磁链这个参数，但在 FOC 观测器中磁链是最关键的参数之一（决定了反电势和转矩常数）
4. **自动检测凸极性** → 你的 Ld/Lq 差异是手动填的，不知道准不准。VESC 直接通过 FFT 2 倍频分量测量凸极性
