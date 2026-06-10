#include "at32f403a_407.h"              // Device header
#include "FOC.h"
#include "foc_config.h"
#include "math.h"
#include "my_math.h"
#include "fast_sin.h"
#include "delay.h"
#include "stdio.h"
#include "mt6701.h"
#include "filter.h"
#include "current_control.h"
#include "usart3.h"

#include "freertos_app.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
float g_udc = 24.0f;                    // 母线电压（单位：V），用于 SVPWM 计算

float  zero = 0.0f;                     // 保留通用零值变量
volatile int g_motorAdValues[3] = {0};  // ADC 三相电流 DMA 采样原始值
volatile uint16_t g_ADoffest[3] = {0};  // ADC 三相补偿值

int cnt = 0;                            // 通用计数变量

float g_virtualElecSpeed = 0.0f;        /* 使用编码器 */

/******************************************************************************
 * 电机 FOC 状态对象
 * 定义并初始化电机全局状态对象 Motor，g_pMotor 是其指针别名。
 * ID/IQ 各 PID 参数及零偏移等在运行阶段的初始化函数中设置。
 ******************************************************************************/
FocState Motor = {
    .current = {
        .adA = 0, .adB = 0, .adC = 0,
        .voltageAOffset = 0, .voltageBOffset = 0, .voltageCOffset = 0,
    },
    .uAlpha = 0.0f, .uBeta = 0.0f,
    .iAlpha = 0.0f, .iBeta = 0.0f,
    .Ia = 0.0f, .Ib = 0.0f, .Ic = 0.0f,
    .ua = 0.0f, .ub = 0.0f, .uc = 0.0f,
    .uq = 0.0f, .ud = 0.0f,
    .iq = 0.0f, .id = 0.0f,
    .tariqMax = FOC_IQ_MAX,
    .rs = 0.198f, .lq = 0.000074f, .ld = 0.000040f,

    .mechanicalAngle = 0.0f,
    .electricalAngle = 0.0f,
    .correctedAngle = 0.0f,
    .zeroOffset = 0.0f,
    .sensoredMechanicalAngle = 0.0f,
    .sensoredCorrectedAngle = 0.0f,

    .speedLastAngle = 0.0f,
    .speed = 0.0f,

    .ctrolmode = 1,                     // 默认开环模式
    .setPwmCallback = NULL
};

PFocState g_pMotor = &Motor;            // 电机对象的全局指针

void MotorSetPwm(float ua, float ub, float uc);

/******************************************************************************
 * 函数名称：getAdoffset
 * 功能描述：通过多次采样计算三相电流 ADC 的零点偏置，
 *           用于后续电流测量时减去偏置得到真实值。
 * 输入参数：无
 * 输出参数：无
 * 注意事项：在电机静止、无电流时调用，默认采样 16 次取平均。
 ******************************************************************************/
void getAdoffset(void)
{
    int offestA = 0, offestB = 0, offestC = 0;

    for (int i = 0; i < 16; i++)
    {
        offestA += g_motorAdValues[0];
        offestB += g_motorAdValues[1];
        offestC += g_motorAdValues[2];
    }
    g_pMotor->current.voltageAOffset = offestA >> 4;
    g_pMotor->current.voltageBOffset = offestB >> 4;
    g_pMotor->current.voltageCOffset = offestC >> 4;

    printf("voltageAOffset: %d, voltageBOffset: %d, voltageCOffset: %d\r\n",
           g_pMotor->current.voltageAOffset,
           g_pMotor->current.voltageBOffset,
           g_pMotor->current.voltageCOffset);
}

/******************************************************************************
 * 函数名称：NormalizeAngle
 * 功能描述：将任意角度归一化到 [0, 2π) 区间。
 *           使用 fmodf 取余后处理负数情况。
 * 输入参数：angle - 输入角度（单位：rad）
 * 返回值：归一化后的角度，范围 [0, 2π)
 ******************************************************************************/
float NormalizeAngle(float angle)
{
    float result = fmodf(angle, FOC_2PI);
    return (result >= 0.0f) ? result : (result + FOC_2PI);
}

/******************************************************************************
 * 函数名称：CalculateElectricalAngle
 * 功能描述：计算电角度：电角度 = 方向 × 机械角度 × 极对数。
 *           结果归一化到 [0, 2π)。
 * 输入参数：mechAngle - 机械角度（单位：rad）
 * 返回值：电角度（单位：rad），范围 [0, 2π)
 ******************************************************************************/
float CalculateElectricalAngle(float mechAngle)
{
    float elecAngle = g_pMotor->dir * mechAngle * g_pMotor->pole_pairs;
    return NormalizeAngle(elecAngle);
}

/******************************************************************************
 * 函数名称：AngleGetCorrectedElec
 * 功能描述：获取修正后的电角度。
 *           从编码器机械角度计算出电角度后，减去零位偏移得到修正后的电角度。
 * 输入参数：mechAngle - 机械角度（单位：rad）
 * 返回值：修正后的电角度（单位：rad），范围 [0, 2π)
 ******************************************************************************/
float AngleGetCorrectedElec(float mechAngle)
{
    float elecAngle = CalculateElectricalAngle(mechAngle);
    float corrected = elecAngle - g_pMotor->zeroOffset;
    corrected = NormalizeAngle(corrected);
    return corrected;
}


/* 取绝对值（避免链接标准库 fabsf，减小代码体积） */
static float FOC_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/* 取符号：正 → 1.0f，负 → -1.0f */
static float FOC_SignF(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

/* 取两数中较小值 */
static float FOC_MinF(float a, float b)
{
    return (a < b) ? a : b;
}


/******************************************************************************
 * 函数名称：MotorApplyStrongDrag
 * 功能描述：施加恒定的 Ud 电压进行强制对准（锁定电角度）。
 *           用于初始角度标定：给一个固定方向的磁场，让转子锁到已知位置。
 * 输入参数：ud - d 轴电压幅值（单位：V）
 ******************************************************************************/
void MotorApplyStrongDrag(float ud)
{
    float uAlpha = 0.0f;
    float uBeta = 0.0f;
    float uq = 0.0f;

    float angleEl = CalculateElectricalAngle(0.0f);

    // 逆 Park 变换（dq → αβ）
    uAlpha = -uq * fast_sin(angleEl) + ud * fast_cos(angleEl);
    uBeta  =  uq * fast_cos(angleEl) + ud * fast_sin(angleEl);

    // 逆 Clarke 变换（αβ → ABC）并叠加母线中点电压偏移
    float ua = uAlpha + g_udc / 2.0f;
    float ub = (FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;
    float uc = (-FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;

    MotorSetPwm(ua, ub, uc);
}

/******************************************************************************
 * 函数名称：AngleInitZeroOffset
 * 功能描述：零电角度标定。
 *           过程：
 *           1. 施加 Ud 强拖电压锁定转子
 *           2. 多次采样编码器角度取平均值
 *           3. 计算电角度平均值作为零偏
 *           4. 停止强拖并恢复 ADC 中断
 * 输入参数：zeroOffset         - 输出：零电角度偏移
 *           correctedElecAngle - 输出：当前修正后的电角度
 ******************************************************************************/
void AngleInitZeroOffset(float *zeroOffset, float *correctedElecAngle)
{
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, FALSE);
    MotorApplyStrongDrag(FOC_STRONGDRAG);
    vTaskDelay(1000);                       // 保持强拖 1 秒让转子稳定

    float sumSin = 0.0f, sumCos = 0.0f;
    const int sampleCount = 10;

    // 两次读取丢弃不稳定值
    Mt6701GetAngleWrapper();
    Mt6701GetAngleWrapper();

    for (int i = 0; i < sampleCount; i++) {
        float mechanicalAngle = Mt6701GetAngleWrapper();
        float elecAngle = CalculateElectricalAngle(mechanicalAngle);
        sumSin += sinf(elecAngle);
        sumCos += cosf(elecAngle);
        vTaskDelay(10);
    }
    *zeroOffset = NormalizeAngle(atan2f(sumSin, sumCos));

    float mechanicalAngle = Mt6701GetAngleWrapper();
    float elecAngle = CalculateElectricalAngle(mechanicalAngle);
    *correctedElecAngle = NormalizeAngle(elecAngle - *zeroOffset);

    vTaskDelay(500);

    MotorApplyStrongDrag(0.0f);             // 停止强拖
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
}

/* ========== 角度工具 ========== */

/* 将角度差值折算到 [-π, +π] */
static float WrapPi(float x)
{
    while (x >  FOC_PI) x -= FOC_2PI;
    while (x < -FOC_PI) x += FOC_2PI;
    return x;
}

/* 用 Ud 电压将转子锁到指定电角度 */
static void MotorLockAtElecAngle(float elecAngle, float ud)
{
    float uAlpha = ud * fast_cos(elecAngle);
    float uBeta  = ud * fast_sin(elecAngle);
    float ua = uAlpha + g_udc / 2.0f;
    float ub = (FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;
    float uc = (-FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;
    MotorSetPwm(ua, ub, uc);
}

/******************************************************************************
 * 函数名称：VerifyZeroOffset
 * 功能描述：多点静态锁定验证零偏精度。
 *
 * 依次将转子锁在 0°/60°/120°/180°/240°/300° 电角度，
 * 分别读取校正后的电角度，计算与命令角度的偏差。
 * 结果通过 USART3 回传 6 个误差值（单位：rad）。
 *
 * 调用建议：先执行一次零位校准（CMD_ZEROCALIBRATIO），
 *          再调用本命令。
 ******************************************************************************/
void VerifyZeroOffset(void)
{
    /* 测试点：0°, 60°, 120°, 180°, 240°, 300° */
    const float testAnglesDeg[] = {0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f};
    float errors[6];

    adc_interrupt_enable(ADC1, ADC_PCCE_INT, FALSE);

    for (int i = 0; i < 6; i++) {
        float thetaCmd = testAnglesDeg[i] * FOC_2PI / 360.0f;
        MotorLockAtElecAngle(thetaCmd, FOC_STRONGDRAG);
        vTaskDelay(1000);   // 等转子吸稳

        float mechAngle = Mt6701GetAngleWrapper();
        float corrected = AngleGetCorrectedElec(mechAngle);
        errors[i] = WrapPi(corrected - thetaCmd);
    }

    MotorApplyStrongDrag(0.0f);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);

    USART3_SendPacket(0x5B, errors, 6);
}

/******************************************************************************
 * 函数名称：adc_tigger
 * 功能描述：触发 ADC 采样的 PWM 时序控制。
 *           通过 TIM2 的通道 3 输出一个提前触发的信号来同步 ADC 采样。
 * 输入参数：time_pwm - PWM 周期计数值
 ******************************************************************************/
void adc_tigger(int time_pwm)
{
    tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, time_pwm - 10);
}

/******************************************************************************
 * 全局 PWM 占空比缓存变量
 ******************************************************************************/
float g_pwmA = 0.0f;
float g_pwmB = 0.0f;
float g_pwmC = 0.0f;

/******************************************************************************
 * 函数名称：setpwm_channel
 * 功能描述：底层 PWM 寄存器写入。
 *           将归一化占空比（0~1）乘以 FOC_ALL_DUTY 后写入 TIM1 比较寄存器。
 * 输入参数：pwm_a/b/c - 归一化占空比（0~1）
 ******************************************************************************/
static void setpwm_channel(float pwm_a, float pwm_b, float pwm_c)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, (uint32_t)(pwm_a * FOC_ALL_DUTY));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, (uint32_t)(pwm_b * FOC_ALL_DUTY));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, (uint32_t)(pwm_c * FOC_ALL_DUTY));
}

/******************************************************************************
 * 函数名称：MotorSetPwm
 * 功能描述：设置三相 PWM 输出。
 *           输入三相电压值（Ua/Ub/Uc），经过限幅和归一化后写入 PWM 寄存器。
 * 输入参数：ua - A 相电压（单位：V）
 *           ub - B 相电压（单位：V）
 *           uc - C 相电压（单位：V）
 ******************************************************************************/
void MotorSetPwm(float ua, float ub, float uc)
{
    // 电压限幅
    ua = LimitValue(ua, 0.0f, g_udc);
    ub = LimitValue(ub, 0.0f, g_udc);
    uc = LimitValue(uc, 0.0f, g_udc);

    // 电压 → 占空比归一化
    g_pwmA = LimitValue(ua / g_udc, 0.0f, 1.0f);
    g_pwmB = LimitValue(ub / g_udc, 0.0f, 1.0f);
    g_pwmC = LimitValue(uc / g_udc, 0.0f, 1.0f);

    setpwm_channel(g_pwmA, g_pwmB, g_pwmC);
}

/******************************************************************************
 * 函数名称：CurrentReconstruction
 * 功能描述：单电阻/双电阻采样电流重构。
 *           根据当前 SVPWM 扇区，利用 Ia + Ib + Ic = 0 补全无法直接采样的那一相。
 *           注意：本实现各 case 语句中有些是同类操作，暂未做合并优化，
 *           后续可根据实际采样拓扑统一。
 * 输入参数：pFOC  - FOC 状态指针
 *           PSVpwm - SVPWM 状态指针（含扇区信息）
 *           ia, ib, ic - 直接采样到的三相电流
 ******************************************************************************/
static void CurrentReconstruction(PFocState pFOC, PSVpwm_State PSVpwm, float ia, float ib, float ic)
{
    (void)ia;
    (void)ib;
    (void)ic;
    switch (PSVpwm->sector) {
        case 1:
            pFOC->Ib = 0.0f - pFOC->Ia - pFOC->Ic;
            break;
        case 2:
            pFOC->Ia = 0.0f - pFOC->Ib - pFOC->Ic;
            break;
        case 3:
            pFOC->Ia = 0.0f - pFOC->Ib - pFOC->Ic;
            break;
        case 4:
            pFOC->Ic = 0.0f - pFOC->Ia - pFOC->Ib;
            break;
        case 5:
            pFOC->Ib = 0.0f - pFOC->Ia - pFOC->Ic;
            break;
        case 6:
            pFOC->Ic = 0.0f - pFOC->Ia - pFOC->Ib;
            break;
        default:
            break;
    }
}

/******************************************************************************
 * 函数名称：clarke_transform
 * 功能描述：Clarke 变换（三相 ABC → 两相 αβ）。
 *           将 ABC 自然坐标系电流变换到 αβ 静止坐标系。
 *           使用幅值不变约定：Ialpha = Ia, Ibeta = (Ia + 2*Ib) / sqrt(3)
 * 输入参数：Ia, Ib - A/B 相电流
 * 输出参数：Ialpha, Ibeta - αβ 轴电流
 ******************************************************************************/
void clarke_transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (1.0f / FOC_SQRT3) * (Ia + 2.0f * Ib);
}

/******************************************************************************
 * 函数名称：park_transform
 * 功能描述：Park 变换（静止 αβ → 旋转 dq）。
 *           使用电角度将 αβ 电流变换到与转子同步旋转的 dq 坐标系。
 *           Id = Ialpha*cos(θ) + Ibeta*sin(θ)
 *           Iq = -Ialpha*sin(θ) + Ibeta*cos(θ)
 * 输入参数：Ialpha, Ibeta - αβ 轴电流
 *           angle_el       - 电角度
 * 输出参数：Id, Iq - dq 轴电流
 ******************************************************************************/
void park_transform(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq)
{
    *Id = Ialpha * fast_cos(angle_el) + Ibeta * fast_sin(angle_el);
    *Iq = -Ialpha * fast_sin(angle_el) + Ibeta * fast_cos(angle_el);
}

/******************************************************************************
 * 函数名称：setSVpwm
 * 功能描述：将 SVPWM 计算的导通时间写入 PWM 寄存器。
 * 输入参数：PSVpwm - SVPWM 状态结构体指针
 ******************************************************************************/
static void setSVpwm(PSVpwm_State PSVpwm)
{
    setpwm_channel(PSVpwm->Ta, PSVpwm->Tb, PSVpwm->Tc);
}

/******************************************************************************
 * 函数名称：inv_park_transform
 * 功能描述：逆 Park 变换（旋转 dq → 静止 αβ）。
 *           将 dq 电压指令变换到 αβ 坐标系，用于 SVPWM 调制。
 *           Ualpha = -Uq*sin(θ) + Ud*cos(θ)
 *           Ubeta  =  Uq*cos(θ) + Ud*sin(θ)
 * 输入参数：Uq, Ud     - dq 轴电压指令
 *           corr_angle - 修正后的电角度
 * 输出参数：Out_Ualpha, Out_Ubeta - αβ 轴电压
 ******************************************************************************/
static void inv_park_transform(float Uq, float Ud, float corr_angle,
                               float *Out_Ualpha, float *Out_Ubeta)
{
    *Out_Ualpha = -Uq * fast_sin(corr_angle) + Ud * fast_cos(corr_angle);
    *Out_Ubeta  =  Uq * fast_cos(corr_angle) + Ud * fast_sin(corr_angle);
}

/******************************************************************************
 * 函数名称：FocContorl
 * 功能描述：FOC 主控制函数（在 ADC 中断中调用，每个 PWM 周期执行一次）。
 *
 * 执行流程：
 *   1. 读取 MT6701 编码器角度，计算修正电角度
 *   2. 读取三相电流 ADC 值 → 计算实际电流值
 *   3. 电流重构（补全未采样的相）
 *   4. Clarke 变换（ABC → αβ）
 *   5. Park 变换（αβ → dq）
 *   6. Id/Iq 低通滤波
 *   7. Id/Iq PI 控制
 *   8. 逆 Park 变换（dq → αβ）
 *   9. SVPWM 调制
 *
 * 输入参数：pFOC   - FOC 状态指针
 *           PSVpwm - SVPWM 状态指针
 *
 * 注意事项：当前 Id 给定为极小值（0.000001），目的是维持 Id≈0 控制策略。
 *           开环模式下不更新 PID 输出（保持 Uq/Ud 手动设定值）。
 ******************************************************************************/
void FocContorl(PFocState pFOC, PSVpwm_State PSVpwm)
{
    /* ==== 步骤 1：获取角度 ==== */
    static float s_vAngle = 0.0f, s_rSpeed = 0.0f;
    /* 始终读取编码器（用于遥测） */
    pFOC->sensoredMechanicalAngle = Mt6701GetAngleWrapper();
    pFOC->sensoredCorrectedAngle = AngleGetCorrectedElec(pFOC->sensoredMechanicalAngle);

    if (g_virtualElecSpeed != 0.0f) {
        if (s_rSpeed < g_virtualElecSpeed) { s_rSpeed += 80.0f*FOC_PWM_TS; if(s_rSpeed>g_virtualElecSpeed)s_rSpeed=g_virtualElecSpeed; }
        s_vAngle += s_rSpeed * FOC_PWM_TS;
        s_vAngle = NormalizeAngle(s_vAngle);
        pFOC->correctedAngle = s_vAngle;
        pFOC->mechanicalAngle = pFOC->sensoredMechanicalAngle;
    } else {
        s_vAngle = 0.0f; s_rSpeed = 0.0f;
        pFOC->mechanicalAngle = pFOC->sensoredMechanicalAngle;
        pFOC->correctedAngle = pFOC->sensoredCorrectedAngle;
    }

    /* ==== 步骤 2：读取电流 ==== */
    pFOC->current.adA = g_motorAdValues[0];
    pFOC->current.adB = g_motorAdValues[1];
    pFOC->current.adC = g_motorAdValues[2];

    /* ==== 步骤 3：ADC 值 → 实际电流
     * I = (ADC_raw - offset) / 4096 * Vref / Gain / Rshunt
     */
    pFOC->Ia = (pFOC->current.adA - pFOC->current.voltageAOffset) / 4096.0f
               * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
    pFOC->Ib = (pFOC->current.adB - pFOC->current.voltageBOffset) / 4096.0f
               * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
    pFOC->Ic = (pFOC->current.adC - pFOC->current.voltageCOffset) / 4096.0f
               * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;

    /* ==== 步骤 4：电流重构 + 坐标变换 ==== */
    CurrentReconstruction(g_pMotor, PSVpwm, pFOC->Ia, pFOC->Ib, pFOC->Ic);

    clarke_transform(-pFOC->Ia, -pFOC->Ib, &pFOC->iAlpha, &pFOC->iBeta);
    park_transform(pFOC->iAlpha, pFOC->iBeta, pFOC->correctedAngle,
                   &pFOC->id, &pFOC->iq);

    /* ==== 步骤 5：Id/Iq 滤波 ==== */
    LPF_Update(PM1_LPF, pFOC->id, pFOC->iq, &(pFOC->id), &(pFOC->iq));

    /* ==== 步骤 6：电流 PI 控制 ==== */
    pFOC->ud = 0.000001f;
    CurrentPIControlIQ(pFOC);
    CurrentPIControlID(pFOC);

    /* 非开环模式下采用 PID 输出值 */
    if (g_pMotor->ctrolmode != FOC_OPEN_LOOP) {
        pFOC->uq = pFOC->iqPID.out;
        pFOC->ud = pFOC->idPID.out;
    }

    /* ==== 步骤 7：逆 Park → SVPWM 调制 ==== */
    inv_park_transform(pFOC->uq, pFOC->ud, pFOC->correctedAngle,
                       &(pFOC->uAlpha), &(pFOC->uBeta));

    SVpwm(PSVpwm, pFOC->uAlpha, pFOC->uBeta);
    setSVpwm(PSVpwm);
}
