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
#include "smo_observer.h"

#include "freertos_app.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
float g_udc = 24.0f;                    // 母线电压（单位：V），用于 SVPWM 计算

float  zero = 0.0f;                     // 保留通用零值变量
volatile int g_motorAdValues[3] = {0};  // ADC 三相电流 DMA 采样原始值
volatile uint16_t g_ADoffest[3] = {0};  // ADC 三相补偿值

int cnt = 0;                            // 通用计数变量

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
    .sensorlessElectricalAngle = 0.0f,
    .sensorlessMechanicalSpeed = 0.0f,
    .sensorlessOpenLoopAngle = 0.0f,
    .sensorMode = FOC_SENSOR_MODE_SENSORED,
    .sensorlessIfState = FOC_SENSORLESS_IF_OFF,
    .sensorlessIfAlignCount = 0U,
    .sensorlessIfLockCount = 0U,
    .sensorlessIfAngle = 0.0f,
    .sensorlessIfSpeed = 0.0f,
    .sensorlessIfIq = 0.0f,
    .sensorlessIfId = 0.0f,

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

/******************************************************************************
 * 函数名称：FOC_SetSensorMode
 * 功能描述：切换 FOC 角度/速度反馈来源，并重置所有相关状态。
 *
 *   有感模式 (SENSORED)  ：使用 MT6701 磁编码器的机械角度，
 *                          经极对数和零位偏移换算后得到电角度。
 *   无感模式 (SENSORLESS)：使用 SMO + PLL 估算的电角度和转速。
 *
 * 切换时会清零：
 *   - 速度环 / 位置环 PI 积分器（避免旧模式残余输出）
 *   - I/F 启动状态机（回到 OFF，重新开始）
 *   - 切换到无感时额外初始化开环虚拟角度并复位 SMO
 ******************************************************************************/
void FOC_SetSensorMode(PFocState pFOC, FocSensorMode_TypeDef mode)
{
    if (pFOC == NULL) {
        return;
    }

    if (mode == FOC_SENSOR_MODE_SENSORLESS) {
        pFOC->sensorMode = FOC_SENSOR_MODE_SENSORLESS;
        pFOC->sensorlessOpenLoopAngle = pFOC->correctedAngle;
        SMO_Reset(&g_smoObserver);
    } else {
        pFOC->sensorMode = FOC_SENSOR_MODE_SENSORED;
    }

    pFOC->speedPID.out = 0.0f;
    pFOC->speedPID.lastBias = 0.0f;
    pFOC->positionPID.lastBias = 0.0f;
    pFOC->sensorlessIfState = FOC_SENSORLESS_IF_OFF;
    pFOC->sensorlessIfAlignCount = 0U;
    pFOC->sensorlessIfLockCount = 0U;
    pFOC->sensorlessIfSpeed = 0.0f;
    pFOC->sensorlessIfIq = 0.0f;
    pFOC->sensorlessIfId = 0.0f;
}

/******************************************************************************
 * 函数名称：FOC_GetSensorMode
 * 功能描述：读取当前 FOC 角度/速度反馈来源（有感/无感）。
 * 返回值：FocSensorMode_TypeDef 枚举，pFOC 为 NULL 时默认返回有感。
 ******************************************************************************/
FocSensorMode_TypeDef FOC_GetSensorMode(PFocState pFOC)
{
    if (pFOC == NULL) {
        return FOC_SENSOR_MODE_SENSORED;
    }
    return (FocSensorMode_TypeDef)pFOC->sensorMode;
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
 * 函数名称：FOC_ResetSensorlessIF
 * 功能描述：复位无感 I/F 启动状态机到 OFF。
 *
 * 将 I/F 相关状态全部清零，回到未启动状态。速度环和电流环的 PI
 * 积分器由调用者负责清理（本函数不处理，因为不同调用场景对 PI
 * 清理的需求可能不同）。
 *
 * 调用时机：
 *   - 退出速度环/无感模式时
 *   - 用户目标速度降到死区以下（主动停机）
 *   - DONE 状态下转速跌入 SMO 盲区（外力堵转，需要重新 I/F）
 *   - RAMP 阶段超时（转子被卡死，需重新 ALIGN）
 ******************************************************************************/
static void FOC_ResetSensorlessIF(PFocState pFOC)
{
    pFOC->sensorlessIfState = FOC_SENSORLESS_IF_OFF;
    pFOC->sensorlessIfAlignCount = 0U;
    pFOC->sensorlessIfLockCount = 0U;
    pFOC->sensorlessIfSpeed = 0.0f;
    pFOC->sensorlessIfIq = 0.0f;
    pFOC->sensorlessIfId = 0.0f;
}

/******************************************************************************
 * 函数名称：FOC_StartSensorlessIF
 * 功能描述：启动无感 I/F（电流/频率）起动序列。
 *
 * I/F 起动是开环拖动策略，用于在 SMO 无法工作的低速区间让电机
 * 先转起来，等到反电动势足够大再切换到 SMO 闭环。
 *
 * 起动分为两个阶段：
 *   ALIGN（对齐）：固定虚拟角度 + Id 电流注入，让转子磁极对齐
 *                  到已知位置，持续 FOC_SENSORLESS_IF_ALIGN_COUNT 个周期。
 *   RAMP （斜坡）：虚拟角度开始旋转，Iq 电流拖动转子跟随磁场，
 *                  虚拟速度从 START_SPEED 按 RAMP_RPM_PER_S 斜坡上升，
 *                  直到满足 SMO/PLL 交接条件后切换到 DONE 闭环。
 *
 * 调用时机：FocContorl 检测到 target > 死区、实际转速 < I/F 阈值、
 *          I/F 处于 OFF 状态时触发。
 *
 * 注意：调用本函数前应先清理 SMO 和 PI 积分器，确保 I/F 从干净
 *       状态启动（参见 FocContorl 中 I/F 启动条件内的清理代码）。
 ******************************************************************************/
static void FOC_StartSensorlessIF(PFocState pFOC)
{
    float targetSign = FOC_SignF(pFOC->tar_speed);
    float iqLimit = FOC_AbsF(pFOC->tariqMax);
    float iqStart = FOC_SENSORLESS_IF_START_IQ;

    if (iqLimit > FOC_EPSILON) {
        iqStart = FOC_MinF(iqStart, iqLimit);
    }

    pFOC->sensorlessIfState = FOC_SENSORLESS_IF_ALIGN;
    pFOC->sensorlessIfAlignCount = 0U;
    pFOC->sensorlessIfLockCount = 0U;
    pFOC->sensorlessIfAngle = pFOC->correctedAngle;
    pFOC->sensorlessIfSpeed = 0.0f;
    pFOC->sensorlessIfIq = 0.0f;
    if (iqLimit > FOC_EPSILON) {
        pFOC->sensorlessIfId = FOC_MinF(FOC_SENSORLESS_IF_ALIGN_ID, iqLimit);
    } else {
        pFOC->sensorlessIfId = FOC_SENSORLESS_IF_ALIGN_ID;
    }

    pFOC->speedPID.out = 0.0f;
    pFOC->speedPID.lastBias = 0.0f;
    pFOC->idPID.lastBias = 0.0f;
    pFOC->iqPID.lastBias = 0.0f;

    (void)targetSign;
    (void)iqStart;
}

/******************************************************************************
 * 函数名称：FOC_UpdateSensorlessIF
 * 功能描述：无感 I/F 启动状态机更新（每个 PWM 周期调用一次）。
 *
 * 状态机流程：
 *   ALIGN（对齐阶段）
 *     - 虚拟角度固定不动，注入 Id 电流（默认 0.5A）产生定向磁场
 *     - 转子在电磁力作用下自动对齐到该磁场方向
 *     - 持续 FOC_SENSORLESS_IF_ALIGN_COUNT 个周期（默认 6000 = 300ms）
 *     - 超时后切换为 RAMP
 *
 *   RAMP（斜坡拖动阶段）
 *     - 虚拟角度按给定转速积分，虚拟机械速度从 START_SPEED_RPM（20rpm）
 *       按 RAMP_RPM_PER_S（1500rpm/s）斜坡上升到 HANDOVER_SPEED_RPM（1000rpm）
 *     - Iq 电流（默认 1A）产生旋转磁场拖动转子跟随
 *     - 每周期检查 SMO/PLL 交接条件：
 *         1. 虚拟速度达到 1000rpm
 *         2. PLL 估算的机械转速 > 500rpm（目标方向）
 *         3. PLL 机械转速幅值 > 500rpm
 *         4. 反电势幅值 > FOC_SENSORLESS_IF_MIN_EMAG（0.15V）
 *         5. PLL 归一化误差 < FOC_SENSORLESS_IF_MAX_PLL_ERROR（0.35）
 *     - 全部 5 条连续满足 LOCK_COUNT（200）个周期 → 切换为 DONE（闭环）
 *     - 任一条件不满足时锁存计数器清零
 *
 *   DONE（闭环运行）
 *     - 由 FocContorl 中的条件判断切换至此，本函数不再介入
 *     - 后续使用 SMO/PLL 角度和速度进行正常 FOC 闭环控制
 *
 * 注：RAMP 超时保护不在本函数内，由 FocContorl 在调用后检查。
 ******************************************************************************/
static void FOC_UpdateSensorlessIF(PFocState pFOC)
{
    float targetSign = FOC_SignF(pFOC->tar_speed);
    float iqLimit = FOC_AbsF(pFOC->tariqMax);
    float iqStart = FOC_SENSORLESS_IF_START_IQ;
    float handoverAbs = FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM;
    float speedAbs = FOC_AbsF(pFOC->sensorlessIfSpeed);
    float rampStep = FOC_SENSORLESS_IF_RAMP_RPM_PER_S * FOC_SMO_TS;
    float pllMechSpeed = 0.0f;
    float pllSpeedAbs;
    float elecSpeed;

    if (iqLimit > FOC_EPSILON) {
        iqStart = FOC_MinF(iqStart, iqLimit);
    }

    if (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_ALIGN) {
        pFOC->correctedAngle = pFOC->sensorlessIfAngle;
        if (pFOC->pole_pairs != 0) {
            pFOC->mechanicalAngle =
                NormalizeAngle(pFOC->sensorlessIfAngle / (float)pFOC->pole_pairs);
        }

        pFOC->sensorlessIfAlignCount++;
        if (pFOC->sensorlessIfAlignCount >= FOC_SENSORLESS_IF_ALIGN_COUNT) {
            pFOC->sensorlessIfState = FOC_SENSORLESS_IF_RAMP;
            pFOC->sensorlessIfSpeed = targetSign * FOC_SENSORLESS_IF_START_SPEED_RPM;
            pFOC->sensorlessIfIq = targetSign * iqStart;
            pFOC->sensorlessIfId = 0.0f;
            pFOC->sensorlessIfLockCount = 0U;
        }
        return;
    }

    if (speedAbs < handoverAbs) {
        speedAbs += rampStep;
        if (speedAbs > handoverAbs) {
            speedAbs = handoverAbs;
        }
    }

    pFOC->sensorlessIfSpeed = targetSign * speedAbs;
    elecSpeed = pFOC->sensorlessIfSpeed * FOC_2PI / 60.0f
                * (float)pFOC->pole_pairs * pFOC->speedDir;
    pFOC->sensorlessIfAngle =
        NormalizeAngle(pFOC->sensorlessIfAngle + elecSpeed * FOC_SMO_TS);

    pFOC->correctedAngle = pFOC->sensorlessIfAngle;
    if (pFOC->pole_pairs != 0) {
        pFOC->mechanicalAngle =
            NormalizeAngle(pFOC->sensorlessIfAngle / (float)pFOC->pole_pairs);
    }

    pFOC->speedPID.out = pFOC->sensorlessIfIq;

    if (pFOC->pole_pairs != 0) {
        pllMechSpeed = g_smoObserver.speed / (float)pFOC->pole_pairs
                       * pFOC->speedDir * 60.0f / FOC_2PI;
    }
    pllSpeedAbs = FOC_AbsF(pllMechSpeed);

    if ((speedAbs >= FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM)
        && ((pllMechSpeed * targetSign) > (FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM * 0.5f))
        && (pllSpeedAbs > (FOC_SENSORLESS_IF_HANDOVER_SPEED_RPM * 0.5f))
        && (g_smoObserver.eMag > FOC_SENSORLESS_IF_MIN_EMAG)
        && (FOC_AbsF(g_smoObserver.pllError) < FOC_SENSORLESS_IF_MAX_PLL_ERROR)) {
        if (pFOC->sensorlessIfLockCount < FOC_SENSORLESS_IF_LOCK_COUNT) {
            pFOC->sensorlessIfLockCount++;
        }
    } else {
        pFOC->sensorlessIfLockCount = 0U;
    }

    if (pFOC->sensorlessIfLockCount >= FOC_SENSORLESS_IF_LOCK_COUNT) {
        pFOC->sensorlessIfState = FOC_SENSORLESS_IF_DONE;
        pFOC->sensorlessIfLockCount = 0U;
        pFOC->speedPID.out = pFOC->sensorlessIfIq;
        pFOC->speedPID.lastBias = pFOC->tar_speed - pFOC->speed;
        pFOC->iqPID.lastBias = 0.0f;
    }
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

    float sum = 0.0f;
    const int sampleCount = 10;

    // 两次读取丢弃不稳定值
    Mt6701GetAngleWrapper();
    Mt6701GetAngleWrapper();

    for (int i = 0; i < sampleCount; i++) {
        float mechanicalAngle = Mt6701GetAngleWrapper();
        float elecAngle = CalculateElectricalAngle(mechanicalAngle);
        sum += elecAngle;
        vTaskDelay(10);
    }
    *zeroOffset = sum / sampleCount;        // 平均电角度即为零偏

    float mechanicalAngle = Mt6701GetAngleWrapper();
    float elecAngle = CalculateElectricalAngle(mechanicalAngle);
    *correctedElecAngle = elecAngle - *zeroOffset;

    vTaskDelay(500);

    MotorApplyStrongDrag(0.0f);             // 停止强拖
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
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
 *   1. 同时维护编码器角度和 SMO/PLL 角度，按 sensorMode 选择控制角度
 *   2. 读取三相电流 ADC 值 → 计算实际电流值
 *   3. 电流重构（补全未采样的相）
 *   4. Clarke 变换（ABC → αβ）
 *   5. Park 变换（αβ → dq）
 *   6. Id/Iq 低通滤波
 *   7. Id/Iq PI 控制
 *   8. 逆 Park 变换（dq → αβ）
 *   9. SVPWM 调制
 *   10. SMO 观测器更新（在后台同步观测）
 *
 * 输入参数：pFOC   - FOC 状态指针
 *           PSVpwm - SVPWM 状态指针
 *
 * 注意事项：当前 Id 给定为极小值（0.000001），目的是维持 Id≈0 控制策略。
 *           开环模式下不更新 PID 输出（保持 Uq/Ud 手动设定值）。
 ******************************************************************************/
void FocContorl(PFocState pFOC, PSVpwm_State PSVpwm)
{
    /*
     * ================================================================
     * 低速/零速/堵转 限流保护
     * ================================================================
     * 无感 FOC 在低速（<150rpm 左右）时反电动势太弱，SMO 无法可靠
     * 估算角度。若此时速度环仍然输出大电流、配合错误的 SMO 角度
     * 做 Park 变换，会产生随机方向的转矩 → 电机剧烈抖动甚至反转。
     *
     * 保护策略：检测到以下任一情况时，将 Iq 上限压到 0.05A——
     *   a) 用户主动停机：tar_speed ≈ 0
     *   b) 外力堵转：tar_speed 非零但实际转速已跌到 SMO 盲区
     *
     * tariq / speedPID.outMax 的原始值在进入保护时自动保存，
     * 退出时自动恢复，避免限流值泄漏到正常工作模式。
     *
     * I/F 启动过程中（ALIGN/RAMP）不触发此保护，因为 I/F 自己
     * 管理电流（sensorlessIfIq/sensorlessIfId）。
     */
    static float  savedTariq   = 0.0f;
    static float  savedOutMax  = 0.0f;
    static uint8_t wasLowSpeed = 0;

    uint8_t inLowSpeedStop = (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORLESS)
                          && (pFOC->ctrolmode == FOC_SPEED_LOOP)
                          && (pFOC->sensorlessIfState != FOC_SENSORLESS_IF_ALIGN)
                          && (pFOC->sensorlessIfState != FOC_SENSORLESS_IF_RAMP)
                          && (FOC_AbsF(pFOC->tar_speed) <= FOC_SENSORLESS_IF_MIN_TARGET_RPM
                              || FOC_AbsF(pFOC->speed) < FOC_SENSORLESS_IF_START_MAX_SPEED_RPM);

    if (inLowSpeedStop) {
        if (!wasLowSpeed) {
            savedTariq  = pFOC->tariq;
            savedOutMax = pFOC->speedPID.outMax;
            wasLowSpeed = 1;
        }
        pFOC->tariq           = 0.05f;
        pFOC->speedPID.outMax = 0.05f;
    } else {
        if (wasLowSpeed) {
            pFOC->tariq           = savedTariq;
            pFOC->speedPID.outMax = savedOutMax;
            wasLowSpeed          = 0;
        }
    }

    /* ==== 步骤 1：获取并选择角度 ==== */
    if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORED) {
        pFOC->sensoredMechanicalAngle = Mt6701GetAngleWrapper();
        pFOC->sensoredCorrectedAngle = AngleGetCorrectedElec(pFOC->sensoredMechanicalAngle);
    }

    pFOC->sensorlessElectricalAngle = g_smoObserver.angle;
    if (pFOC->pole_pairs != 0) {
        pFOC->sensorlessMechanicalSpeed =
            g_smoObserver.speed / (float)pFOC->pole_pairs * pFOC->speedDir
            * 60.0f / FOC_2PI;
    } else {
        pFOC->sensorlessMechanicalSpeed = 0.0f;
    }

    if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORLESS) {
        /*
         * I/F 启动条件：
         *   无感 + 速度环 + 目标速度高于死区 + I/F 处于 OFF 状态
         *   + 实际转速已跌到 I/F 启动阈值以下
         *
         * 进入 I/F 前必须做完整的状态复位，原因：
         *   电机可能在堵转/失步状态下运行了若干周期——SMO 角度是噪声、
         *   速度环和电流环的积分器可能已饱和。直接在这个脏状态上启动
         *   I/F 会导致 Align 角度随机、或残留的大电流在错误角度下输出。
         */
        if ((pFOC->ctrolmode == FOC_SPEED_LOOP)
            && (FOC_AbsF(pFOC->tar_speed) > FOC_SENSORLESS_IF_MIN_TARGET_RPM)
            && (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_OFF)
            && (FOC_AbsF(pFOC->speed) < FOC_SENSORLESS_IF_START_MAX_SPEED_RPM)) {
            SMO_Reset(&g_smoObserver);          // 丢弃已损坏的 SMO 内部状态
            pFOC->uq = 0.0f;                    // 本周期不输出电压，避免错误角度下
            pFOC->ud = 0.0f;                    //   的残余电压造成转矩冲击
            pFOC->speedPID.out = 0.0f;          // 清零速度环积分
            pFOC->speedPID.lastBias = 0.0f;
            pFOC->iqPID.out = 0.0f;             // 清零 Q 轴电流环积分
            pFOC->iqPID.lastBias = 0.0f;
            pFOC->idPID.out = 0.0f;             // 清零 D 轴电流环积分
            pFOC->idPID.lastBias = 0.0f;
            FOC_StartSensorlessIF(pFOC);        // 干净起点进入 I/F 启动序列
        }

        /*
         * I/F 复位条件（任一满足即复位到 OFF）：
         *   1. 退出速度环模式（切到开环/电流环/位置环）
         *   2. 用户将目标速度降到死区以下（主动停机）
         *   3. 正常运行中转速跌入 SMO 盲区（外力堵转）——
         *      这是关键：DONE 状态下若不主动复位，I/F 启动条件
         *      (要求 IF_OFF) 永远不会满足，电机会卡在 DONE 状态
         *      用噪声 SMO 角度持续输出错误转矩。
         */
        if ((pFOC->ctrolmode != FOC_SPEED_LOOP)
            || (FOC_AbsF(pFOC->tar_speed) <= FOC_SENSORLESS_IF_MIN_TARGET_RPM)
            || (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_DONE
                && FOC_AbsF(pFOC->speed) < FOC_SENSORLESS_IF_START_MAX_SPEED_RPM)) {
            FOC_ResetSensorlessIF(pFOC);
        }

        if ((pFOC->sensorlessIfState == FOC_SENSORLESS_IF_ALIGN)
            || (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_RAMP)) {
            FOC_UpdateSensorlessIF(pFOC);
            /*
             * RAMP 阶段超时重试：
             * 若转子被外力长时间卡住，虚拟速度会一直爬到上限但转子
             * 不动，交接条件（SMO 转速、反电势幅值、PLL 误差）永远
             * 无法满足，I/F 将永远卡在 RAMP。
             *
             * 超时后复位到 OFF → 下个周期自动重新触发 ALIGN→RAMP，
             * 形成重试循环。松手时 I/F 总能从一个新鲜的 ALIGN 开始，
             * 虚拟速度从 20rpm 起步，转子容易跟上。
             *
             * 60000 周期 = 3 秒 @ 20kHz PWM。
             */
            static uint32_t rampTimeout = 0;
            if (pFOC->sensorlessIfState == FOC_SENSORLESS_IF_RAMP) {
                if (++rampTimeout > 60000) {
                    FOC_ResetSensorlessIF(pFOC);
                    rampTimeout = 0;
                }
            } else {
                rampTimeout = 0;
            }
        } else if (pFOC->ctrolmode == FOC_OPEN_LOOP) {
            float openLoopMechSpeed;
            if (fabsf(pFOC->uq) > FOC_EPSILON) {
                openLoopMechSpeed = pFOC->uq * FOC_SENSORLESS_OPEN_LOOP_UQ_TO_SPEED;
            } else {
                openLoopMechSpeed = 0.0f;
            }
            float openLoopElecSpeed = openLoopMechSpeed * FOC_2PI / 60.0f
                                     * (float)pFOC->pole_pairs * pFOC->speedDir;
            pFOC->sensorlessOpenLoopAngle =
                NormalizeAngle(pFOC->sensorlessOpenLoopAngle + openLoopElecSpeed * FOC_SMO_TS);
            pFOC->correctedAngle = pFOC->sensorlessOpenLoopAngle;
            if (pFOC->pole_pairs != 0) {
                pFOC->mechanicalAngle =
                    NormalizeAngle(pFOC->sensorlessOpenLoopAngle / (float)pFOC->pole_pairs);
            }
        } else if (pFOC->ctrolmode == FOC_SPEED_LOOP
                   && pFOC->sensorlessIfState != FOC_SENSORLESS_IF_ALIGN
                   && pFOC->sensorlessIfState != FOC_SENSORLESS_IF_RAMP
                   && (FOC_AbsF(pFOC->tar_speed) <= FOC_SENSORLESS_IF_MIN_TARGET_RPM
                       || FOC_AbsF(pFOC->speed) < FOC_SENSORLESS_IF_START_MAX_SPEED_RPM)) {
            /*
             * 低速/零速/堵转：冻结开环虚拟角度作为 Park 变换的参考系。
             *
             * 此时 SMO 输出的角度充满噪声——反电动势幅值太低，atan2
             * 和 PLL 都无法给出有意义的估算值。若继续用 SMO 角度做
             * Park/逆 Park 变换，即使 uq/ud 很小，也会在随机角度下
             * 产生随机方向的转矩。
             *
             * 冻结一个固定角度虽然与真实转子位置不对齐，但至少是稳定
             * 的——不会产生交变转矩。配合上文的 0.05A 限流，即使角度
             * 偏差较大，实际力也很小，不会造成抖动。
             */
            pFOC->correctedAngle = pFOC->sensorlessOpenLoopAngle;
            if (pFOC->pole_pairs != 0) {
                pFOC->mechanicalAngle =
                    NormalizeAngle(pFOC->sensorlessOpenLoopAngle / (float)pFOC->pole_pairs);
            }
        } else {
            /* 非开环模式：即使 PLL 未锁定也用 PLL 角度（至少随真实反电势转动），
             * 避免回退到已冻结的虚拟角度导致 Park 变换完全失效 */
            pFOC->correctedAngle = pFOC->sensorlessElectricalAngle;
            if (pFOC->pole_pairs != 0) {
                pFOC->mechanicalAngle =
                    NormalizeAngle(pFOC->sensorlessElectricalAngle / (float)pFOC->pole_pairs);
            }
        }
    } else {
        FOC_ResetSensorlessIF(pFOC);
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

    // 注释：Ia 采样有异常，当前暂用 Ib/Ic 重构
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

    /* ==== 步骤 8：SMO 滑模观测器 ==== */
    /*
     * 仅无感模式需要更新 SMO。有感模式跳过可以显著减轻 PWM 中断
     * 的计算负担（SMO_Update 包含 atan2f、atanf、fast_sin/cos、
     * 多次浮点乘除和滤波，在 50us 中断周期中占用可观）。
     */
    if (pFOC->sensorMode == FOC_SENSOR_MODE_SENSORLESS) {
        SMO_Update(&g_smoObserver, pFOC->uAlpha, pFOC->uBeta,
                   pFOC->iAlpha, pFOC->iBeta);
    }
}
