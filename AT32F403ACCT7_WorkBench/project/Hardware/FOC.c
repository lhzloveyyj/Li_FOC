#include "at32f403a_407.h"              // Device header
#include "foc.h"
#include "foc_config.h"
#include "math.h"
#include "my_math.h"
#include "fast_sin.h"
#include "delay.h"
#include "stdio.h"
#include "mt6701.h"

#include "freertos_app.h"

/*******************************全局变量***************************/
float g_udc = 24.0f;

float  zero = 0.0f;	
volatile int g_motorAdValues[3]={0};
volatile uint16_t g_ADoffest[3]={0};

int cnt =0;

/*******************************************************************/

/*****************定义电机1的FOC状态结构体*************************/
FocState Motor = {
   // current 可视情况初始化
  .current = {
      .adA = 0,
      .adB = 0,
      .adC = 0,
      .voltageAOffset = 0,
      .voltageBOffset = 0,
      .voltageCOffset = 0,
  },

    .uAlpha = 0.0f, .uBeta = 0.0f, 	
    .iAlpha = 0.0f, .iBeta = 0.0f, 	
    .Ia = 0.0f, .Ib = 0.0f, .Ic = 0.0f,			
    .ua = 0.0f, .ub = 0.0f, .uc = 0.0f, 		
    .uq = 0.0f, .ud = 0.0f, 			
    .iq = 0.0f, .id = 0.0f, 			

    .mechanicalAngle = 0.0f,
    .electricalAngle = 0.0f,
    .correctedAngle = 0.0f,
    .zeroOffset = 0.0f,

    //.idPID = {0},
    //.iqPID = {0},

    .speedLastAngle = 0.0f,
    .speed = 0.0f,
    //.speedPID = {0},

    .setPwmCallback = NULL  // 或者设为具体函数名
};


PFocState g_pMotor = &Motor;

void MotorSetPwm(float ua, float ub, float uc);

/******************************************************************************
  函数说明：获取电压偏置
  @brief  通过多次采样计算电压偏置值，用于后续电流测量补偿
  @param  pFOC 指向FOC状态结构体的指针
  @retval 无
******************************************************************************/
void getAdoffset(void)
{
	int offestA = 0,offestB = 0,offestC = 0;
	
	for(int i=0;i<16;i++)
	{	
		offestA += g_motorAdValues[0];
		offestB += g_motorAdValues[1];
        offestC += g_motorAdValues[2];
	}
	g_pMotor->current.voltageAOffset = offestA >> 4;
	g_pMotor->current.voltageBOffset = offestB >> 4;
    g_pMotor->current.voltageCOffset = offestC >> 4;
	
}



/**
 * @brief     将角度值归一化到 [0, 2π) 区间
 * @param     angle   输入角度（单位：rad）
 * @return    归一化后的角度值，范围为 [0, 2π)
 */
float NormalizeAngle(float angle)
{
    float result = fmodf(angle, FOC_2PI);  // 使用浮点取余
    return (result >= 0.0f) ? result : (result + FOC_2PI);
}


/**
 * @brief     计算电角度：电角度 = 机械角度 × 极对数，并归一化到 [0, 2π)
 * @param     mechAngle   机械角度（单位：rad）
 * @return    电角度（单位：rad），范围 [0, 2π)
 */
float CalculateElectricalAngle(float mechAngle)
{
    float elecAngle = g_pMotor->dir * mechAngle * g_pMotor->pole_pairs;
    return NormalizeAngle(elecAngle);
}

/**
 * @brief     获取修正后的电角度（减去零电角度偏移并归一化到 [0, 2π)）
 * @param     mechAngle    机械角度（单位：rad）
 * @return    电角度（单位：rad），范围 [0, 2π)
 */
float AngleGetCorrectedElec(float mechAngle)
{
    float elecAngle = CalculateElectricalAngle(mechAngle);       // 电角度 = 机械角 × 极对数
    float corrected = elecAngle - g_pMotor->zeroOffset;             // 减去零电位偏移
    corrected = NormalizeAngle(corrected);                  // 归一化到 [0, 2π)

    return corrected;
}

/**
 * @brief     施加恒定 Ud 电压进行强制对准（锁定电角度）
 * @param     ud   d轴电压（单位：V）
 */
void MotorApplyStrongDrag(float ud)
{
    float uAlpha = 0.0f;
    float uBeta = 0.0f;
    float uq = 0.0f;

    // 获取修正后的电角度（theta_e）
    float angleEl = CalculateElectricalAngle(0.0f);

    // Park 逆变换（dq -> αβ）
    uAlpha = -uq * fast_sin(angleEl) + ud * fast_cos(angleEl);
    uBeta  =  uq * fast_cos(angleEl) + ud * fast_sin(angleEl);

    // Clarke 逆变换（αβ -> abc），带中点电压偏移
    float ua = uAlpha + g_udc / 2.0f;
    float ub = (FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;
    float uc = (-FOC_SQRT3 * uBeta - uAlpha) / 2.0f + g_udc / 2.0f;

    MotorSetPwm(ua, ub, uc);
}


/**
 * @brief     角度模块初始化，采集零电角度偏移（调用强拖，进行多次平均）
 * @param     readAngleFunc   用于读取机械角度的函数指针（单位：rad）
 */
void AngleInitZeroOffset(float *zeroOffset , float *correctedElecAngle)
{
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, FALSE);
    MotorApplyStrongDrag(FOC_STRONGDRAG);           // 施加 Ud 强拖，固定转子磁极方向
    vTaskDelay(2000);                       // 保持拖动 2 秒

    // 多次采样以降低抖动影响
    float sum = 0.0f;
    const int sampleCount = 10;
	float mechanicalAngle = 0.0f;

    for (int i = 0; i < sampleCount; i++) {
		mechanicalAngle = Mt6701GetAngleWrapper();
        float elecAngle = CalculateElectricalAngle(mechanicalAngle);
        sum += elecAngle;
        vTaskDelay(10);
    }
	*zeroOffset = sum / sampleCount;  // 计算平均值作为零偏
	mechanicalAngle = Mt6701GetAngleWrapper();
    
    float elecAngle = CalculateElectricalAngle(mechanicalAngle);    //当前电角度
    
    *correctedElecAngle = elecAngle - *zeroOffset; //修正后的电角度
    
    vTaskDelay(500); 
    
	MotorApplyStrongDrag(0.0f);
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
}


void adc_tigger(int time_pwm)
{
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, time_pwm-10);
}


float g_pwmA = 0.0f;
float g_pwmB = 0.0f;
float g_pwmC = 0.0f;

/******************************************************************************
  函数说明：电机 PWM输出设置函数
  @brief  根据输入占空比设置电机1三相PWM输出
  @param  pwm_a 相A占空比
  @param  pwm_b 相B占空比
  @param  pwm_c 相C占空比
  @retval 无
******************************************************************************/
static void setpwm_channel(float pwm_a, float pwm_b, float pwm_c)
{
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, pwm_a * FOC_ALL_DUTY * 0.95f);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, pwm_b * FOC_ALL_DUTY * 0.95f);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, pwm_c * FOC_ALL_DUTY * 0.95f);
}

/**
 * @brief     设置三相 PWM 输出（Ua、Ub、Uc 为 SVPWM 输出电压）
 * @param     ua   A相电压（单位：V）
 * @param     ub   B相电压（单位：V）
 * @param     uc   C相电压（单位：V）
 */
void MotorSetPwm(float ua, float ub, float uc)
{
    // 电压限幅保护（防止超出允许范围）
    ua = LimitValue(ua, 0.0f, g_udc);
    ub = LimitValue(ub, 0.0f, g_udc);
    uc = LimitValue(uc, 0.0f, g_udc);

    // 电压归一化到占空比 [0, 1]
    g_pwmA = LimitValue(ua / g_udc, 0.0f, 1.0f);
    g_pwmB = LimitValue(ub / g_udc, 0.0f, 1.0f);
    g_pwmC = LimitValue(uc / g_udc, 0.0f, 1.0f);

    // 输出到定时器 PWM 寄存器
    setpwm_channel(g_pwmA, g_pwmB, g_pwmC);
}


// Clarke变换（电流）
void clarke_transform(float Ia, float Ib, float *Ialpha, float *Ibeta) {
    *Ialpha = Ia;
    *Ibeta = (1 / FOC_SQRT3) * (Ia + 2 * Ib);  // Clarke变换公式，线性组合
}

// Park变换（电流）
void park_transform(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq) {
    *Id = Ialpha * fast_cos(angle_el) + Ibeta * fast_sin(angle_el);  // Park变换公式
    *Iq = -Ialpha * fast_sin(angle_el) + Ibeta * fast_cos(angle_el);
}

/******************************************************************************
  函数说明：设置SVPWM输出
  @brief  根据SVPWM算法计算结果设置PWM输出
  @param  pFOC 指向FOC状态结构体的指针
  @param  PSVpwm 指向SVPWM状态结构体的指针
  @retval 无
******************************************************************************/
static void setSVpwm(PSVpwm_State PSVpwm)
{
	
    setpwm_channel(PSVpwm->Ta, PSVpwm->Tb, PSVpwm->Tc);
}

/******************************************************************************
  函数说明：逆Park变换
  @brief  将dq坐标系的电压转换为αβ坐标系，以便进行SVPWM计算
  @param  pFOC 指向FOC状态结构体的指针
  @retval 无
******************************************************************************/
static void inv_park_transform(float Uq, float Ud, float corr_angle, float *Out_Ualpha, float *Out_Ubeta)
{
	*Out_Ualpha = -Uq * fast_sin(corr_angle) + Ud * fast_cos(corr_angle);
	*Out_Ubeta  =  Uq * fast_cos(corr_angle) + Ud * fast_sin(corr_angle);
}

/******************************************************************************
  函数说明：逆Clarke变换
  @brief  将αβ坐标系的电压转换为三相电压，适用于PWM输出
  @param  pFOC 指向FOC状态结构体的指针
  @retval 无
******************************************************************************/
static void inv_clarke_transform(float Ualpha, float Ubeta, float *Out_Ua, float *Out_Ub, float *Out_Uc)
{
	*Out_Ua = Ualpha + g_udc/2;
	*Out_Ub = (FOC_SQRT3 * Ubeta - Ualpha)/2 + g_udc/2;
	*Out_Uc = (-FOC_SQRT3 * Ubeta - Ualpha)/2 + g_udc/2;
}

// FOC 控制主函数
void FocContorl(PFocState pFOC,  PSVpwm_State PSVpwm)
{
	//获取机械角度
	pFOC->mechanicalAngle = Mt6701GetAngleWrapper();
	
	//计算电角度
	pFOC->correctedAngle = AngleGetCorrectedElec(pFOC->mechanicalAngle);
	
	pFOC->current.adA = g_motorAdValues[0];
	pFOC->current.adB = g_motorAdValues[1];
    pFOC->current.adC = g_motorAdValues[2];

	//I = adc采样的电压 / 增益 / 电阻
	pFOC->Ia = (pFOC->current.adA - pFOC->current.voltageAOffset)/4096.0f * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
	pFOC->Ib = (pFOC->current.adB - pFOC->current.voltageBOffset)/4096.0f * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
    pFOC->Ic = (pFOC->current.adC - pFOC->current.voltageCOffset)/4096.0f * FOC_ADC_REF_VOLTAGE / FOC_GAIN / FOC_SHUNT_R;
	
    // 因为ia采样有点问题，暂时先用ibic  
	clarke_transform(-pFOC->Ib, -pFOC->Ia, &pFOC->iAlpha, &pFOC->iBeta);
    park_transform(pFOC->iAlpha, pFOC->iBeta, pFOC->correctedAngle, &pFOC->id, &pFOC->iq);
    
	//PID控制器
	//pFOC->Ud = PI_Compute(&pi_Id, 0.0f, pFOC->Id);
	//pFOC->Uq = PI_Compute(&pi_Id, 0.0f, pFOC->Iq);
	
	pFOC->ud = 0.000001f;
	//pFOC->uq = 0.0f;
	
	//逆park变换
	inv_park_transform(pFOC->uq, pFOC->ud, pFOC->correctedAngle, &(pFOC->uAlpha), &(pFOC->uBeta));
	
	//逆clarke变换
	inv_clarke_transform(pFOC->uAlpha, pFOC->uBeta , &(pFOC->ua), &(pFOC->ub), &(pFOC->uc));
	
	//设置PWM
	//MotorSetPwm(pFOC->ua, pFOC->ub, pFOC->uc);
	
	SVpwm(PSVpwm, pFOC->uAlpha, pFOC->uBeta);
	
	//设置SVPWM
	setSVpwm(PSVpwm);
}






