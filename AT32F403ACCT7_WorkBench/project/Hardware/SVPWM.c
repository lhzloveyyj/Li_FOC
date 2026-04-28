#include "SVPWM.h"
#include "FOC.h"
#include "foc_config.h"

/**
 * SVPWM 全局状态实例
 *
 * SVPWM（Space Vector Pulse Width Modulation，空间矢量脉宽调制）
 * 通过八个基本电压矢量（六个非零矢量 + 两个零矢量）合成任意方向
 * 的参考电压矢量，使得电机磁场接近圆形旋转磁场。
 *
 * 扇区划分与矢量合成：
 *   扇区  u1>0  u2>0  u3>0  基本矢量
 *    1     +     +     -     V4(100), V6(110)
 *    2     -     +     -     V2(010), V6(110)
 *    3     +     +     +     V2(010), V3(011)
 *    4     -     -     +     V1(001), V3(011)
 *    5     +     -     +     V1(001), V5(101)
 *    6     -     -     -     V4(100), V5(101)
 *
 * 其中 u1 = Ubeta, u2 = sin60*Ualpha - 0.5*Ubeta, u3 = -u1 - u2
 * 扇区编号 formula = (u1>0) + (u2>0)*2 + (u3>0)*4
 */
SVpwm_State TpSVpwm = {
    .Ts = 1.0f,
    .sector = 0,
    .Ta = 0.0f, .Tb = 0.0f, .Tc = 0.0f,
    .u1 = 0.0f, .u2 = 0.0f, .u3 = 0.0f,
    .t1 = 0.0f, .t2 = 0.0f, .t3 = 0.0f,
    .t4 = 0.0f, .t5 = 0.0f, .t6 = 0.0f, .t7 = 0.0f,
    .times = 0,
};

PSVpwm_State PSVpwm = &TpSVpwm;

/******************************************************************************
 * 函数名称：SVpwm
 * 功能描述：SVPWM 调制计算主函数。
 *
 * 计算流程：
 *   1. 根据 Uα/Uβ 计算 Clark 变换分量 u1/u2/u3
 *   2. 判断参考矢量所在扇区
 *   3. 计算相邻基本矢量的作用时间
 *   4. 检测过调制并缩放
 *   5. 计算七段式 SVPWM 的三相导通时间 Ta/Tb/Tc
 *
 * 输入参数：PSVpwm   - SVPWM 状态结构体指针
 *           U_alpha  - α 轴电压
 *           U_beta   - β 轴电压
 ******************************************************************************/
void SVpwm(PSVpwm_State PSVpwm, float U_alpha, float U_beta)
{
    PSVpwm->K = FOC_SQRT3 * PSVpwm->Ts / g_udc;

    /* 计算 Clark 变换分量 (Clarke-transformed components) */
    PSVpwm->u1 = U_beta * PSVpwm->K;
    PSVpwm->u2 = (FOC_SQRT3_DIV_2 * U_alpha - FOC_1_2 * U_beta) * PSVpwm->K;
    PSVpwm->u3 = (-FOC_SQRT3_DIV_2 * U_alpha - FOC_1_2 * U_beta) * PSVpwm->K;

    /* 扇区判定 */
    PSVpwm->sector = (PSVpwm->u1 > 0.0f)
                     + ((PSVpwm->u2 > 0.0f) << 1)
                     + ((PSVpwm->u3 > 0.0f) << 2);

    /* 根据扇区计算矢量作用时间和三相占空比
     * 使用七段式 SVPWM：零矢量 V0 和 V7 对称分布，降低谐波。
     */
    switch (PSVpwm->sector)
    {
        case 3: /* 扇区 1：V4(100) + V6(110) */
            PSVpwm->t4 = PSVpwm->u2;
            PSVpwm->t6 = PSVpwm->u1;
            PSVpwm->sum = PSVpwm->t4 + PSVpwm->t6;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t4 *= PSVpwm->k_svpwm;
                PSVpwm->t6 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t4 - PSVpwm->t6) / 2.0f;
            PSVpwm->Ta = PSVpwm->t4 + PSVpwm->t6 + PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t6 + PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t7;
            break;

        case 1: /* 扇区 2：V2(010) + V6(110) */
            PSVpwm->t2 = -PSVpwm->u2;
            PSVpwm->t6 = -PSVpwm->u3;
            PSVpwm->sum = PSVpwm->t2 + PSVpwm->t6;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t2 *= PSVpwm->k_svpwm;
                PSVpwm->t6 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t2 - PSVpwm->t6) / 2.0f;
            PSVpwm->Ta = PSVpwm->t6 + PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t2 + PSVpwm->t6 + PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t7;
            break;

        case 5: /* 扇区 3：V2(010) + V3(011) */
            PSVpwm->t2 = PSVpwm->u1;
            PSVpwm->t3 = PSVpwm->u3;
            PSVpwm->sum = PSVpwm->t2 + PSVpwm->t3;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t2 *= PSVpwm->k_svpwm;
                PSVpwm->t3 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t2 - PSVpwm->t3) / 2.0f;
            PSVpwm->Ta = PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t2 + PSVpwm->t3 + PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t3 + PSVpwm->t7;
            break;

        case 4: /* 扇区 4：V1(001) + V3(011) */
            PSVpwm->t1 = -PSVpwm->u1;
            PSVpwm->t3 = -PSVpwm->u2;
            PSVpwm->sum = PSVpwm->t1 + PSVpwm->t3;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t1 *= PSVpwm->k_svpwm;
                PSVpwm->t3 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t1 - PSVpwm->t3) / 2.0f;
            PSVpwm->Ta = PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t3 + PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t1 + PSVpwm->t3 + PSVpwm->t7;
            break;

        case 6: /* 扇区 5：V1(001) + V5(101) */
            PSVpwm->t1 = PSVpwm->u3;
            PSVpwm->t5 = PSVpwm->u2;
            PSVpwm->sum = PSVpwm->t1 + PSVpwm->t5;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t1 *= PSVpwm->k_svpwm;
                PSVpwm->t5 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t1 - PSVpwm->t5) / 2.0f;
            PSVpwm->Ta = PSVpwm->t5 + PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t1 + PSVpwm->t5 + PSVpwm->t7;
            break;

        case 2: /* 扇区 6：V4(100) + V5(101) */
            PSVpwm->t4 = -PSVpwm->u3;
            PSVpwm->t5 = -PSVpwm->u1;
            PSVpwm->sum = PSVpwm->t4 + PSVpwm->t5;
            if (PSVpwm->sum > PSVpwm->Ts) {
                PSVpwm->k_svpwm = PSVpwm->Ts / PSVpwm->sum;
                PSVpwm->t4 *= PSVpwm->k_svpwm;
                PSVpwm->t5 *= PSVpwm->k_svpwm;
            }
            PSVpwm->t7 = (PSVpwm->Ts - PSVpwm->t4 - PSVpwm->t5) / 2.0f;
            PSVpwm->Ta = PSVpwm->t4 + PSVpwm->t5 + PSVpwm->t7;
            PSVpwm->Tb = PSVpwm->t7;
            PSVpwm->Tc = PSVpwm->t5 + PSVpwm->t7;
            break;

        default:
            break;
    }
}
