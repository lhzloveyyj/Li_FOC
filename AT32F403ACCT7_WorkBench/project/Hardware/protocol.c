#include "protocol.h"
#include "usart3.h"
#include "freertos_app.h"
#include "led.h"
#include "flash_ops.h"
#include "FOC.h"
#include "foc_config.h"
#include "mostemp.h"
#include "smo_observer.h"

/* =================== 遥测使能标志 =================== */
volatile uint8_t anglePrintingEnabled = 0;      // 机械角度打印（TMR2 中断中触发）
volatile uint8_t uabcEnabled = 0;               // 三相电压 Ua/Ub/Uc
volatile uint8_t adcEnabled  = 0;               // ADC 原始值
volatile uint8_t tabcEnabled = 0;               // SVPWM 三相占空比 Ta/Tb/Tc
volatile uint8_t IabcEnabled = 0;               // 三相电流 Ia/Ib/Ic
volatile uint8_t UAlpha_BetaEnabled = 0;        // Uα/Uβ
volatile uint8_t IAlpha_BetaEnabled = 0;        // Iα/Iβ
volatile uint8_t IQ_ID_Enabled = 0;             // Id/Iq
volatile uint8_t mostemp_Enabled = 0;           // MOS 温度
volatile uint8_t speed_Enabled = 0;             // 实际速度
volatile uint8_t speedOut_Enabled = 0;          // 速度环 PID 输出
volatile uint8_t local_Enabled = 0;             // 实际位置
volatile uint8_t localOut_Enabled = 0;          // 位置环 PID 输出
volatile uint8_t adcvbus_Enabled = 0;           // 母线电压
volatile uint8_t smoAngle_Enabled = 0;          // PLL 角度：SMO 反电势经 PLL 锁相后的最终角度
volatile uint8_t smoSpeed_Enabled = 0;          // SMO 估计速度
volatile uint8_t smoBackEmf_Enabled = 0;        // SMO 反电势 eAlpha/eBeta
volatile uint8_t electricalAngle_Enabled = 0;    // 编码器实际电角度
volatile uint8_t smoRawAngle_Enabled = 0;       // SMO 角度：反电势 atan2 直接角度，不经过 PLL
volatile uint8_t smoDiag_Enabled = 0;           // SMO 诊断量：pllError/eMag，判断 PLL 跟踪和反电势幅值

static float g_zeroOffset = 0.0f;               // 零电角度偏移（标定结果）
static float g_correctedElecAngle = 0.0f;       // 当前修正后的电角度

/******************************************************************************
 * 函数名称：Comm_CommandHandler
 * 功能描述：通信命令处理函数。
 *           按 g_commCmd 值切换执行对应操作，支持：
 *           - 参数设置（PID 参数、控制模式、电机参数）
 *           - 遥测使能/禁能
 *           - 零点标定
 *           - SMO 调试数据使能
 *
 * 注意：上位机命令通过 USART3 逐字节解析写入 g_commCmd 和 g_cmdData，
 *       此函数在 comm_task 中轮询处理。
 ******************************************************************************/
void Comm_CommandHandler(void)
{
    led_device_t *ledRun = freertos_get_run_led();
    float data[17] = {0.0f};

    /* 收到命令时 LED 闪烁指示 */
    if ((g_commCmd != CMD_NONE) && (0 == led_get(ledRun))) {
        led_set(ledRun, 1);
        vTaskDelay(50);
        led_set(ledRun, 0);
    }

    switch (g_commCmd)
    {
        /* ---- 连接电机：加载参数并返回完整状态 ---- */
        case CMD_CONNECT_MOTOR:
            foc_params_load(&g_readback);
            g_pMotor->pole_pairs = g_readback.pole_pairs;
            g_pMotor->dir        = g_readback.dir;
            g_pMotor->zeroOffset = g_readback.elec_offset;
            g_pMotor->speedDir   = g_readback.speeddir;
            g_pMotor->rs         = g_readback.rs;
            g_pMotor->lq         = g_readback.lq;
            g_pMotor->ld         = g_readback.ld;
            data[0]   = (float)g_pMotor->pole_pairs;
            data[1]   = (float)g_pMotor->dir;
            data[2]   = g_pMotor->zeroOffset;
            data[3]   = g_pMotor->iqPID.kp;
            data[4]   = g_pMotor->iqPID.ki;
            data[5]   = getVbus();
            data[6]   = g_pMotor->speedDir;
            data[7]   = g_pMotor->speedPID.kp;
            data[8]   = g_pMotor->speedPID.ki;
            data[9]   = g_pMotor->positionPID.kp;
            data[10]  = g_pMotor->positionPID.kd;
            data[11]  = g_pMotor->iqPID.outMax;
            data[12]  = g_pMotor->speedPID.outMax;
            data[13]  = g_pMotor->positionPID.outMax;
            data[14]  = g_pMotor->rs;
            data[15]  = g_pMotor->lq;
            data[16]  = g_pMotor->ld;
            USART3_SendPacket(CMD_CONNECT_MOTOR, &data[0], 17);
            mostemp_Enabled = 1;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 机械角度遥测 ---- */
        case CMD_MECHANICALANGLE:
            anglePrintingEnabled = 1;
            g_commCmd = CMD_NONE;
            break;
        case CMD_MECHANICALANGLE_CLOSE:
            anglePrintingEnabled = 0;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置极对数 ---- */
        case CMD_SETPAIRS:
            foc_params_load(&g_params);
            g_params.pole_pairs = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置方向 ---- */
        case CMD_SETDIR:
            foc_params_load(&g_params);
            g_params.dir = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE;
            break;

        /* ---- 零点校准（强拖 + 标定） ---- */
        case CMD_ZEROCALIBRATIO:
            led_set(ledRun, 1);
            AngleInitZeroOffset(&g_zeroOffset, &g_correctedElecAngle);
            data[0] = g_zeroOffset;
            data[1] = g_correctedElecAngle;
            foc_params_load(&g_params);
            g_params.elec_offset = g_zeroOffset;
            foc_params_save(&g_params);
            USART3_SendPacket(CMD_ZEROCALIBRATIO_OVER, &data[0], 2);
            led_set(ledRun, 0);
            g_commCmd = CMD_NONE;
            break;

        /* ---- 三相电压遥测 ---- */
        case CMD_UABC:       uabcEnabled = 1;       g_commCmd = CMD_NONE; break;
        case CMD_UABC_CLOSE: uabcEnabled = 0;       g_commCmd = CMD_NONE; break;

        /* ---- 设置 Uq ---- */
        case CMD_SETUQ:
            g_pMotor->uq = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- ADC 原始值遥测 ---- */
        case CMD_ADC:        adcEnabled = 1;        g_commCmd = CMD_NONE; break;
        case CMD_ADC_CLOSE:  adcEnabled = 0;        g_commCmd = CMD_NONE; break;

        /* ---- 母线电压 ADC 遥测 ---- */
        case CMD_ADCVBUS:         adcvbus_Enabled = 1;  g_commCmd = CMD_NONE; break;
        case CMD_ADCVBUS_CLOSE:   adcvbus_Enabled = 0;  g_commCmd = CMD_NONE; break;

        /* ---- SMO 遥测 ---- */
        case CMD_SMO_ANGLE:       smoAngle_Enabled = 1;     g_commCmd = CMD_NONE; break;
        case CMD_SMO_ANGLE_CLOSE: smoAngle_Enabled = 0;     g_commCmd = CMD_NONE; break;
        case CMD_SMO_SPEED:       smoSpeed_Enabled = 1;     g_commCmd = CMD_NONE; break;
        case CMD_SMO_SPEED_CLOSE: smoSpeed_Enabled = 0;     g_commCmd = CMD_NONE; break;
        case CMD_SMO_BACKEMF:     smoBackEmf_Enabled = 1;   g_commCmd = CMD_NONE; break;
        case CMD_SMO_BACKEMF_CLOSE: smoBackEmf_Enabled = 0; g_commCmd = CMD_NONE; break;
        /* SMO 角度是不经过 PLL 的 atan2 角度，用来判断反电势估计本体是否正确 */
        case CMD_SMO_RAW_ANGLE:       smoRawAngle_Enabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_SMO_RAW_ANGLE_CLOSE: smoRawAngle_Enabled = 0; g_commCmd = CMD_NONE; break;

        /* 诊断量包含 PLL 归一化误差和反电势幅值，用来判断 PLL 是否跟丢或反电势是否过弱 */
        case CMD_SMO_DIAG:       smoDiag_Enabled = 1;     g_commCmd = CMD_NONE; break;
        case CMD_SMO_DIAG_CLOSE: smoDiag_Enabled = 0;     g_commCmd = CMD_NONE; break;

        /* 有感运行稳定后可手动复位，让 PLL 从当前有效反电势重新初始化 */
        case CMD_SMO_RESET:
            SMO_Reset(&g_smoObserver);
            g_commCmd = CMD_NONE;
            break;

        /* ---- FOC 反馈来源切换 ---- */
        case CMD_SENSOR_SENSORED:
            FOC_SetSensorMode(g_pMotor, FOC_SENSOR_MODE_SENSORED);
            g_commCmd = CMD_NONE;
            break;
        case CMD_SENSOR_SENSORLESS:
            FOC_SetSensorMode(g_pMotor, FOC_SENSOR_MODE_SENSORLESS);
            g_commCmd = CMD_NONE;
            break;

        /* ---- 电角度遥测 ---- */
        case CMD_ELECTRICALANGLE:       electricalAngle_Enabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_ELECTRICALANGLE_CLOSE: electricalAngle_Enabled = 0; g_commCmd = CMD_NONE; break;

        /* ---- 母线电压读取（单次应答） ---- */
        case CMD_DCVBUS:
            data[0] = getVbus();
            USART3_SendPacket(CMD_DCVBUS, &data[0], 1);
            g_commCmd = CMD_NONE;
            break;

        /* ---- SVPWM 占空比遥测 ---- */
        case CMD_TABC:       tabcEnabled = 1;       g_commCmd = CMD_NONE; break;
        case CMD_TABC_CLOSE: tabcEnabled = 0;       g_commCmd = CMD_NONE; break;

        /* ---- 三相电流遥测 ---- */
        case CMD_IABC:       IabcEnabled = 1;       g_commCmd = CMD_NONE; break;
        case CMD_IABC_CLOSE: IabcEnabled = 0;       g_commCmd = CMD_NONE; break;

        /* ---- Uα/Uβ 遥测 ---- */
        case CMD_UALPHA_BETA:       UAlpha_BetaEnabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_UALPHA_BETA_CLOSE: UAlpha_BetaEnabled = 0; g_commCmd = CMD_NONE; break;

        /* ---- Iα/Iβ 遥测 ---- */
        case CMD_IALPHA_BETA:       IAlpha_BetaEnabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_IALPHA_BETA_CLOSE: IAlpha_BetaEnabled = 0; g_commCmd = CMD_NONE; break;

        /* ---- Id/Iq 遥测 ---- */
        case CMD_IQ_ID:       IQ_ID_Enabled = 1;    g_commCmd = CMD_NONE; break;
        case CMD_IQ_ID_CLOSE: IQ_ID_Enabled = 0;    g_commCmd = CMD_NONE; break;

        /* ---- 设置目标 Id/Iq ---- */
        case CMD_SETIQ:
            g_pMotor->tariq = g_cmdData;
            if ((g_pMotor->ctrolmode == FOC_SPEED_LOOP
                 || g_pMotor->ctrolmode == FOC_POSITION_LOOP)
                && (g_pMotor->tariq > FOC_EPSILON)) {
                if (g_pMotor->speedPID.out > g_pMotor->tariq) {
                    g_pMotor->speedPID.out = g_pMotor->tariq;
                }
                if (g_pMotor->speedPID.out < -g_pMotor->tariq) {
                    g_pMotor->speedPID.out = -g_pMotor->tariq;
                }
            }
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETID:
            g_pMotor->tarid = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 控制模式切换 ---- */
        case CMD_OPEN_LOOP:
            g_pMotor->ctrolmode = FOC_OPEN_LOOP;
            g_pMotor->iqPID.lastBias = 0.0f;
            g_pMotor->speedPID.out = 0.0f;
            g_pMotor->speedPID.lastBias = 0.0f;
            g_commCmd = CMD_NONE;
            break;
        case CMD_CURRENT_LOOP:
            g_pMotor->ctrolmode = FPC_CURRENT_LOOP;
            g_pMotor->iqPID.out = g_pMotor->uq;
            g_pMotor->iqPID.lastBias = g_pMotor->iqPID.tar - g_pMotor->iqPID.pre;
            g_pMotor->speedPID.out = 0.0f;
            g_pMotor->speedPID.lastBias = 0.0f;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SPEED_LOOP:
            g_pMotor->ctrolmode = FOC_SPEED_LOOP;
            if (FOC_GetSensorMode(g_pMotor) == FOC_SENSOR_MODE_SENSORLESS) {
                g_pMotor->sensorlessIfState = FOC_SENSORLESS_IF_OFF;
                g_pMotor->sensorlessIfAlignCount = 0U;
                g_pMotor->sensorlessIfLockCount = 0U;
                g_pMotor->sensorlessIfSpeed = 0.0f;
                g_pMotor->sensorlessIfIq = 0.0f;
                g_pMotor->sensorlessIfId = 0.0f;
                g_pMotor->speedPID.out = 0.0f;
                g_pMotor->speedPID.lastBias = 0.0f;
                g_pMotor->iqPID.out = 0.0f;
            } else {
                g_pMotor->speedPID.out = g_pMotor->iq;
                g_pMotor->speedPID.lastBias = g_pMotor->tar_speed - g_pMotor->speed;
                g_pMotor->iqPID.out = g_pMotor->uq;
            }
            g_pMotor->iqPID.lastBias = 0.0f;
            g_commCmd = CMD_NONE;
            break;
        case CMD_POSITION_LOOP:
            FOC_SetSensorMode(g_pMotor, FOC_SENSOR_MODE_SENSORED);
            g_pMotor->ctrolmode = FOC_POSITION_LOOP;
            g_pMotor->speedPID.out = g_pMotor->iq;
            g_pMotor->speedPID.lastBias = g_pMotor->tar_speed - g_pMotor->speed;
            g_pMotor->iqPID.out = g_pMotor->uq;
            g_pMotor->iqPID.lastBias = 0.0f;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置 Ud ---- */
        case CMD_SETUD:
            g_pMotor->ud = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置电流环 PID 参数 ---- */
        case CMD_SETIQPIDKP:
            g_pMotor->iqPID.kp = g_cmdData;
            g_pMotor->idPID.kp = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETIQPIDKI:
            g_pMotor->iqPID.ki = g_cmdData;
            g_pMotor->idPID.ki = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 速度遥测 ---- */
        case CMD_SPEED:       speed_Enabled = 1;    g_commCmd = CMD_NONE; break;
        case CMD_SPEED_CLODE: speed_Enabled = 0;    g_commCmd = CMD_NONE; break;

        /* ---- 设置速度方向 ---- */
        case CMD_SETSPEEDDIR:
            foc_params_load(&g_params);
            g_params.speeddir = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE;
            break;

        /* ---- 速度 PID 输出遥测 ---- */
        case CMD_SPEEDOUT:       speedOut_Enabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_SPEEDOUT_CLOSE: speedOut_Enabled = 0; g_commCmd = CMD_NONE; break;

        /* ---- 设置速度目标 ---- */
        case CMD_SETSPEEDTAR:
            g_pMotor->tar_speed = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置速度环 PID 参数 ---- */
        case CMD_SETSPEEDPIDKP:
            g_pMotor->speedPID.kp = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETSPEEDPIDKI:
            g_pMotor->speedPID.ki = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 位置相关 ---- */
        case CMD_SETLOCALTAR:
            g_pMotor->tarPosition = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_LOCAL:       local_Enabled = 1;     g_commCmd = CMD_NONE; break;
        case CMD_LOCAL_CLOSE: local_Enabled = 0;     g_commCmd = CMD_NONE; break;
        case CMD_LOCALOUT:       localOut_Enabled = 1; g_commCmd = CMD_NONE; break;
        case CMD_LOCALOUT_CLOSE: localOut_Enabled = 0; g_commCmd = CMD_NONE; break;

        /* ---- 设置位置环 PID 参数 ---- */
        case CMD_SETLOCALPIDKP:
            g_pMotor->positionPID.kp = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETLOCALPIDKD:
            g_pMotor->positionPID.kd = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置各环输出限幅 ---- */
        case CMD_SETIQPIDOUT:
            g_pMotor->iqPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETSPEEDPIDOUT:
            g_pMotor->speedPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE;
            break;
        case CMD_SETLOCALPIDOUT:
            g_pMotor->positionPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置电机模型参数（Rs/Lq/Ld） ---- */
        case CMD_SETMOTORRS:
            /* Rs/Lq/Ld 是 Flash 参数，也是 SMO 运行参数。
             * 收到上位机命令后同时更新两处：掉电保存 + 当前运行立即生效。
             */
            foc_params_load(&g_params);
            g_params.rs = g_cmdData;
            foc_params_save(&g_params);
            g_pMotor->rs = g_cmdData;
            g_smoObserver.cfg.rs = g_pMotor->rs;
            g_commCmd = CMD_NONE;
            break;

        case CMD_SETMOTORLQ:
            /* SMO 使用 Lq/Ld 的平均值作为 αβ 等效电感 Ls */
            foc_params_load(&g_params);
            g_params.lq = g_cmdData;
            foc_params_save(&g_params);
            g_pMotor->lq = g_cmdData;
            g_smoObserver.cfg.ls = (g_pMotor->lq + g_pMotor->ld) * 0.5f;
            g_commCmd = CMD_NONE;
            break;

        case CMD_SETMOTORLD:
            foc_params_load(&g_params);
            g_params.ld = g_cmdData;
            foc_params_save(&g_params);
            g_pMotor->ld = g_cmdData;
            g_smoObserver.cfg.ls = (g_pMotor->lq + g_pMotor->ld) * 0.5f;
            g_commCmd = CMD_NONE;
            break;

        /* ---- 设置 Iq 参考电流上限 ---- */
        case CMD_SETIQMAX:
            g_pMotor->tariqMax = g_cmdData;
            g_commCmd = CMD_NONE;
            break;

        default:
            /* 未识别命令也要清掉，避免上位机发了新命令但旧固件不支持时 LED 一直闪 */
            g_commCmd = CMD_NONE;
            break;
    }
}
