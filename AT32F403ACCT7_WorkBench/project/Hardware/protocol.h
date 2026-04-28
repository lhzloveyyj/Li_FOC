#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

/**
 * @brief 通信协议定义
 *
 * 上位机（QT）通过 USART3 与本固件通信。
 * 帧格式见 usart3.c 中的详细注释。
 */

/* ======================= 帧结构定义 ======================= */
#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

/* ======================= 命令字枚举 ======================= */
typedef enum
{
    CMD_NONE                  = 0x00,  // 无效命令
    CMD_CONNECT_MOTOR         = 0x01,  // 连接电机（返回所有参数）
    CMD_MECHANICALANGLE       = 0x02,  // 打开机械角度传输
    CMD_MECHANICALANGLE_CLOSE = 0X03,  // 关闭机械角度传输
    CMD_SETPAIRS              = 0x04,  // 设置极对数
    CMD_SETDIR                = 0x05,  // 设置方向
    CMD_ZEROCALIBRATIO        = 0x06,  // 零点校准（强拖+标定）
    CMD_ZEROCALIBRATIO_OVER   = 0x07,  // 零点校准结束（自动返回结果）
    CMD_UABC                  = 0x08,  // 三相电压 Ua/Ub/Uc
    CMD_UABC_CLOSE            = 0x09,  // 关闭三相电压
    CMD_SETUQ                 = 0x0A,  // 设置 Uq
    CMD_ADC                   = 0x0B,  // 三相 ADC 原始值
    CMD_ADC_CLOSE             = 0x0C,  // 关闭三相 ADC
    CMD_DCVBUS                = 0x0D,  // 母线电压
    CMD_TABC                  = 0x0E,  // SVPWM 三相占空比 Ta/Tb/Tc
    CMD_TABC_CLOSE            = 0x0F,  // 关闭三相占空比
    CMD_IABC                  = 0x10,  // 三相电流 Ia/Ib/Ic
    CMD_IABC_CLOSE            = 0x11,  // 关闭三相电流
    CMD_UALPHA_BETA           = 0x12,  // Uα/Uβ
    CMD_UALPHA_BETA_CLOSE     = 0x13,  // 关闭 Uα/Uβ
    CMD_IALPHA_BETA           = 0x14,  // Iα/Iβ
    CMD_IALPHA_BETA_CLOSE     = 0x15,  // 关闭 Iα/Iβ
    CMD_IQ_ID                 = 0x16,  // Iq/Id
    CMD_IQ_ID_CLOSE           = 0x17,  // 关闭 Iq/Id
    CMD_SETIQ                 = 0x18,  // 设置目标 Iq
    CMD_SETID                 = 0x19,  // 设置目标 Id
    CMD_OPEN_LOOP             = 0x20,  // 开环模式
    CMD_CURRENT_LOOP          = 0x21,  // 电流环模式
    CMD_SPEED_LOOP            = 0x22,  // 速度-电流环模式
    CMD_POSITION_LOOP         = 0x23,  // 位置-速度-电流环模式
    CMD_MOSTEMP               = 0x24,  // MOS 温度
    CMD_SETUD                 = 0x25,  // 设置 Ud
    CMD_SETIQPIDKP            = 0x26,  // 设置电流环 Kp
    CMD_SETIQPIDKI            = 0x27,  // 设置电流环 Ki
    CMD_SPEED                 = 0x28,  // 开启速度打印
    CMD_SPEED_CLODE           = 0x29,  // 关闭速度打印
    CMD_SETSPEEDDIR           = 0x30,  // 设置速度方向
    CMD_SPEEDOUT              = 0x31,  // 速度环 PID 输出
    CMD_SPEEDOUT_CLOSE        = 0x32,  // 关闭速度环 PID 输出
    CMD_SETSPEEDTAR           = 0x33,  // 设置速度目标
    CMD_SETSPEEDPIDKP         = 0x34,  // 设置速度环 Kp
    CMD_SETSPEEDPIDKI         = 0x35,  // 设置速度环 Ki
    CMD_SETLOCALTAR           = 0x36,  // 设置位置目标
    CMD_LOCAL                 = 0x37,  // 开启位置打印
    CMD_LOCAL_CLOSE           = 0x38,  // 关闭位置打印
    CMD_LOCALOUT              = 0x39,  // 位置环 PID 输出
    CMD_LOCALOUT_CLOSE        = 0x40,  // 关闭位置环 PID 输出
    CMD_SETLOCALPIDKP         = 0x41,  // 设置位置环 Kp
    CMD_SETLOCALPIDKD         = 0x42,  // 设置位置环 Kd
    CMD_SETIQPIDOUT           = 0X43,  // 设置电流环输出限幅
    CMD_SETSPEEDPIDOUT        = 0X44,  // 设置速度环输出限幅
    CMD_SETLOCALPIDOUT        = 0X45,  // 设置位置环输出限幅
    CMD_ADCVBUS               = 0X46,  // 母线电压 ADC 原始值
    CMD_ADCVBUS_CLOSE         = 0X47,  // 关闭母线电压 ADC
    CMD_SMO_ANGLE             = 0X48,  // SMO 估算电角度
    CMD_SMO_ANGLE_CLOSE       = 0X49,  // 关闭 SMO 电角度
    CMD_SMO_SPEED             = 0X4A,  // SMO 估算电速度
    CMD_SMO_SPEED_CLOSE       = 0X4B,  // 关闭 SMO 电速度
    CMD_SMO_BACKEMF           = 0X4C,  // SMO 反电势 eAlpha/eBeta
    CMD_SMO_BACKEMF_CLOSE     = 0X4D,  // 关闭 SMO 反电势
    CMD_SETMOTORRS            = 0X4E,  // 设置电机相电阻 Rs
    CMD_SETMOTORLQ            = 0X4F,  // 设置电机 q 轴电感 Lq
    CMD_SETMOTORLD            = 0X50,  // 设置电机 d 轴电感 Ld
    CMD_ELECTRICALANGLE       = 0X51,  // 编码器实际电角度
    CMD_ELECTRICALANGLE_CLOSE = 0X52,  // 关闭实际电角度
} CMD_TypeDef;

/* =================== 遥测使能标志（外部使用） =================== */
extern uint16_t adcvbus;
extern volatile uint8_t anglePrintingEnabled;
extern volatile uint8_t uabcEnabled;
extern volatile uint8_t adcEnabled;
extern volatile uint8_t tabcEnabled;
extern volatile uint8_t IabcEnabled;
extern volatile uint8_t UAlpha_BetaEnabled;
extern volatile uint8_t IAlpha_BetaEnabled;
extern volatile uint8_t IQ_ID_Enabled;
extern volatile uint8_t mostemp_Enabled;
extern volatile uint8_t speed_Enabled;
extern volatile uint8_t speedOut_Enabled;
extern volatile uint8_t local_Enabled;
extern volatile uint8_t localOut_Enabled;
extern volatile uint8_t adcvbus_Enabled;
extern volatile uint8_t smoAngle_Enabled;
extern volatile uint8_t smoSpeed_Enabled;
extern volatile uint8_t smoBackEmf_Enabled;
extern volatile uint8_t electricalAngle_Enabled;

/* =================== 函数声明 =================== */
void Comm_CommandHandler(void);

#endif
