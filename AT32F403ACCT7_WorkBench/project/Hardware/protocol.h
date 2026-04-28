#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

// ======================= 帧结构定义 ======================= //
#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

// ======================= 命令字定义 ======================= //
typedef enum
{
    CMD_NONE                    = 0x00,  // 无效命令
    CMD_CONNECT_MOTOR           = 0x01,  // 连接电机
    CMD_MECHANICALANGLE         = 0x02,  // 打开机械角度传输
    CMD_MECHANICALANGLE_CLOSE   = 0X03,  // 关闭机械角度传输
    CMD_SETPAIRS                = 0x04,  // 设置极对数
    CMD_SETDIR                  = 0x05,  // 设置方向
    CMD_ZEROCALIBRATIO          = 0x06,  // 零点校准
    CMD_ZEROCALIBRATIO_OVER     = 0x07,  // 零点校准结束
    CMD_UABC                    = 0x08,  // 打印三项电压ua,ub,uc
    CMD_UABC_CLOSE              = 0x09,  // 关闭打印三项电压ua,ub,uc
    CMD_SETUQ                   = 0x0A,  // 设置Uq
    CMD_ADC                     = 0x0B,  // 打印三项ADC采样值
    CMD_ADC_CLOSE               = 0x0C,  // 关闭打印三项ADC采样值
    CMD_DCVBUS                  = 0x0D,  // 获取母线电压
    CMD_TABC                    = 0x0E,  // 三相SVPWM输出
    CMD_TABC_CLOSE              = 0x0F,  // 关闭三相SVPWM输出
    CMD_IABC                    = 0x10,  // 打印三相电流
    CMD_IABC_CLOSE              = 0x11,  // 关闭打印三相电流
    CMD_UALPHA_BETA             = 0x12,  // 打印 uAlpha, uBeta
    CMD_UALPHA_BETA_CLOSE       = 0x13,  // 关闭打印 uAlpha, uBeta
    CMD_IALPHA_BETA             = 0x14,  // 打印 IAlpha, uBeta
    CMD_IALPHA_BETA_CLOSE       = 0x15,  // 关闭打印 IAlpha, uBeta
    CMD_IQ_ID                   = 0x16,  // 打印 IQ,ID
    CMD_IQ_ID_CLOSE             = 0x17,  // 关闭打印 IQ,ID
    CMD_SETIQ                   = 0x18,  // 设置IQ
    CMD_SETID                   = 0x19,  // 设置ID
    CMD_OPEN_LOOP               = 0x20,  // 开环模式
    CMD_CURRENT_LOOP            = 0x21,  // 电流环
    CMD_SPEED_LOOP              = 0x22,  // 速度-电流环
    CMD_POSITION_LOOP           = 0x23,  // 位置-速度-电流环
    CMD_MOSTEMP                 = 0x24,  // MOS温度
    CMD_SETUD                   = 0x25,  ///< 设置Ud
    CMD_SETIQPIDKP              = 0x26,  ///< 设置电流环KP
    CMD_SETIQPIDKI              = 0x27,  ///< 设置电流环KI
    CMD_SPEED                   = 0x28,  ///< 开启速度打印
    CMD_SPEED_CLODE             = 0x29,  ///< 关闭速度打印
    CMD_SETSPEEDDIR             = 0x30,  ///< 设置速度方向
    CMD_SPEEDOUT                = 0x31,  ///< 打印速度
    CMD_SPEEDOUT_CLOSE          = 0x32,  ///< 停止打印速度
    CMD_SETSPEEDTAR             = 0x33,  ///< 设置速度期望
    CMD_SETSPEEDPIDKP           = 0x34,  ///< 设置速度环KP
    CMD_SETSPEEDPIDKI           = 0x35,  ///< 设置速度环KI
    CMD_SETLOCALTAR             = 0x36,  ///< 设置位置期望
    CMD_LOCAL                   = 0x37,  ///< 开启位置打印
    CMD_LOCAL_CLOSE             = 0x38,  ///< 关闭位置打印
    CMD_LOCALOUT                = 0x39,  ///< 打印位置
    CMD_LOCALOUT_CLOSE          = 0x40,  ///< 停止打印位置
    CMD_SETLOCALPIDKP           = 0x41,  ///< 设置速位置环KP
    CMD_SETLOCALPIDKD           = 0x42,  ///< 设置位置环KD
    CMD_SETIQPIDOUT             = 0X43,  // 设置电流环输出限制
    CMD_SETSPEEDPIDOUT          = 0X44,  // 设置速度环输出限制
    CMD_SETLOCALPIDOUT          = 0X45,  // 设置位置环输出限制
    CMD_ADCVBUS                 = 0X46,  // 母线电压ADC原始值
    CMD_ADCVBUS_CLOSE           = 0X47,  // 关闭母线电压ADC原始值
    CMD_SMO_ANGLE               = 0X48,  // 打印SMO估算电角度
    CMD_SMO_ANGLE_CLOSE         = 0X49,  // 关闭SMO估算电角度
    CMD_SMO_SPEED               = 0X4A,  // 打印SMO估算电速度
    CMD_SMO_SPEED_CLOSE         = 0X4B,  // 关闭SMO估算电速度
    CMD_SMO_BACKEMF             = 0X4C,  // 打印SMO反电势eAlpha/eBeta
    CMD_SMO_BACKEMF_CLOSE       = 0X4D,  // 关闭SMO反电势eAlpha/eBeta
    CMD_SETMOTORRS              = 0X4E,  // 设置电机相电阻Rs
    CMD_SETMOTORLQ              = 0X4F,  // 设置电机q轴电感Lq
    CMD_SETMOTORLD              = 0X50,  // 设置电机d轴电感Ld
    CMD_ELECTRICALANGLE         = 0X51,  // 打印实际电角度 correctedAngle
    CMD_ELECTRICALANGLE_CLOSE   = 0X52,  // 关闭实际电角度 correctedAngle
    // ... 未来可以继续扩展
} CMD_TypeDef;

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

void Comm_CommandHandler(void);

#endif
