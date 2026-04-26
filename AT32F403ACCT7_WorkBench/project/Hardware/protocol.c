#include "protocol.h"
#include "usart3.h"  
#include "freertos_app.h"
#include "led.h"  
#include "flash_ops.h"
#include "FOC.h"
#include "mostemp.h"

volatile uint8_t anglePrintingEnabled = 0;
volatile uint8_t uabcEnabled = 0;
volatile uint8_t adcEnabled  = 0;
volatile uint8_t tabcEnabled = 0;
volatile uint8_t IabcEnabled = 0;
volatile uint8_t UAlpha_BetaEnabled = 0;
volatile uint8_t IAlpha_BetaEnabled = 0;
volatile uint8_t IQ_ID_Enabled = 0;
volatile uint8_t mostemp_Enabled = 0;
volatile uint8_t speed_Enabled = 0;
volatile uint8_t speedOut_Enabled = 0;
volatile uint8_t local_Enabled = 0;
volatile uint8_t localOut_Enabled = 0;

static float g_zeroOffset = 0.0f;
static float g_correctedElecAngle = 0.0f;

void Comm_CommandHandler(void)
{
    led_device_t *ledRun = freertos_get_run_led();
    float data[14] = {0.0f};
    
    if((g_commCmd != CMD_NONE) && (0 == led_get(ledRun))){
        led_set(ledRun, 1);
        vTaskDelay(50);
        led_set(ledRun, 0);
    }
    
    //数据回传上位机
    switch(g_commCmd)
    {
        case CMD_CONNECT_MOTOR:
            foc_params_load(&g_readback);
            g_pMotor->pole_pairs = g_readback.pole_pairs;
            g_pMotor->dir        = g_readback.dir;
            g_pMotor->zeroOffset = g_readback.elec_offset;
            g_pMotor->speedDir   = g_readback.speeddir;
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
            USART3_SendPacket(CMD_CONNECT_MOTOR, &data[0], 14);
          
            mostemp_Enabled = 1;
            g_commCmd = CMD_NONE;  
            break;
        
        case CMD_MECHANICALANGLE:
            anglePrintingEnabled = 1;
            g_commCmd = CMD_NONE;  
            break;
        
        case CMD_MECHANICALANGLE_CLOSE:
            anglePrintingEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETPAIRS:
            foc_params_load(&g_params); 
            g_params.pole_pairs  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETDIR:
            foc_params_load(&g_params); 
            g_params.dir  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ZEROCALIBRATIO:
            led_set(ledRun, 1);
            AngleInitZeroOffset(&g_zeroOffset, &g_correctedElecAngle);
            data[0] = g_zeroOffset;
            data[1] = g_correctedElecAngle;
            
            foc_params_load(&g_params); 
            g_params.elec_offset  = g_zeroOffset;
            foc_params_save(&g_params);
            
            USART3_SendPacket(CMD_ZEROCALIBRATIO_OVER, &data[0], 2);
            led_set(ledRun, 0);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UABC:
            uabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UABC_CLOSE:
            uabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETUQ:
            g_pMotor->uq = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ADC:
            adcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ADC_CLOSE:
            adcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_DCVBUS:
            data[0] = getVbus();
            USART3_SendPacket(CMD_DCVBUS, &data[0], 1);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_TABC:
            tabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_TABC_CLOSE:
            tabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IABC:
            IabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IABC_CLOSE:
            IabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UALPHA_BETA:
            UAlpha_BetaEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UALPHA_BETA_CLOSE:
            UAlpha_BetaEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IALPHA_BETA:
            IAlpha_BetaEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IALPHA_BETA_CLOSE:
            IAlpha_BetaEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IQ_ID:
            IQ_ID_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IQ_ID_CLOSE:
            IQ_ID_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQ:
            g_pMotor->tariq = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETID:
            g_pMotor->tarid = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_OPEN_LOOP:
            g_pMotor->ctrolmode = FOC_OPEN_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_CURRENT_LOOP:
            g_pMotor->ctrolmode = FPC_CURRENT_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED_LOOP:
            g_pMotor->ctrolmode = FOC_SPEED_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_POSITION_LOOP :
            g_pMotor->ctrolmode = FOC_POSITION_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETUD :
            g_pMotor->ud = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQPIDKP :
            g_pMotor->iqPID.kp = g_cmdData;
            g_pMotor->idPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQPIDKI :
            g_pMotor->iqPID.ki = g_cmdData;
            g_pMotor->idPID.ki = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED:
            speed_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED_CLODE:
            speed_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDDIR:
            foc_params_load(&g_params); 
            g_params.speeddir  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEEDOUT:
            speedOut_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEEDOUT_CLOSE:
            speedOut_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDTAR:
            g_pMotor->tar_speed = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDPIDKP:
            g_pMotor->speedPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDPIDKI:
            g_pMotor->speedPID.ki = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALTAR:
            g_pMotor->tarPosition = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCAL:
            local_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCAL_CLOSE:
            local_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCALOUT:
            localOut_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCALOUT_CLOSE:
            localOut_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALPIDKP:
            g_pMotor->positionPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALPIDKD:
            g_pMotor->positionPID.kd = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
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
        
        default:
            break;
    }
}


