#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "usart3.h"  
#include "protocol.h" 
#include "freertos_app.h"

//#define USART3_DEBUG

#ifdef USART3_DEBUG
#define DEBUG_PRINT(fmt, ...)    printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)    // 不打印
#endif


#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

uint8_t uart3_tx_buffer[USART3_TX_BUFFER_SIZE] = {0};
volatile uint8_t usart3_tx_dma_status = 0;

int fputc(int ch, FILE *f)
{
  while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(PRINT_UART, (uint16_t)ch);
  while(usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET);
  return ch;
}

/**
 * @brief  USART3 + DMA 发送一帧浮点数据
 * @param  cmd     命令字（用户传入）
 * @param  values  浮点数组指针
 * @param  count   浮点数个数
 * 
 * 帧格式：
 * [帧头 0xA5][命令字][长度][数据...][校验和][帧尾 0x49]
 * 
 * 校验和 = 帧头到数据区的逐字节累加。
 * 使用 DMA 发送，传输完成由中断释放信号量。
 */
void USART3_SendPacket(uint8_t cmd, float *values, uint8_t count)
{
    uint8_t idx = 0;
    uint8_t data_len = count * sizeof(float);
    uint8_t checksum = 0;
    uint8_t *pdata = (uint8_t *)values;

    // 帧头
    uart3_tx_buffer[idx++] = FRAME_HEAD;

    // 命令字
    uart3_tx_buffer[idx++] = cmd; // 数据上传命令

    // 数据长度
    uart3_tx_buffer[idx++] = data_len;

    // 数据区
    for (uint8_t i = 0; i < data_len; i++)
    {
        uart3_tx_buffer[idx++] = pdata[i];
    }

    // 校验和计算（Header+CMD+Length+Payload）
    for (uint8_t i = 0; i < idx; i++)
    {
        checksum += uart3_tx_buffer[i];
    }
    uart3_tx_buffer[idx++] = checksum;

    // 帧尾
    uart3_tx_buffer[idx++] = FRAME_TAIL;

    while(xSemaphoreTake(usart3_dma_tx_sem_handle,pdMS_TO_TICKS(2)!=pdTRUE));//获取信号量，等待DMA发送可用
    
    // 配置DMA发送长度并启动
    dma_data_number_set(DMA1_CHANNEL1, idx);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

// 静态缓冲区和索引
#define FRAME_LEN 8
static uint8_t rx_buf[FRAME_LEN];
static uint8_t rx_idx = 0;
static uint8_t rx_start = 0;
uint8_t g_commCmd = 0x00;

/**
 * @brief 逐字节解析固定长度协议帧
 * 协议帧结构（固定长度 8 字节）：
 * 索引   0      1       2       3       4       5       6       7
 *          ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * 名称   帧头   CMD   数据0   数据1   数据2   数据3   校验   帧尾
 * 类型   uint8  uint8  uint8  uint8  uint8  uint8  uint8  uint8
 * 值     0xA5  命令字 float/uint32                 校验   0x49
 * 校验和计算：
 *   checksum = 帧头 + CMD + 数据字节0 + 数据字节1 + 数据字节2 + 数据字节3
 * A5 01 00 00 00 00 01 A6 
 * 30 2E 31 32 = 0.12 ,67
 */
void USART3_ParseFixedCommand(uint8_t byte)
{
    // 帧头
    if(rx_start == 0 && byte == FRAME_HEAD){
        rx_idx = 0;
        rx_buf[rx_idx++] = byte;
        DEBUG_PRINT("rx_idx : %d, 0x%02X \r\n",rx_idx-1, rx_buf[rx_idx-1]);

        rx_start = 1;
        
    }
    else if(rx_start == 1){
        rx_buf[rx_idx++] = byte;
        //DEBUG_PRINT("rx_idx : %d, 0x%02X \r\n",rx_idx-1, rx_buf[rx_idx-1]);
        //DEBUG_PRINT("rx_idx is %d, FRAME_LEN is 0x%02X, rx_buf[FRAME_LEN - 1] is 0x%02X, FRAME_TAILis 0x%02X\r\n",rx_idx,FRAME_LEN,rx_buf[FRAME_LEN - 1],FRAME_TAIL);
        if(rx_idx == FRAME_LEN && rx_buf[FRAME_LEN - 1] == FRAME_TAIL){
            //DEBUG_PRINT("ENTER FRAME_TAIL\r\n");
            // 校验和检查 (帧头+CMD+4字节数据)
            uint8_t checksum = 0;
            for(int i = 0; i < 6; i++) checksum += rx_buf[i];
            if(checksum != rx_buf[6])
            {
                rx_idx = 0; // 出错重置
                rx_start = 0;
                //DEBUG_PRINT("check err, rx_buf[6] is 0x%02X, checksum is 0x%02X\r\n",rx_buf[6], checksum);
                return;
            }
            
            DEBUG_PRINT("check OK\r\n");
            // 解析命令和数据
            g_commCmd = rx_buf[1];
            float data;
            memcpy(&data, &rx_buf[2], 4);    
            //DEBUG_PRINT("raw bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n",rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]);
            //DEBUG_PRINT("float data is : %f\r\n",data);

            // 重置索引准备接收下一帧
            rx_idx = 0;
            rx_start = 0;
        }
        
        if(rx_idx >= 8){
            rx_idx = 0;
            rx_start = 0;
        }
    }
}

