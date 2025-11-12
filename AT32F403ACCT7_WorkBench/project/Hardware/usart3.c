#include "usart3.h"           
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

uint8_t uart3_tx_buffer[USART3_TX_BUFFER_SIZE] = {0};
volatile uint8_t usart3_tx_dma_status = 0;

//=========================上位机通信 =========================//
void USART3_SendPacket(float *values, uint8_t count)
{
    uint8_t idx = 0;
    uint8_t data_len = count * sizeof(float);
    uint8_t checksum = 0;
    uint8_t *pdata = (uint8_t *)values;

    // 帧头
    uart3_tx_buffer[idx++] = FRAME_HEAD;

    // 命令字
    uart3_tx_buffer[idx++] = 0x01; // 数据上传命令

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

    // 等待上一次DMA发送完成
    while (usart3_tx_dma_status == 0);

    usart3_tx_dma_status = 0;

    // 配置DMA发送长度并启动
    dma_data_number_set(DMA1_CHANNEL1, idx);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
}


