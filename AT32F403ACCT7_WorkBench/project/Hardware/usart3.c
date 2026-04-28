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
#define DEBUG_PRINT(fmt, ...)
#endif

#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

uint8_t uart3_tx_buffer[USART3_TX_BUFFER_SIZE] = {0};
volatile uint8_t usart3_tx_dma_status = 0;

/******************************************************************************
 * 函数名称：fputc
 * 功能描述：重定向 printf 到 USART3（通过串口输出调试信息）。
 * 输入参数：ch - 要发送的字符
 *           f  - 文件指针（stdio 内部使用）
 * 返回值：发送的字符
 ******************************************************************************/
int fputc(int ch, FILE *f)
{
    while (usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(PRINT_UART, (uint16_t)ch);
    while (usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET);
    return ch;
}

/******************************************************************************
 * 函数名称：USART3_SendPacket
 * 功能描述：通过 USART3 + DMA 发送一帧数据给上位机。
 *
 * 帧格式：
 *   ┌──────┬──────┬──────┬──────────┬────────┬──────┐
 *   │ 帧头  │ 命令 │ 长度 │ 数据区    │ 校验和 │ 帧尾 │
 *   │ 0xA5 │ 1 B  │ 1 B  │ N B      │ 1 B    │ 0x49 │
 *   └──────┴──────┴──────┴──────────┴────────┴──────┘
 *
 *   数据区：count 个 float（4 字节/个）
 *   校验和 = Header + CMD + Length + Payload（逐字节累加）
 *
 *   使用 DMA1 通道 1 发送，传输完成后由 DMA 中断释放信号量。
 *
 * 输入参数：cmd    - 命令字
 *           values - 浮点数据数组指针
 *           count  - 浮点数个数
 ******************************************************************************/
void USART3_SendPacket(uint8_t cmd, float *values, uint8_t count)
{
    uint8_t idx = 0;
    uint8_t data_len = count * sizeof(float);
    uint8_t checksum = 0;
    uint8_t *pdata = (uint8_t *)values;

    uart3_tx_buffer[idx++] = FRAME_HEAD;       // 帧头
    uart3_tx_buffer[idx++] = cmd;              // 命令字
    uart3_tx_buffer[idx++] = data_len;          // 数据长度

    for (uint8_t i = 0; i < data_len; i++) {   // 数据区
        uart3_tx_buffer[idx++] = pdata[i];
    }

    for (uint8_t i = 0; i < idx; i++) {        // 校验和
        checksum += uart3_tx_buffer[i];
    }
    uart3_tx_buffer[idx++] = checksum;

    uart3_tx_buffer[idx++] = FRAME_TAIL;       // 帧尾

    if (usart3_tx_dma_status == 1) {
        usart3_tx_dma_status = 0;
        dma_data_number_set(DMA1_CHANNEL1, idx);
        dma_channel_enable(DMA1_CHANNEL1, TRUE);
    }
}

/* =================== 接收缓冲区 =================== */
#define FRAME_LEN 8
static uint8_t rx_buf[FRAME_LEN];
static uint8_t rx_idx = 0;
static uint8_t rx_start = 0;
uint8_t g_commCmd = 0x00;
float g_cmdData = 0.0f;

/******************************************************************************
 * 函数名称：USART3_ParseFixedCommand
 * 功能描述：逐字节解析来自上位机的固定长度命令帧。
 *
 * 下行协议帧结构（固定 8 字节）：
 *   偏移   0      1       2       3       4       5       6       7
 *   ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *   │ 帧头  │ CMD  │ data0│ data1│ data2│ data3│ 校验  │ 帧尾  │
 *   │ 0xA5 │ cmd  │ float/uint32 (4B)       │ sum   │ 0x49  │
 *   └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *
 * 校验和 = 帧头 + CMD + data0~3
 * 解析成功后将命令写入 g_commCmd，数据解析为 float 存入 g_cmdData。
 *
 * 输入参数：byte - USART3 接收到的字节
 ******************************************************************************/
void USART3_ParseFixedCommand(uint8_t byte)
{
    /* 检测帧头 */
    if (rx_start == 0 && byte == FRAME_HEAD) {
        rx_idx = 0;
        rx_buf[rx_idx++] = byte;
        DEBUG_PRINT("rx_idx : %d, 0x%02X\r\n", rx_idx - 1, rx_buf[rx_idx - 1]);
        rx_start = 1;
    }
    else if (rx_start == 1) {
        rx_buf[rx_idx++] = byte;

        if (rx_idx == FRAME_LEN && rx_buf[FRAME_LEN - 1] == FRAME_TAIL) {
            /* 校验和检查：取前 6 字节累加 */
            uint8_t checksum = 0;
            for (int i = 0; i < 6; i++) checksum += rx_buf[i];
            if (checksum != rx_buf[6]) {
                rx_idx = 0;
                rx_start = 0;
                DEBUG_PRINT("check err, rx_buf[6] is 0x%02X, checksum is 0x%02X\r\n",
                            rx_buf[6], checksum);
                return;
            }

            DEBUG_PRINT("check OK\r\n");
            g_commCmd = rx_buf[1];
            memcpy(&g_cmdData, &rx_buf[2], 4);

            rx_idx = 0;
            rx_start = 0;
        }

        /* 超长保护：超过 8 字节丢弃 */
        if (rx_idx >= 8) {
            rx_idx = 0;
            rx_start = 0;
        }
    }
}
