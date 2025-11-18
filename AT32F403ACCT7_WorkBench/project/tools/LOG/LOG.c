#include <stdint.h>
#include <string.h>

//#include "usbd_int.h"
//#include "cdc_class.h"
//#include "cdc_desc.h"
#include "LOG.h"

//extern usbd_core_type usb_core_dev;



//const uint8_t FLOAT_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};


//uint16_t PackFloatArrayToBytes(const float *input, uint8_t float_count, uint8_t *output)
//{
//    uint8_t *bytePtr = (uint8_t *)input; 

//    for (uint8_t i = 0; i < float_count * 4; i++) {
//        output[i] = bytePtr[i];
//    }

//    output[float_count * 4] = 0x00;
//    output[float_count * 4 + 1] = 0x00;
//    output[float_count * 4 + 2] = 0x80;
//    output[float_count * 4 + 3] = 0x7F;

//    return float_count * 4 + 4;
//}

//#include <stdarg.h>
//#include <stdio.h>

//char usb_print_buffer[128]; // 

//void usb_printf(const char *fmt, ...)
//{
//    va_list args;
//    va_start(args, fmt);
//    int len = vsnprintf(usb_print_buffer, sizeof(usb_print_buffer), fmt, args);
//    va_end(args);

//    if (len > 0)
//    {
//        uint32_t timeout = 10000;
//        while (usb_vcp_send_data(&usb_core_dev, (uint8_t *)usb_print_buffer, len) != SUCCESS)
//        {
//            if (--timeout == 0)
//                break; // 超时退出，避免死循环
//        }
//    }
//}







