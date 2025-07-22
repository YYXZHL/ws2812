#include "ws2812_spi.h"
#include "tuya_iot_config.h"
#include "tuya_cloud_types.h"
#include "tal_log.h"
#include "tal_thread.h"
#include "tal_system.h"
#include "tkl_spi.h"

static UCHAR_T *s_buffer = NULL;
static TUYA_SPI_NUM_E s_spi_port;

/**
 * @brief 初始化驱动并分配缓冲区
 */
OPERATE_RET ws2812_spi_init(TUYA_SPI_NUM_E port) {
    OPERATE_RET rt;

    size_t buf_len = (size_t)WS2812_LED_COUNT * 24;  // 每灯 24 字节编码

    s_buffer = malloc(buf_len);
    if (!s_buffer) {
        return OPRT_MALLOC_FAILED;
    }

    TUYA_SPI_BASE_CFG_T cfg = {
        .mode      = TUYA_SPI_MODE0,
        .freq_hz   = WS2812_SPI_FREQ,
        .databits  = TUYA_SPI_DATA_BIT8,
        .bitorder  = TUYA_SPI_ORDER_MSB2LSB,
        .role      = TUYA_SPI_ROLE_MASTER,
        .type      = TUYA_SPI_AUTO_TYPE
    };

    TUYA_CALL_ERR_GOTO(tkl_spi_init(port, &cfg), EXIT_FAIL);
    return OPRT_OK;

EXIT_FAIL:
    free(s_buffer);
    s_buffer = NULL;
    return rt;
}

/**
 * @brief 设置单个像素的 GRB 数据到缓冲区
 */
OPERATE_RET ws2812_spi_set_pixel(UINT16_T index, UCHAR_T red, UCHAR_T green, UCHAR_T blue) {
    if (index >= WS2812_LED_COUNT || s_buffer == NULL) {
        return OPRT_INVALID_PARM;
    }

    uint32_t color = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    size_t base = (size_t)index * 24;
    for (int bit = 0; bit < 24; bit++) {
         s_buffer[base + bit] = ( ( color << bit ) & 0x800000 ) ? WS2812_1 : WS2812_0; 
    }
    return OPRT_OK;
}

/**
 * @brief 发送所有像素数据并拉低复位线
 */
OPERATE_RET ws2812_spi_refresh(VOID_T) {
    if (s_buffer == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    size_t len = (size_t)WS2812_LED_COUNT * 24;
    tkl_spi_send(s_spi_port, s_buffer, len);
    /* 拉低 >50μs 触发复位 */
    delay_ms(WS2812_RESET_DELAY_MS);
    return OPRT_OK;
}

/**
 * @brief 释放资源并反初始化 SPI
 */
OPERATE_RET ws2812_spi_deinit(VOID_T) {
    if (s_buffer) {
        free(s_buffer);
        s_buffer = NULL;
    }
    return tkl_spi_deinit(s_spi_port);
}

/**
 * @brief 设置所有 LED 为相同的颜色
 */
OPERATE_RET ws2812_spi_set_all(UCHAR_T red, UCHAR_T green, UCHAR_T blue) {
    if (s_buffer == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }
    for (UINT16_T i = 0; i < WS2812_LED_COUNT; i++) {
        ws2812_spi_set_pixel(i, red, green, blue);
    }
    return OPRT_OK;
}

#define W2812_TEST 0
VOID_T ws2812_app_init(VOID_T) 
{
#if(W2812_TEST == 0)
    // 假设使用 SPI0，控制 4 颗 WS2812
    ws2812_spi_init(TUYA_SPI_NUM_0);

    // 设置熄灭
    ws2812_spi_set_all(0, 0, 0);
    // 设置为绿色
    //ws2812_spi_set_all(0x00, 0x00, 0xFF);
    // ……（继续设置其他 LED）

    // 刷新到 LED
    ws2812_spi_refresh();

    // 完成后释放
    //ws2812_spi_deinit();

#else
    UCHAR_T send_buff[] = {0xE0,0xFF,0xF8,0xFF,0x55};
    uint8_t color = 0;
    // 呼吸灯亮度表（非线性变化，符合人眼感知）
    static const uint8_t gammaBreath[256] = {
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,
        1,   1,   2,   2,   2,   2,   2,   3,   3,   3,   4,   4,   5,   5,   6,   6,
        7,   7,   8,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  20,  21,
        23,  24,  26,  27,  29,  31,  33,  35,  37,  39,  42,  44,  47,  49,  52,  55,
        58,  61,  64,  67,  71,  74,  78,  82,  86,  90,  94,  98,  103, 107, 112, 117,
        122, 127, 132, 138, 143, 149, 155, 161, 167, 174, 180, 187, 194, 201, 208, 215,
        223, 230, 238, 246, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 246, 238, 230, 223,
        215, 208, 201, 194, 187, 180, 174, 167, 161, 155, 149, 143, 138, 132, 127, 122,
        117, 112, 107, 103, 98,  94,  90,  86,  82,  78,  74,  71,  67,  64,  61,  58,
        55,  52,  49,  47,  44,  42,  39,  37,  35,  33,  31,  29,  27,  26,  24,  23,
        21,  20,  18,  17,  16,  15,  14,  13,  12,  11,  10,  9,   8,   8,   7,   7,
        6,   6,   5,   5,   4,   4,   3,   3,   3,   2,   2,   2,   2,   2,   1,   1,
        1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
    };

    ws2812_spi_init(TUYA_SPI_NUM_0);
    while(1)
    {
        delay_ms(20);
        #if 0
        tkl_spi_send(TUYA_SPI_NUM_0, send_buff, 5);
        #else
        ws2812_spi_set_all(0x00, 0x00,gammaBreath[color++]);
        ws2812_spi_refresh();
        #endif
        TAL_PR_DEBUG("SPI send ok!\r\n\r\n\r\n");
    }
#endif
}


