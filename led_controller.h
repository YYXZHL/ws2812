
#ifndef __LED_CONTROLLER_H__
#define __LED_CONTROLLER_H__

#include "tuya_cloud_types.h"
#include "tal_sw_timer.h"
#include "tal_gpio.h"
#include "tdd_pixel_ws2812.h"

// LED数量定义 (从原ws2812_spi.h移植)
#define WS2812_LED_COUNT 12

#define TIMER_ID_STATE  TUYA_TIMER_NUM_1 // 定时器控制器ID
#define TIMER_ID_ACTION TUYA_TIMER_NUM_2 // 动作定时器ID
/**
 * @file led_controller.h
 * @brief LED状态机控制器 - 双定时器版本
 * 
 * 设计说明：
 * 1. 使用双定时器架构：状态定时器处理状态超时和转换，动作定时器处理LED动态效果
 * 2. 呼吸灯使用预计算的亮度表实现非线性亮度变化，符合人眼感知
 * 3. 所有时间参数通过宏定义配置，便于调整
 * 4. 状态机支持状态缓存机制，确保自检过程中不丢失指令
 * 5. 简化的单线程设计，无需互斥锁保护
 */

// ========================== 时间参数配置 ==========================
// 自检时间参数
#define INIT_RED_TIME     1000    // 红色显示时间 (ms)
#define INIT_GREEN_TIME   1000    // 绿色显示时间 (ms)
#define INIT_BLUE_TIME    1000    // 蓝色显示时间 (ms)

// 状态超时参数
#define CONFIG_SUCCESS_TIMEOUT  2000  // 配网成功显示时间 (ms)
#define VOLUME_DISPLAY_TIMEOUT  2000  // 音量显示时间 (ms)
#define DIALOG_TOTAL_TIME       5000  // 对话状态总时间 (ms)

// 闪烁时间参数
#define DIALOG_LIGHT_ON_TIME    100   // 对话状态亮灯时间 (ms)
#define DIALOG_LIGHT_OFF_TIME   150   // 对话状态灭灯时间 (ms)
#define DIALOG_BLINK_COUNT      (DIALOG_TOTAL_TIME / (DIALOG_LIGHT_ON_TIME + DIALOG_LIGHT_OFF_TIME)) // 闪烁次数

// 呼吸灯参数
#define BREATH_TIMER_INTERVAL   10    // 呼吸灯定时器周期 (ms)
#define BREATH_TABLE_SIZE       256   // 呼吸灯亮度表大小

// 新增场景参数
#define CONFIGURING_BLINK_ON_TIME   250  // 配网中：绿灯亮时间 (ms)
#define CONFIGURING_BLINK_OFF_TIME  250  // 配网中：绿灯灭时间 (ms)
#define CONFIG_SUCCESS_STEP_INTERVAL 200  // 配网成功：每200ms亮一个灯 (ms)
#define CONFIG_SUCCESS_HOLD_TIME     2000 // 配网成功：达标后保持亮2s (ms)
#define WAKE_BLINK_INTERVAL          200  // 唤醒：蓝灯闪烁周期 (ms)
#define WAKE_BLINK_TIMES             2    // 唤醒：闪两下
#define WAKE_HOLD_TIME               12000 // 唤醒：常亮等待对话超时 (ms)
#define STANDBY_STEP_INTERVAL        250  // 待机：跑马步进周期 (ms)

// ========================== 状态枚举定义 ==========================
typedef enum {
    LED_INIT,          ///< 上电自检状态（红->绿->蓝）
    LED_IDLE,          ///< 空闲状态（所有LED熄灭）
    LED_CONFIGURING,   ///< 配网中（绿灯闪烁：250ms亮/250ms灭）
    LED_CONFIG_SUCCESS,///< 配网成功（显示WIFI信号强度：每200ms按顺序点亮，达标后保持2s后熄灭）
    LED_NET_ERROR,     ///< 网络异常（红灯常亮）
    LED_DIALOG,        ///< 对话中（蓝灯闪烁）
    LED_VOLUME,        ///< 调节音量（黄灯等级显示）
    LED_BREATHING,     ///< 呼吸灯效果（蓝灯呼吸）
    LED_WAKE,          ///< 唤醒（蓝灯闪两下后常亮，12秒内无对话关闭）
    LED_STANDBY        ///< 待机（单个绿灯按自定义顺序360度循环跑马）
} LedState;

/**
 * @brief 初始化LED控制器
 * 
 * 功能说明：
 * 1. 初始化状态机数据结构
 * 2. 初始化TDD WS2812驱动
 * 3. 创建状态定时器和动作定时器
 * 4. 进入上电自检状态
 */
void led_controller_init(void);

/**
 * @brief 设置LED状态
 * 
 * @param new_state 新状态（LedState枚举值）
 * @param value 状态附加参数：
 *   - LED_CONFIG_SUCCESS: WIFI信号强度(0-12)
 *   - LED_VOLUME: 音量等级(0-12)
 *   - 其他状态: 忽略此参数
 * 
 * 状态转换说明：
 * 1. 如果当前处于自检状态，新状态将被缓存，自检完成后自动执行
 * 2. 其他状态下立即执行新状态，并清理前一个状态的资源
 */
void set_led_state(LedState new_state, uint8_t value);

/**
 * @brief 去初始化LED控制器
 * 
 * 功能说明：
 * 1. 关闭TDD WS2812驱动
 * 2. 释放相关资源
 */
void led_controller_deinit(void);

#endif /* __LED_CONTROLLER_H__ */
