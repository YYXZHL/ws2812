#include "led_controller.h"
#include "tal_log.h"
#include "tal_sw_timer.h"
#include "tal_gpio.h"
#include "tdd_pixel_ws2812.h"
#include "tdl_pixel_driver.h"
#include <string.h>

// TDD WS2812驱动函数声明
OPERATE_RET tdd_2812_driver_open(OUT DRIVER_HANDLE_T *handle, IN unsigned short pixel_num);
OPERATE_RET tdd_ws2812_driver_close(IN DRIVER_HANDLE_T *handle);
OPERATE_RET tdd_ws2812_driver_send_data(IN DRIVER_HANDLE_T handle, IN unsigned short *data_buf, IN unsigned int buf_len);

// 颜色分量结构（RGB格式）
typedef struct {
    uint8_t r;  // 红色分量
    uint8_t g;  // 绿色分量
    uint8_t b;  // 蓝色分量
} RGBColor;

// 预定义颜色（RGB格式）
static const RGBColor COLOR_BLACK   = {0, 0, 0};     // 黑色（LED关闭）
static const RGBColor COLOR_RED     = {255, 0, 0};   // 红色
static const RGBColor COLOR_GREEN   = {0, 255, 0};   // 绿色
static const RGBColor COLOR_BLUE    = {0, 0, 255};   // 蓝色
static const RGBColor COLOR_YELLOW  = {255, 255, 0}; // 黄色

// 自定义点亮顺序（0-based）：9,8,7,6,5,4,3,2,1,12,11,10 => 8,7,6,5,4,3,2,1,0,11,10,9
static const uint8_t LED_ORDER[WS2812_LED_COUNT] = {8, 7, 6, 5, 4, 3, 2, 1, 0, 11, 10, 9};

// 呼吸灯亮度表（非线性变化，符合人眼感知）
static const uint8_t BREATH_BRIGHTNESS_TABLE[BREATH_TABLE_SIZE] = {
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

// LED控制状态机结构
typedef struct {
    LedState current_state;      // 当前状态
    LedState pending_state;      // 等待状态（在自检过程中接收的新状态）
    uint8_t pending_value;       // 等待状态参数
    BOOL_T has_pending_state;    // 是否有等待状态
    
    // 状态专用数据
    union {
        struct {
            uint8_t step;        // 自检步骤：0-红,1-绿,2-蓝
        } init;
        struct {
            uint8_t index;       // 呼吸灯查表索引 (0-255)
        } breath;
        struct {
            BOOL_T is_light_on;  // 当前LED亮灭状态
            uint16_t blink_count; // 已闪烁次数
        } blink;                 // 对话/简单闪烁使用
        struct {
            uint8_t target_level;   // 目标等级（0-12）
            uint8_t current_count;  // 当前已点亮数量
            BOOL_T hold_phase;      // 是否处于保持阶段
        } cfgsucc;                // 配网成功序列
        struct {
            uint8_t index;          // 跑马灯当前位置
        } standby;                // 待机跑马
        struct {
            uint8_t completed_toggles; // 已完成的明灭次数
            BOOL_T in_solid_phase;     // 是否进入常亮阶段
            BOOL_T is_light_on;        // 当前亮灭
        } wake;                   // 唤醒
    } state_data;
    
    // 定时器
    TIMER_ID main_timer;   // 主定时器：处理所有状态转换和动作
} LedController;

static LedController led_ctrl;

// TDD WS2812驱动相关变量
static DRIVER_HANDLE_T tdd_pixel_handle = NULL;
static unsigned short pixel_buffer[WS2812_LED_COUNT * 3]; // RGB数据缓冲区
static BOOL_T tdd_driver_initialized = FALSE;

// 直接调用TDD WS2812驱动函数（不再使用函数指针接口）

// TDD驱动初始化函数
static OPERATE_RET tdd_pixel_init(void) {
    OPERATE_RET ret;
    
    if (tdd_driver_initialized) {
        return OPRT_OK;
    }
    
    // 注册WS2812驱动
    PIXEL_DRIVER_CONFIG_T driver_config = {
        .port = TUYA_SPI_NUM_0,
        .line_seq = GRB_ORDER  // WS2812使用GRB顺序
    };
    
    ret = tdd_ws2812_driver_register(&driver_config);
    if (ret != OPRT_OK) {
        TAL_PR_ERR("Failed to register TDD WS2812 driver: %d", ret);
        return ret;
    }
    
    // 打开设备
    ret = tdd_2812_driver_open(&tdd_pixel_handle, WS2812_LED_COUNT);
    if (ret != OPRT_OK) {
        TAL_PR_ERR("Failed to open TDD WS2812 device: %d", ret);
        return ret;
    }
    
    // 清空缓冲区
    memset(pixel_buffer, 0, sizeof(pixel_buffer));
    
    tdd_driver_initialized = TRUE;
    TAL_PR_DEBUG("TDD WS2812 driver initialized successfully");
    
    return OPRT_OK;
}

// 设置单个LED的颜色
static OPERATE_RET tdd_pixel_set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (!tdd_driver_initialized || index >= WS2812_LED_COUNT) {
        return OPRT_INVALID_PARM;
    }
    
    // TDD驱动使用GRB顺序
    pixel_buffer[index * 3 + 0] = green;  // G
    pixel_buffer[index * 3 + 1] = red;    // R
    pixel_buffer[index * 3 + 2] = blue;   // B
    
    return OPRT_OK;
}

// 设置所有LED为同一颜色
static OPERATE_RET tdd_pixel_set_all(uint8_t red, uint8_t green, uint8_t blue) {
    if (!tdd_driver_initialized) {
        return OPRT_INVALID_PARM;
    }
    
    for (int i = 0; i < WS2812_LED_COUNT; i++) {
        tdd_pixel_set_pixel(i, red, green, blue);
    }
    
    return OPRT_OK;
}

// 刷新LED显示
static OPERATE_RET tdd_pixel_refresh(void) {
    if (!tdd_driver_initialized || tdd_pixel_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }
    
    return tdd_ws2812_driver_send_data(tdd_pixel_handle, pixel_buffer, WS2812_LED_COUNT * 3);
}

// TDD驱动去初始化函数
static OPERATE_RET tdd_pixel_deinit(void) {
    if (!tdd_driver_initialized) {
        return OPRT_OK;
    }
    
    if (tdd_pixel_handle != NULL) {
        tdd_ws2812_driver_close(&tdd_pixel_handle);
        tdd_pixel_handle = NULL;
    }
    
    tdd_driver_initialized = FALSE;
    TAL_PR_DEBUG("TDD WS2812 driver deinitialized");
    
    return OPRT_OK;
}

// 设置所有LED为同一颜色
static void set_all_leds(const RGBColor *color) {
    tdd_pixel_set_all(color->r, color->g, color->b);
    tdd_pixel_refresh();
}

// 设置等级显示（用于信号强度和音量）
static void set_level_leds(const RGBColor *color, uint8_t level) {
    // 确保不超过LED数量
    if (level > WS2812_LED_COUNT) {
        level = WS2812_LED_COUNT;
    }
    
    // 先全部置黑，再按顺序点亮前level个
    for (int i = 0; i < WS2812_LED_COUNT; i++) {
        uint8_t phys = LED_ORDER[i];
        if (i < level) {
            tdd_pixel_set_pixel(phys, color->r, color->g, color->b);
        } else {
            tdd_pixel_set_pixel(phys, COLOR_BLACK.r, COLOR_BLACK.g, COLOR_BLACK.b);
        }
    }
    tdd_pixel_refresh();
}

// 主定时器回调：处理所有状态事件
static void main_timer_cb(TIMER_ID timer_id, VOID_T *arg) {
    
    switch (led_ctrl.current_state) {
        case LED_INIT:
            // 自检状态转换：红->绿->蓝
            led_ctrl.state_data.init.step++;
            
            if (led_ctrl.state_data.init.step == 1) {
                // 切换到绿色
                set_all_leds(&COLOR_GREEN);
                tal_sw_timer_start(led_ctrl.main_timer, INIT_GREEN_TIME, TAL_TIMER_ONCE);
            } else if (led_ctrl.state_data.init.step == 2) {
                // 切换到蓝色
                set_all_leds(&COLOR_BLUE);
                tal_sw_timer_start(led_ctrl.main_timer, INIT_BLUE_TIME, TAL_TIMER_ONCE);
            } else {
                // 自检完成
                TAL_PR_DEBUG("Init complete");
                
                // 进入空闲状态
                led_ctrl.current_state = LED_IDLE;
                
                // 执行等待状态或进入空闲
                if (led_ctrl.has_pending_state) {
                    // 保存等待状态值
                    LedState next_state = led_ctrl.pending_state;
                    uint8_t next_value = led_ctrl.pending_value;
                    
                    // 重置等待状态
                    led_ctrl.has_pending_state = FALSE;
                    
                    // 执行新状态
                    set_led_state(next_state, next_value);
                    return;
                } else {
                    set_all_leds(&COLOR_BLACK);
                }
            }
            break;
            
        case LED_CONFIG_SUCCESS: {
            if (!led_ctrl.state_data.cfgsucc.hold_phase) {
                if (led_ctrl.state_data.cfgsucc.current_count < led_ctrl.state_data.cfgsucc.target_level) {
                    led_ctrl.state_data.cfgsucc.current_count++;
                    set_level_leds(&COLOR_GREEN, led_ctrl.state_data.cfgsucc.current_count);
                    if (led_ctrl.state_data.cfgsucc.current_count >= led_ctrl.state_data.cfgsucc.target_level) {
                        // 达到目标，进入保持阶段
                        led_ctrl.state_data.cfgsucc.hold_phase = TRUE;
                        tal_sw_timer_start(led_ctrl.main_timer, CONFIG_SUCCESS_HOLD_TIME, TAL_TIMER_ONCE);
                    } else {
                        tal_sw_timer_start(led_ctrl.main_timer, CONFIG_SUCCESS_STEP_INTERVAL, TAL_TIMER_ONCE);
                    }
                } else {
                    // 目标为0，直接进入保持阶段
                    led_ctrl.state_data.cfgsucc.hold_phase = TRUE;
                    tal_sw_timer_start(led_ctrl.main_timer, CONFIG_SUCCESS_HOLD_TIME, TAL_TIMER_ONCE);
                }
            } else {
                // 保持结束 -> 熄灭并空闲
                led_ctrl.current_state = LED_IDLE;
                set_all_leds(&COLOR_BLACK);
            }
            break;
        }
        
        case LED_VOLUME:
            // 显示状态超时，进入空闲
            led_ctrl.current_state = LED_IDLE;
            set_all_leds(&COLOR_BLACK);
            break;
            
        case LED_DIALOG:
            // 对话状态：切换亮灭状态
            if (led_ctrl.state_data.blink.is_light_on) {
                // 当前亮 -> 切换为灭
                set_all_leds(&COLOR_BLACK);
                led_ctrl.state_data.blink.is_light_on = FALSE;
                tal_sw_timer_start(led_ctrl.main_timer, DIALOG_LIGHT_OFF_TIME, TAL_TIMER_ONCE);
            } else {
                // 当前灭 -> 切换为亮
                set_all_leds(&COLOR_BLUE);
                led_ctrl.state_data.blink.is_light_on = TRUE;
                led_ctrl.state_data.blink.blink_count++;
                
                // 检查是否达到总闪烁次数
                if (led_ctrl.state_data.blink.blink_count >= DIALOG_BLINK_COUNT) {
                    led_ctrl.current_state = LED_IDLE;
                    set_all_leds(&COLOR_BLACK);
                } else {
                    tal_sw_timer_start(led_ctrl.main_timer, DIALOG_LIGHT_ON_TIME, TAL_TIMER_ONCE);
                }
            }
            break;
            
        case LED_CONFIGURING: {
            // 配网中：绿灯250ms亮/250ms灭闪烁
            if (led_ctrl.state_data.blink.is_light_on) {
                set_all_leds(&COLOR_BLACK);
                led_ctrl.state_data.blink.is_light_on = FALSE;
                tal_sw_timer_start(led_ctrl.main_timer, CONFIGURING_BLINK_OFF_TIME, TAL_TIMER_ONCE);
            } else {
                set_all_leds(&COLOR_GREEN);
                led_ctrl.state_data.blink.is_light_on = TRUE;
                tal_sw_timer_start(led_ctrl.main_timer, CONFIGURING_BLINK_ON_TIME, TAL_TIMER_ONCE);
            }
            break;
        }
        
        case LED_BREATHING: {
            // 更新呼吸灯索引
            led_ctrl.state_data.breath.index++;
            
            // 处理索引循环
            if (led_ctrl.state_data.breath.index >= BREATH_TABLE_SIZE) {
                led_ctrl.state_data.breath.index = 0;
            }
            
            // 获取当前亮度值
            uint8_t brightness = BREATH_BRIGHTNESS_TABLE[led_ctrl.state_data.breath.index];
            
            // 呼吸灯：蓝灯呼吸
            tdd_pixel_set_all(0, 0, brightness);
            tdd_pixel_refresh();
            
            // 设置下一次呼吸定时
            tal_sw_timer_start(led_ctrl.main_timer, BREATH_TIMER_INTERVAL, TAL_TIMER_ONCE);
            break;
        }
        
        case LED_WAKE: {
            if (!led_ctrl.state_data.wake.in_solid_phase) {
                // 闪烁阶段
                if (led_ctrl.state_data.wake.is_light_on) {
                    set_all_leds(&COLOR_BLACK);
                    led_ctrl.state_data.wake.is_light_on = FALSE;
                } else {
                    set_all_leds(&COLOR_BLUE);
                    led_ctrl.state_data.wake.is_light_on = TRUE;
                }
                led_ctrl.state_data.wake.completed_toggles++;
                
                if (led_ctrl.state_data.wake.completed_toggles >= (WAKE_BLINK_TIMES * 2)) {
                    // 进入常亮阶段
                    set_all_leds(&COLOR_BLUE);
                    led_ctrl.state_data.wake.in_solid_phase = TRUE;
                    tal_sw_timer_start(led_ctrl.main_timer, WAKE_HOLD_TIME, TAL_TIMER_ONCE);
                } else {
                    tal_sw_timer_start(led_ctrl.main_timer, WAKE_BLINK_INTERVAL, TAL_TIMER_ONCE);
                }
            } else {
                // 常亮结束 -> 熄灭并空闲
                led_ctrl.current_state = LED_IDLE;
                set_all_leds(&COLOR_BLACK);
            }
            break;
        }
        
        case LED_STANDBY: {
            // 单个绿灯按自定义顺序跑马
            led_ctrl.state_data.standby.index++;
            if (led_ctrl.state_data.standby.index >= WS2812_LED_COUNT) {
                led_ctrl.state_data.standby.index = 0;
            }
            // 清空
            tdd_pixel_set_all(0, 0, 0);
            // 点亮当前位置
            uint8_t phys = LED_ORDER[led_ctrl.state_data.standby.index];
            tdd_pixel_set_pixel(phys, COLOR_GREEN.r, COLOR_GREEN.g, COLOR_GREEN.b);
            tdd_pixel_refresh();
            tal_sw_timer_start(led_ctrl.main_timer, STANDBY_STEP_INTERVAL, TAL_TIMER_ONCE);
            break;
        }
        
        default:
            // 其他状态无需处理
            break;
    }
}

// 清理当前状态资源
static void cleanup_current_state(void) {
    // 停止主定时器
    tal_sw_timer_stop(led_ctrl.main_timer);
    
    // 重置状态数据
    memset(&led_ctrl.state_data, 0, sizeof(led_ctrl.state_data));
}

// 初始化LED控制器
void led_controller_init(void) {
    TAL_PR_DEBUG("Initializing LED controller");
    
    // 清零控制结构体
    memset(&led_ctrl, 0, sizeof(LedController));
    
    // 初始化TDD WS2812驱动
    if (OPRT_OK != tdd_pixel_init()) {
        TAL_PR_ERR("Failed to initialize TDD WS2812 driver");
        return;
    }
    TAL_PR_DEBUG("TDD WS2812 driver initialized");
    
    // 创建主定时器
    tal_sw_timer_create(main_timer_cb, NULL, &led_ctrl.main_timer);
    
    TAL_PR_DEBUG("LED controller initialized");
    
    // 初始状态：上电自检
    set_led_state(LED_INIT, 0);
}

// 设置LED状态
void set_led_state(LedState new_state, uint8_t value) {
    TAL_PR_DEBUG("Setting LED state: %d, value: %d", new_state, value);
    
    // 上电自检独占处理：自检过程中接收的新状态将被缓存
    if (led_ctrl.current_state == LED_INIT && new_state != LED_INIT) {
        led_ctrl.pending_state = new_state;
        led_ctrl.pending_value = value;
        led_ctrl.has_pending_state = TRUE;
        TAL_PR_DEBUG("Init in progress, pending state: %d", new_state);
        return;
    }
    
    // 清理前一个状态
    cleanup_current_state();
    
    // 执行新状态
    switch (new_state) {
        case LED_INIT: // 上电自检（红->绿->蓝）
            set_all_leds(&COLOR_RED);
            led_ctrl.state_data.init.step = 0;
            tal_sw_timer_start(led_ctrl.main_timer, INIT_RED_TIME, TAL_TIMER_ONCE);
            break;
            
        case LED_IDLE: // 空闲状态（所有LED熄灭）
            set_all_leds(&COLOR_BLACK);
            break;
            
        case LED_CONFIGURING: // 配网中（绿灯闪烁：250ms亮/250ms灭）
            // 从亮开始闪烁
            set_all_leds(&COLOR_GREEN);
            led_ctrl.state_data.blink.is_light_on = TRUE;
            tal_sw_timer_start(led_ctrl.main_timer, CONFIGURING_BLINK_ON_TIME, TAL_TIMER_ONCE);
            break;
            
        case LED_CONFIG_SUCCESS: { // 配网成功（显示WIFI信号强度：步进展示）
            uint8_t target = value;
            if (target > WS2812_LED_COUNT) target = WS2812_LED_COUNT;
            led_ctrl.state_data.cfgsucc.target_level = target;
            led_ctrl.state_data.cfgsucc.current_count = 0;
            led_ctrl.state_data.cfgsucc.hold_phase = FALSE;
            if (target == 0) {
                // 直接进入保持阶段（保持0灯）
                set_level_leds(&COLOR_GREEN, 0);
                led_ctrl.state_data.cfgsucc.hold_phase = TRUE;
                tal_sw_timer_start(led_ctrl.main_timer, CONFIG_SUCCESS_HOLD_TIME, TAL_TIMER_ONCE);
            } else {
                // 立即点亮第1个并开始步进
                led_ctrl.state_data.cfgsucc.current_count = 1;
                set_level_leds(&COLOR_GREEN, 1);
                tal_sw_timer_start(led_ctrl.main_timer, CONFIG_SUCCESS_STEP_INTERVAL, TAL_TIMER_ONCE);
            }
            break;
        }
            
        case LED_NET_ERROR: // 网络异常（红灯常亮）
            set_all_leds(&COLOR_RED);
            break;
            
        case LED_DIALOG: // 对话中（蓝灯闪烁）
            set_all_leds(&COLOR_BLUE);
            led_ctrl.state_data.blink.is_light_on = TRUE;
            led_ctrl.state_data.blink.blink_count = 0;
            tal_sw_timer_start(led_ctrl.main_timer, DIALOG_LIGHT_ON_TIME, TAL_TIMER_ONCE);
            break;
            
        case LED_VOLUME: // 音量调节（黄灯等级显示）
            set_level_leds(&COLOR_YELLOW, value);
            tal_sw_timer_start(led_ctrl.main_timer, VOLUME_DISPLAY_TIMEOUT, TAL_TIMER_ONCE);
            break;
            
        case LED_BREATHING: // 呼吸灯效果（蓝灯呼吸）
            led_ctrl.state_data.breath.index = 0;
            tal_sw_timer_start(led_ctrl.main_timer, BREATH_TIMER_INTERVAL, TAL_TIMER_ONCE);
            break;
            
        case LED_WAKE: // 唤醒：蓝灯闪两下后常亮，12s后无对话关闭
            // 先点亮蓝色，开始计时闪烁
            set_all_leds(&COLOR_BLUE);
            led_ctrl.state_data.wake.in_solid_phase = FALSE;
            led_ctrl.state_data.wake.completed_toggles = 0;
            led_ctrl.state_data.wake.is_light_on = TRUE;
            tal_sw_timer_start(led_ctrl.main_timer, WAKE_BLINK_INTERVAL, TAL_TIMER_ONCE);
            break;
            
        case LED_STANDBY: // 待机：单个绿灯跑马
            // 先点亮顺序中的第一个
            led_ctrl.state_data.standby.index = 0;
            tdd_pixel_set_all(0, 0, 0);
            tdd_pixel_set_pixel(LED_ORDER[0], COLOR_GREEN.r, COLOR_GREEN.g, COLOR_GREEN.b);
            tdd_pixel_refresh();
            tal_sw_timer_start(led_ctrl.main_timer, STANDBY_STEP_INTERVAL, TAL_TIMER_ONCE);
            break;
    }
    
    // 更新当前状态
    led_ctrl.current_state = new_state;
}

// 去初始化LED控制器
void led_controller_deinit(void) {
    TAL_PR_DEBUG("Deinitializing LED controller");
    
    // 停止所有定时器
    if (led_ctrl.main_timer) {
        tal_sw_timer_stop(led_ctrl.main_timer);
        tal_sw_timer_delete(led_ctrl.main_timer);
        led_ctrl.main_timer = NULL;
    }
    
    // 关闭TDD驱动
    tdd_pixel_deinit();
    
    // 清零控制结构体
    memset(&led_ctrl, 0, sizeof(LedController));
    
    TAL_PR_DEBUG("LED controller deinitialized");
}