/**
 * @file ui.h — 倒立摆监控界面 (无撕裂版, 双页)
 */
#ifndef UI_H
#define UI_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 颜色 ========== */
#define C_BG          0x000000
#define C_ACCENT      0x00d4aa
#define C_RED         0xdc2626
#define C_GREEN       0x059669
#define C_GRAY        0x6b7280
#define C_LIGHT       0x9ca3af
#define C_WHITE       0xffffff
#define C_DOT_GREEN   0x22c55e
#define C_DOT_GRAY    0x374151
#define C_LINE        0x1f2937
#define C_TITLE       0xe2e8f0
#define C_SLIDER_BG   0x1f2937
#define C_SEND_BG     0x0f3460
#define C_SEND_PRESS  0x0a2548

/* ========== CAN 状态 ========== */
typedef enum { CAN_OK = 0, CAN_TIMEOUT = 1, CAN_OFF = 2 } can_status_t;

/* ========== 全局命令 ========== */
extern volatile int g_cmd_start;
extern volatile int g_cmd_stop;

/* ========== PID 全局变量 ========== */
extern volatile float g_pid_kp, g_pid_ki, g_pid_kd;

/* ========== API ========== */
void ui_init(void);

/* 页面切换 0=MONITOR 1=PID 2=INFO */
void page_switch(uint8_t idx);

/* MONITOR 页面更新 */
void ui_update_data(uint16_t angle, int16_t speed, int16_t location, uint8_t state);
void ui_update_gauge(uint16_t raw);
void ui_update_clock(void);
void ui_set_can_status(can_status_t st);

/* PID 页面: 读取当前 slider 值 */
void pid_get_values(float *kp, float *ki, float *kd);

/* INFO 页面: 系统信息更新 (1Hz) */
void sysinfo_update(uint32_t uptime_sec, uint8_t can_load_pct,
                    uint8_t wifi_clients, uint8_t fps,
                    uint32_t heap_free_kb, uint8_t core0_load, uint8_t core1_load);

#ifdef __cplusplus
}
#endif
#endif
