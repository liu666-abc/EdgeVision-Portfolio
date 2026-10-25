/**
 * @file ui.cpp — 无撕裂双页界面 (MONITOR + PID) — 中文版
 */
#include "ui.h"
#include "lvgl_port.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>

#define PI 3.14159265f

/* 中文字体声明 */
LV_FONT_DECLARE(my_chinese_font_14);
LV_FONT_DECLARE(my_chinese_font_16);

/* ========== 页面容器 ========== */
static lv_obj_t *page_monitor = NULL;
static lv_obj_t *page_pid     = NULL;
static lv_obj_t *page_info    = NULL;
static lv_obj_t *tab_labels[3] = {NULL};
static uint8_t  current_page = 0;

/* ========== MONITOR 页句柄 ========== */
static lv_obj_t *label_clock = NULL;
static lv_obj_t *dot_can = NULL, *label_can = NULL;
static lv_obj_t *val_angle = NULL, *val_speed = NULL, *val_loc = NULL, *val_state = NULL;
static lv_obj_t *gauge_line = NULL, *gauge_val = NULL;
static lv_point_t gauge_pts[2];
static lv_coord_t gauge_cx = 160, gauge_cy = 240, pointer_len = 100;

/* ========== PID 页句柄 ========== */
static lv_obj_t *slider_kp = NULL, *slider_ki = NULL, *slider_kd = NULL;
static lv_obj_t *label_kp = NULL, *label_ki = NULL, *label_kd = NULL;
static lv_obj_t *send_btn = NULL, *send_label = NULL;
static lv_timer_t *send_timer = NULL;

/* ========== 辅助 ========== */
static lv_obj_t *mk_label(lv_obj_t *p, const char *txt, lv_color_t c, const lv_font_t *f,
                           lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, c, 0);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_pos(l, x, y);
    return l;
}

extern volatile int g_cmd_start;
extern volatile int g_cmd_stop;
static void start_cb(lv_event_t *e) { g_cmd_start = 1; }
static void stop_cb(lv_event_t *e)  { g_cmd_stop  = 1; }

/* ========== 滑动切页 ========== */
static void swipe_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir == LV_DIR_LEFT  && current_page < 2) page_switch(current_page + 1);
    if (dir == LV_DIR_RIGHT && current_page > 0) page_switch(current_page - 1);
}

/* ========== 页面切换 ========== */
void page_switch(uint8_t idx)
{
    current_page = idx;
    if (page_monitor) { if (idx == 0) lv_obj_clear_flag(page_monitor, LV_OBJ_FLAG_HIDDEN);
                        else          lv_obj_add_flag(page_monitor, LV_OBJ_FLAG_HIDDEN); }
    if (page_pid)     { if (idx == 1) lv_obj_clear_flag(page_pid, LV_OBJ_FLAG_HIDDEN);
                        else          lv_obj_add_flag(page_pid, LV_OBJ_FLAG_HIDDEN); }
    if (page_info)    { if (idx == 2) lv_obj_clear_flag(page_info, LV_OBJ_FLAG_HIDDEN);
                        else          lv_obj_add_flag(page_info, LV_OBJ_FLAG_HIDDEN); }
    for (int i = 0; i < 3; i++) {
        if (tab_labels[i]) lv_obj_set_style_text_color(tab_labels[i],
            lv_color_hex(i == idx ? C_WHITE : C_GRAY), 0);
    }
}

/* ================================================================
 *  MONITOR 页 (page_monitor)
 * ================================================================ */
static void status_bar(lv_obj_t *scr)
{
    mk_label(scr, "WiFi热点", lv_color_hex(C_LIGHT), &my_chinese_font_14, 16, 8);
    label_clock = mk_label(scr, "00:00:00", lv_color_hex(C_WHITE), &my_chinese_font_16, 360, 8);
    dot_can = lv_obj_create(scr); lv_obj_remove_style_all(dot_can);
    lv_obj_set_size(dot_can, 8, 8); lv_obj_set_style_radius(dot_can, 4, 0);
    lv_obj_set_style_bg_color(dot_can, lv_color_hex(C_DOT_GREEN), 0);
    lv_obj_set_style_border_width(dot_can, 0, 0); lv_obj_set_pos(dot_can, 760, 12);
    label_can = mk_label(scr, "CAN正常", lv_color_hex(C_LIGHT), &my_chinese_font_14, 740, 6);
}

static void data_labels(lv_obj_t *p)
{
    const char *n[] = {"角度","速度","位置","状态"};
    lv_obj_t **v[]  = {&val_angle, &val_speed, &val_loc, &val_state};
    for (int i = 0; i < 4; i++) {
        lv_coord_t y = 60 + i * 72;
        mk_label(p, n[i], lv_color_hex(C_GRAY), &my_chinese_font_14, 20, y);
        *v[i] = mk_label(p, "0", lv_color_hex(C_ACCENT), &my_chinese_font_16, 20, y + 20);
    }
}

static void gauge_create(lv_obj_t *p)
{
    lv_obj_t *cont = lv_obj_create(p); lv_obj_remove_style_all(cont);
    lv_obj_set_pos(cont, 260, 60); lv_obj_set_size(cont, 320, 320);
    lv_obj_set_style_bg_opa(cont, 0, 0); lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *arc = lv_arc_create(cont);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB); lv_obj_remove_style(arc, NULL, LV_PART_INDICATOR);
    lv_obj_set_size(arc, 260, 260); lv_obj_align(arc, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_arc_color(arc, lv_color_hex(C_LINE), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, 0, 0); lv_obj_set_style_border_width(arc, 0, 0);
    lv_arc_set_bg_angles(arc, 2250, 3150); lv_arc_set_angles(arc, 2250, 3150);

    struct { int raw; const char *t; } ticks[] = {{0,"0"},{1024,"1K"},{2048,"2K"},{3072,"3K"},{4096,"4K"}};
    for (int i = 0; i < 5; i++) {
        float deg = 225.0f + (ticks[i].raw / 4096.0f) * 90.0f, rad = deg * PI / 180.0f;
        lv_coord_t tx = gauge_cx + (lv_coord_t)(cosf(rad) * 125);
        lv_coord_t ty = gauge_cy + (lv_coord_t)(sinf(rad) * 125);
        lv_coord_t ix = gauge_cx + (lv_coord_t)(cosf(rad) * 110);
        lv_coord_t iy = gauge_cy + (lv_coord_t)(sinf(rad) * 110);
        lv_obj_t *tk = lv_obj_create(cont); lv_obj_remove_style_all(tk);
        lv_obj_set_size(tk, 2, 2); lv_obj_set_pos(tk, ix - 1, iy - 1);
        lv_obj_set_style_bg_color(tk, lv_color_hex(C_GRAY), 0);
        mk_label(cont, ticks[i].t, lv_color_hex(C_GRAY), &my_chinese_font_14, tx - 8, ty - 8);
    }

    gauge_line = lv_line_create(cont);
    lv_obj_set_style_line_color(gauge_line, lv_color_hex(C_ACCENT), 0);
    lv_obj_set_style_line_width(gauge_line, 2, 0);
    gauge_pts[0].x = gauge_cx; gauge_pts[0].y = gauge_cy;
    gauge_pts[1].x = gauge_cx; gauge_pts[1].y = gauge_cy - pointer_len;
    lv_line_set_points(gauge_line, gauge_pts, 2);
    gauge_val = mk_label(cont, "2048", lv_color_hex(C_ACCENT), &my_chinese_font_16,
                         gauge_cx - 24, gauge_cy - pointer_len - 20);
}

static void buttons(lv_obj_t *p)
{
    lv_obj_t *bs = lv_btn_create(p); lv_obj_remove_style_all(bs);
    lv_obj_set_pos(bs, 610, 90); lv_obj_set_size(bs, 160, 80); lv_obj_set_style_radius(bs, 0, 0);
    lv_obj_set_style_bg_color(bs, lv_color_hex(C_GREEN), 0);
    lv_obj_set_style_border_color(bs, lv_color_hex(0x34d399), 0);
    lv_obj_set_style_border_width(bs, 3, 0);
    lv_obj_add_event_cb(bs, start_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *sl = lv_label_create(bs); lv_label_set_text(sl, "启动");
    lv_obj_set_style_text_color(sl, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_text_font(sl, &my_chinese_font_16, 0); lv_obj_center(sl);

    lv_obj_t *bp = lv_btn_create(p); lv_obj_remove_style_all(bp);
    lv_obj_set_pos(bp, 610, 200); lv_obj_set_size(bp, 160, 80); lv_obj_set_style_radius(bp, 0, 0);
    lv_obj_set_style_bg_color(bp, lv_color_hex(C_RED), 0);
    lv_obj_set_style_border_color(bp, lv_color_hex(0xf87171), 0);
    lv_obj_set_style_border_width(bp, 3, 0);
    lv_obj_add_event_cb(bp, stop_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *pl = lv_label_create(bp); lv_label_set_text(pl, "停止");
    lv_obj_set_style_text_color(pl, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_text_font(pl, &my_chinese_font_16, 0); lv_obj_center(pl);

}

/* ================================================================
 *  PID 页 (page_pid): x=0, y=30, w=800, h=420
 * ================================================================ */
static void pid_slider_event(lv_event_t *e);

static lv_obj_t *make_slider(lv_obj_t *p, lv_coord_t x, lv_coord_t y,
                              int32_t min, int32_t max, int32_t val)
{
    lv_obj_t *s = lv_slider_create(p);
    lv_obj_set_pos(s, x, y); lv_obj_set_size(s, 500, 20);
    lv_obj_set_style_radius(s, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 0, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 0, LV_PART_KNOB);
    lv_obj_set_style_bg_color(s, lv_color_hex(C_SLIDER_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, lv_color_hex(C_WHITE), LV_PART_KNOB);
    lv_obj_set_style_width(s, 12, LV_PART_KNOB);
    lv_obj_set_style_height(s, 24, LV_PART_KNOB);
    lv_obj_set_style_border_width(s, 0, LV_PART_KNOB);
    lv_slider_set_range(s, min, max);
    lv_slider_set_value(s, val, LV_ANIM_OFF);
    lv_obj_add_event_cb(s, pid_slider_event, LV_EVENT_VALUE_CHANGED, NULL);
    return s;
}

static void update_pid_labels(void)
{
    char b[16];
    int kv = slider_kp ? lv_slider_get_value(slider_kp) : 500;
    int iv = slider_ki ? lv_slider_get_value(slider_ki) : 5;
    int dv = slider_kd ? lv_slider_get_value(slider_kd) : 0;

    g_pid_kp = (float)kv / 1000.0f;
    g_pid_ki = (float)iv / 100.0f;
    g_pid_kd = (float)dv / 100.0f;

    if (label_kp) { snprintf(b, sizeof(b), "%1.3f", g_pid_kp); lv_label_set_text(label_kp, b); }
    if (label_ki) { snprintf(b, sizeof(b), "%1.2f", g_pid_ki); lv_label_set_text(label_ki, b); }
    if (label_kd) { snprintf(b, sizeof(b), "%1.2f", g_pid_kd); lv_label_set_text(label_kd, b); }
}

static void pid_slider_event(lv_event_t *e) { update_pid_labels(); }

static void send_reset_cb(lv_timer_t *t)
{
    if (send_label) { lv_label_set_text(send_label, "发送");
                      lv_obj_set_style_text_color(send_label, lv_color_hex(C_WHITE), 0); }
    send_timer = NULL;
}

static void send_btn_cb(lv_event_t *e)
{
    if (send_label) { lv_label_set_text(send_label, "已发送");
                      lv_obj_set_style_text_color(send_label, lv_color_hex(C_ACCENT), 0); }
    if (!send_timer) send_timer = lv_timer_create(send_reset_cb, 500, NULL);
    else lv_timer_reset(send_timer);
    /* CAN 发送在 main.cpp 的 can_send_pid_frame 中完成 */
    extern void can_send_pid_frame(void);
    can_send_pid_frame();
}

static void pid_page_init(lv_obj_t *scr)
{
    page_pid = lv_obj_create(scr);
    lv_obj_remove_style_all(page_pid);
    lv_obj_set_pos(page_pid, 0, 30); lv_obj_set_size(page_pid, 800, 420);
    lv_obj_set_style_bg_opa(page_pid, 0, 0);
    lv_obj_set_style_border_width(page_pid, 0, 0);
    lv_obj_clear_flag(page_pid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page_pid, LV_OBJ_FLAG_HIDDEN); /* 初始隐藏 */
    lv_obj_add_flag(page_pid, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(page_pid, swipe_cb, LV_EVENT_GESTURE, NULL);

    /* 标题 */
    mk_label(page_pid, "PID 参数", lv_color_hex(C_TITLE), &my_chinese_font_16, 20, 20);

    /* Kp */
    mk_label(page_pid, "Kp", lv_color_hex(C_GRAY), &my_chinese_font_16, 40, 80);
    label_kp = mk_label(page_pid, "0.500", lv_color_hex(C_ACCENT), &my_chinese_font_16, 120, 80);
    slider_kp = make_slider(page_pid, 260, 80, 0, 1000, 500);

    /* Ki */
    mk_label(page_pid, "Ki", lv_color_hex(C_GRAY), &my_chinese_font_16, 40, 170);
    label_ki = mk_label(page_pid, "0.05", lv_color_hex(C_ACCENT), &my_chinese_font_16, 120, 170);
    slider_ki = make_slider(page_pid, 260, 170, 0, 100, 5);

    /* Kd */
    mk_label(page_pid, "Kd", lv_color_hex(C_GRAY), &my_chinese_font_16, 40, 260);
    label_kd = mk_label(page_pid, "0.00", lv_color_hex(C_ACCENT), &my_chinese_font_16, 120, 260);
    slider_kd = make_slider(page_pid, 260, 260, 0, 1000, 0);

    /* SEND 按钮 */
    send_btn = lv_btn_create(page_pid); lv_obj_remove_style_all(send_btn);
    lv_obj_set_pos(send_btn, 340, 360); lv_obj_set_size(send_btn, 120, 50);
    lv_obj_set_style_radius(send_btn, 0, 0);
    lv_obj_set_style_bg_color(send_btn, lv_color_hex(C_SEND_BG), 0);
    lv_obj_set_style_border_width(send_btn, 0, 0);
    lv_obj_add_event_cb(send_btn, send_btn_cb, LV_EVENT_CLICKED, NULL);
    send_label = lv_label_create(send_btn);
    lv_label_set_text(send_label, "发送");
    lv_obj_set_style_text_color(send_label, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_text_font(send_label, &my_chinese_font_16, 0);
    lv_obj_center(send_label);
}

/* ================================================================
 *  INFO 页 (page_info): 系统信息
 * ================================================================ */
static lv_obj_t *info_uptime = NULL, *info_can = NULL, *info_wifi = NULL, *info_fps = NULL;
static lv_obj_t *info_ver = NULL, *info_heap = NULL, *info_c0 = NULL, *info_c1 = NULL;
static lv_obj_t *bar_can = NULL, *bar_c0 = NULL, *bar_c1 = NULL;

static lv_obj_t *make_bar(lv_obj_t *p, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t *b = lv_bar_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_pos(b, x, y); lv_obj_set_size(b, 160, 6);
    lv_obj_set_style_radius(b, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(b, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_LINE), LV_PART_MAIN);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_border_width(b, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(b, 0, LV_PART_INDICATOR);
    lv_bar_set_range(b, 0, 100);
    lv_bar_set_value(b, 0, LV_ANIM_OFF);
    return b;
}

static void info_page_init(lv_obj_t *scr)
{
    page_info = lv_obj_create(scr);
    lv_obj_remove_style_all(page_info);
    lv_obj_set_pos(page_info, 0, 30); lv_obj_set_size(page_info, 800, 420);
    lv_obj_set_style_bg_opa(page_info, 0, 0);
    lv_obj_set_style_border_width(page_info, 0, 0);
    lv_obj_clear_flag(page_info, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page_info, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(page_info, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(page_info, swipe_cb, LV_EVENT_GESTURE, NULL);

    mk_label(page_info, "系统信息", lv_color_hex(C_TITLE), &my_chinese_font_16, 20, 20);

    /* 分隔线 */
    lv_obj_t *sep = lv_obj_create(page_info);
    lv_obj_remove_style_all(sep);
    lv_obj_set_size(sep, 720, 1); lv_obj_set_pos(sep, 40, 48);
    lv_obj_set_style_bg_color(sep, lv_color_hex(C_LINE), 0);

    /* 8 行标签 + 数值, y=70起, 间距36 */
    struct { const char *tag; lv_obj_t **val; lv_obj_t **bar; int y; } rows[] = {
        {"运行时间:",     &info_uptime, NULL,    70},
        {"CAN 负载:",     &info_can,    &bar_can, 106},
        {"WiFi 客户端:",  &info_wifi,   NULL,    142},
        {"帧率:",         &info_fps,    NULL,    178},
        {"版本:",         &info_ver,    NULL,    214},
        {"可用内存:",     &info_heap,   NULL,    250},
        {"核心0 负载:",   &info_c0,     &bar_c0, 286},
        {"核心1 负载:",   &info_c1,     &bar_c1, 322},
    };
    for (int i = 0; i < 8; i++) {
        mk_label(page_info, rows[i].tag, lv_color_hex(C_GRAY), &my_chinese_font_16, 40, rows[i].y);
        lv_obj_t *v = mk_label(page_info, i==4?"v1.0.0":"--", lv_color_hex(C_ACCENT),
                                &my_chinese_font_16, 200, rows[i].y);
        *rows[i].val = v;
        if (rows[i].bar) {
            *rows[i].bar = make_bar(page_info, 360, rows[i].y + 8);
        }
    }
}

void sysinfo_update(uint32_t uptime_sec, uint8_t can_load, uint8_t wifi_clients,
                    uint8_t fps, uint32_t heap_kb, uint8_t c0, uint8_t c1)
{
    char b[32];
    if (info_uptime) { snprintf(b,sizeof(b),"%02d:%02d:%02d",
        (int)(uptime_sec/3600),(int)((uptime_sec/60)%60),(int)(uptime_sec%60));
        lv_label_set_text(info_uptime, b); }
    if (info_can)  { snprintf(b,sizeof(b),"%d%%",can_load); lv_label_set_text(info_can,b); }
    if (info_wifi) { snprintf(b,sizeof(b),"%d",wifi_clients); lv_label_set_text(info_wifi,b); }
    if (info_fps)  { snprintf(b,sizeof(b),"%d",fps); lv_label_set_text(info_fps,b); }
    if (info_heap) { snprintf(b,sizeof(b),"%lu KB",heap_kb); lv_label_set_text(info_heap,b); }
    if (info_c0)   { snprintf(b,sizeof(b),"%d%%",c0); lv_label_set_text(info_c0,b); }
    if (info_c1)   { snprintf(b,sizeof(b),"%d%%",c1); lv_label_set_text(info_c1,b); }
    if (bar_can)   lv_bar_set_value(bar_can, can_load, LV_ANIM_OFF);
    if (bar_c0)    lv_bar_set_value(bar_c0, c0, LV_ANIM_OFF);
    if (bar_c1)    lv_bar_set_value(bar_c1, c1, LV_ANIM_OFF);
}

/* ================================================================
 *  底部 Tab
 * ================================================================ */
static void tab_click_cb(lv_event_t *e) {
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    page_switch(idx);
}
static void bottom_tabs(lv_obj_t *scr)
{
    /* 独立容器, 纯黑底, LVGL 系统顶层 (永不覆盖) */
    lv_obj_t *bar = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(bar);
    lv_obj_set_pos(bar, 0, 448); lv_obj_set_size(bar, 800, 32);
    lv_obj_set_style_bg_color(bar, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bar, 0, 0);

    const char *t[] = {"监控", "PID", "信息"};
    for (int i = 0; i < 3; i++) {
        lv_obj_t *btn = lv_btn_create(bar);
        lv_obj_remove_style_all(btn);
        lv_obj_set_size(btn, 120, 28);
        lv_obj_set_pos(btn, (lv_coord_t)(200 + i * 160), 2);
        lv_obj_set_style_radius(btn, 0, 0);
        lv_obj_set_style_bg_opa(btn, 0, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_shadow_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, tab_click_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);

        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, t[i]);
        lv_obj_set_style_text_color(l, lv_color_hex(i == 0 ? C_WHITE : C_GRAY), 0);
        lv_obj_set_style_text_font(l, &my_chinese_font_14, 0);
        lv_obj_center(l);
        tab_labels[i] = l;
    }
}

/* ================================================================
 *  公开 API
 * ================================================================ */
void ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(scr, swipe_cb, LV_EVENT_GESTURE, NULL);

    /* MONITOR 页 */
    page_monitor = lv_obj_create(scr); lv_obj_remove_style_all(page_monitor);
    lv_obj_set_size(page_monitor, 800, 448);  /* 留出底部32px给Tab栏 */
    lv_obj_set_style_bg_opa(page_monitor, 0, 0);
    lv_obj_set_style_border_width(page_monitor, 0, 0);
    lv_obj_clear_flag(page_monitor, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_flag(page_monitor, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(page_monitor, swipe_cb, LV_EVENT_GESTURE, NULL);
    status_bar(page_monitor);
    data_labels(page_monitor);
    gauge_create(page_monitor);
    buttons(page_monitor);

    /* PID 页 */
    pid_page_init(scr);

    /* INFO 页 */
    info_page_init(scr);

    /* 底部 Tab */
    bottom_tabs(scr);
}

void ui_update_data(uint16_t angle, int16_t speed, int16_t location, uint8_t state)
{
    char b[16];
    static uint16_t la = 0xFFFF;
    static int16_t ls = 0x7FFF, ll = 0x7FFF;
    static uint8_t lst = 0xFF;

    if (val_angle && angle != la)
        { la = angle; snprintf(b,sizeof(b),"%u",la); lv_label_set_text(val_angle,b); }
    if (val_speed && speed != ls)
        { ls = speed; snprintf(b,sizeof(b),"%d",ls); lv_label_set_text(val_speed,b); }
    if (val_loc && location != ll)
        { ll = location; snprintf(b,sizeof(b),"%d",ll); lv_label_set_text(val_loc,b); }
    if (val_state && state != lst)
        { lst = state; snprintf(b,sizeof(b),"%d",lst); lv_label_set_text(val_state,b); }
}

void ui_update_gauge(uint16_t raw)
{
    if (!gauge_line) return;
    float deg = 225.0f + ((float)raw / 4095.0f) * 90.0f, rad = deg * PI / 180.0f;
    lv_coord_t ex = gauge_cx + (lv_coord_t)(cosf(rad) * pointer_len);
    lv_coord_t ey = gauge_cy + (lv_coord_t)(sinf(rad) * pointer_len);
    gauge_pts[0].x = gauge_cx; gauge_pts[0].y = gauge_cy;
    gauge_pts[1].x = ex;       gauge_pts[1].y = ey;
    lv_line_set_points(gauge_line, gauge_pts, 2);
    if (gauge_val) { char b[8]; snprintf(b,sizeof(b),"%u",raw); lv_label_set_text(gauge_val, b); }
}

void ui_update_clock(void)
{
    if (!label_clock) return;
    int64_t s = esp_timer_get_time() / 1000000;
    char b[16]; snprintf(b,sizeof(b),"%02d:%02d:%02d",(int)((s/3600)%24),(int)((s/60)%60),(int)(s%60));
    lv_label_set_text(label_clock, b);
}

void ui_set_can_status(can_status_t st)
{
    if (!dot_can || !label_can) return;
    switch (st) {
    case CAN_OK: lv_obj_set_style_bg_color(dot_can, lv_color_hex(C_DOT_GREEN), 0);
                 lv_label_set_text(label_can, "CAN正常"); break;
    case CAN_TIMEOUT: lv_obj_set_style_bg_color(dot_can, lv_color_hex(C_DOT_GRAY), 0);
                      lv_label_set_text(label_can, "CAN异常"); break;
    default: lv_obj_set_style_bg_color(dot_can, lv_color_hex(C_DOT_GRAY), 0);
             lv_label_set_text(label_can, "CAN离线"); break;
    }
}

void pid_get_values(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_pid_kp;
    if (ki) *ki = g_pid_ki;
    if (kd) *kd = g_pid_kd;
}
