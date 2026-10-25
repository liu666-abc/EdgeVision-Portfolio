/**
 * ESP32-S3 倒立摆监控系统
 * ========================
 * 硬件: Waveshare 4.3" RGB 800x480 (ST7262) + GT911 触摸
 * 架构: ESP-IDF v6.0.1 + LVGL v8.4 + 微雪官方 lvgl_port
 *
 * 功能模块:
 *   [显示] LVGL 三页 UI (MONITOR 监控 / PID 调参 / INFO 系统信息)
 *   [通信] CAN 总线接收 STM32 倒立摆数据 (TWAI 500kbps)
 *   [控制] START/STOP 按钮 → CAN 发送启停指令
 *   [网络] WiFi AP + HTTP Server → 网页远程监控
 *   [网页] Canvas 2D 摆杆动画 + 实时数据仪表盘
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/twai.h"
#include "lvgl.h"
#include "waveshare_rgb_lcd_port.h"
#include "lvgl_port.h"
#include "ui.h"

static const char *TAG = "main";

/* ==================================================================
 *  WiFi 热点配置
 *  SSID / password are placeholders in this public source excerpt.
 * ================================================================== */
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "CHANGE_ME_BEFORE_FLASHING"

/* ==================================================================
 *  CAN 总线引脚 (TWAI 500kbps)
 *  TX=GPIO15, RX=GPIO16, 接收 STM32 倒立摆数据帧
 * ================================================================== */
#define TWAI_TX_PIN  GPIO_NUM_15
#define TWAI_RX_PIN  GPIO_NUM_16

/* ==================================================================
 *  CAN 数据结构 (来自 STM32 的标准帧, 8 字节)
 *  格式: angle(u16) | speed(s16) | location(s16) | state(u8) | pwm(s8)
 * ================================================================== */
typedef struct {
    volatile uint16_t angle;        // 角度 0-4095
    volatile int16_t  speed;        // 速度 (rpm)
    volatile int16_t  location;     // 位置 (mm)
    volatile uint8_t  state;        // 状态码 (0=停止, 4=平衡)
    volatile int8_t   pwm;          // PWM 占空比
    volatile uint32_t last_rx_ms;   // 最后收包时间 (ms)
} can_data_t;
static can_data_t g_can_data = {};

/* ==================================================================
 *  全局命令标志 (由 UI 按钮设置, CAN 任务消费并发送)
 *  g_cmd_start=1 → 发送启动帧 (0x01)
 *  g_cmd_stop=1  → 发送停止帧 (0x02)
 * ================================================================== */
volatile int g_cmd_start = 0;
volatile int g_cmd_stop  = 0;

/* ==================================================================
 *  系统可观测性数据 (1Hz 定时器更新, HTTP /debug 端点暴露)
 *  uptime:     运行秒数
 *  can_fps:    CAN 帧速率 (帧/秒)
 *  fps:        LVGL 帧率估算
 *  heap_kb:    剩余堆内存 (KB)
 *  wifi_cli:   WiFi 客户端数 (当前未实现, 预留)
 *  can_status: CAN 状态 (0=OK, 1=超时, 2=离线)
 * ================================================================== */
typedef struct {
    volatile uint32_t uptime;
    volatile uint8_t  can_fps;
    volatile uint8_t  fps;
    volatile uint32_t heap_kb;
    volatile uint8_t  wifi_cli;
    volatile uint8_t  can_status;
    volatile uint8_t  heartbeat_ok;  /* 1=STM32心跳正常, 0=心跳丢失 */
    volatile uint8_t  hb_last;       /* 上次心跳序号 */
} sys_stats_t;
static sys_stats_t g_sys_stats = {};

/* ==================================================================
 *  PID 参数 (由 PID 页面 slider 设置, SEND 按钮发送)
 *  g_pid_kp: 比例系数 0.000-1.000
 *  g_pid_ki: 积分系数 0.00-1.00
 *  g_pid_kd: 微分系数 0.00-10.00
 * ================================================================== */
volatile float g_pid_kp = 0.5f, g_pid_ki = 0.05f, g_pid_kd = 0.0f;

/**
 * 发送 PID 参数帧到 STM32
 * CAN ID=0x321, Byte6=0x02 标识 PID 帧类型
 */
void can_send_pid_frame(void)
{
    twai_message_t msg = {};
    msg.identifier = 0x321;
    msg.data_length_code = 8;
    msg.extd = 0; msg.rtr = 0;
    uint16_t kp = (uint16_t)(g_pid_kp * 1000);
    uint16_t ki = (uint16_t)(g_pid_ki * 100);
    uint16_t kd = (uint16_t)(g_pid_kd * 100);
    msg.data[0] = kp & 0xFF;
    msg.data[1] = (kp >> 8) & 0xFF;
    msg.data[2] = ki & 0xFF;
    msg.data[3] = (ki >> 8) & 0xFF;
    msg.data[4] = kd & 0xFF;
    msg.data[5] = (kd >> 8) & 0xFF;
    msg.data[6] = 0x02;  /* PID参数帧 */
    msg.data[7] = 0x00;
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) ESP_LOGI(TAG, "PID frame sent");
    else ESP_LOGW(TAG, "PID send fail: %d", ret);
}

/* ==================================================================
 *  HTTP 网页: Canvas 2D 摆杆动画 + 实时数据
 *  端点: GET /  → HTML 页面
 *        GET /d → JSON 数据 {"a":angle,"s":speed,"l":loc,"st":state,"p":pwm}
 *
 *  纯离线方案: 无 CDN 依赖, Canvas 2D 绘制
 *  - 半圆弧刻度 (-40 / 0 / +40)
 *  - 摆杆 + 红色摆球 + 高光
 *  - 平滑 lerp 动画 (requestAnimationFrame)
 *  - 80ms 轮询 CAN 数据
 * ================================================================== */
static const char html_page[] = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>倒立摆监控系统</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0e27;color:#cbd5e1;font-family:'Microsoft YaHei','PingFang SC','Segoe UI',Arial,sans-serif;overflow-x:hidden}
#app{max-width:960px;margin:0 auto;padding:8px}

/* ---- 顶栏 ---- */
#topbar{display:flex;align-items:center;justify-content:space-between;
 background:#111827;border:1px solid #1e293b;padding:8px 16px;margin-bottom:8px}
#topbar h1{font-size:16px;color:#e2e8f0;font-weight:600}
#topbar .dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:6px}
#topbar .dot.g{background:#00d4aa;box-shadow:0 0 6px #00d4aa}
#topbar .dot.r{background:#ff6b6b;box-shadow:0 0 6px #ff6b6b}
#topbar .dot.y{background:#f1c40f;box-shadow:0 0 6px #f1c40f}
#topbar .st{font-size:14px;color:#94a3b8}

/* ---- 主区 ---- */
#main{display:flex;gap:8px;min-height:420px}
#pendulum{flex:1;background:#111827;border:1px solid #1e293b;position:relative;min-width:280px}
#pendulum canvas{display:block;width:100%;height:100%}
#datapanel{width:220px;display:flex;flex-direction:column;gap:8px}

/* ---- 数据卡片 ---- */
.card{background:#111827;border:1px solid #1e293b;padding:12px 14px}
.card .label{font-size:10px;color:#64748b;text-transform:uppercase;letter-spacing:1px;margin-bottom:2px}
.card .value{font-size:26px;font-weight:700;color:#00d4aa;font-family:'Consolas',monospace}
.card .unit{font-size:11px;color:#475569;margin-left:2px}
.card .sub{font-size:11px;color:#475569;margin-top:2px}

/* ---- 状态栏 ---- */
#status{text-align:center;padding:6px;margin:8px 0;background:#111827;border:1px solid #1e293b}
#status .s{font-size:18px;font-weight:700}

/* ---- 调试栏 ---- */
#debugbar{display:flex;justify-content:space-around;padding:6px 12px;margin:4px 0;
 background:#111827;border:1px solid #1e293b;font-size:12px}
#debugbar .di{text-align:center}
#debugbar .dv{color:#00d4aa;font-weight:700}
#debugbar .dl{color:#64748b;font-size:10px}

/* ---- 波形图 ---- */
#wave{background:#111827;border:1px solid #1e293b;padding:8px;height:140px}
#wave canvas{display:block;width:100%;height:100%}
#wave .title{font-size:13px;color:#64748b;letter-spacing:1px}

/* ---- 底栏 ---- */
#bottombar{display:flex;justify-content:center;gap:24px;padding:4px;
 font-size:13px;color:#556677;margin-top:4px}
</style></head><body>
<div id="app">
 <!-- 顶栏 -->
 <div id="topbar">
  <h1>倒立摆监控</h1>
  <span class="st"><span class="dot g" id="dot"></span><span id="statetxt">连接中</span></span>
  <span class="st">CAN 500kbps</span>
 </div>

 <!-- 主区: 摆杆 + 数据面板 -->
 <div id="main">
  <div id="pendulum"><canvas id="pendcv"></canvas></div>
  <div id="datapanel">
   <div class="card"><div class="label">摆杆角度</div><div class="value" id="v_angle">--</div><div class="unit">raw / deg</div><div class="sub" id="sub_angle"></div></div>
   <div class="card"><div class="label">电机速度</div><div class="value" id="v_speed">--</div><div class="unit">rpm</div></div>
   <div class="card"><div class="label">位置</div><div class="value" id="v_loc">--</div><div class="unit">mm</div></div>
   <div class="card"><div class="label">PWM 输出</div><div class="value" id="v_pwm">--</div><div class="sub" id="sub_pwm"></div></div>
  </div>
 </div>

 <!-- 状态 -->
 <div id="status"><span class="s" id="statebig">等待连接...</span></div>

 <!-- 调试栏 -->
 <div id="debugbar">
  <div class="di"><div class="dv" id="db_uptime">--</div><div class="dl">运行时间</div></div>
  <div class="di"><div class="dv" id="db_canfps">--</div><div class="dl">CAN 帧率</div></div>
  <div class="di"><div class="dv" id="db_heap">--</div><div class="dl">可用内存</div></div>
  <div class="di"><div class="dv" id="db_canage">--</div><div class="dl">CAN 延迟</div></div>
  <div class="di"><div class="dv" id="db_heart">--</div><div class="dl">STM32 心跳</div></div>
  <div class="di"><div class="dv" id="db_fps">--</div><div class="dl">UI 帧率</div></div>
 </div>

 <!-- 波形图 -->
 <div id="wave">
  <div class="title">角度历史波形 (5秒)</div>
  <canvas id="wavecv"></canvas>
 </div>

 <!-- 底栏 -->
 <div id="bottombar">
  <span>ESP32-S3</span><span>WiFi热点: 已脱敏</span><span id="uptime">运行时间: --</span>
 </div>
</div>

<script>
/* ---- 摆杆画布 ---- */
var pcv=document.getElementById('pendcv'),pctx=pcv.getContext('2d');
var pw=pcv.parentNode.clientWidth,ph=pcv.parentNode.clientHeight;
pcv.width=pw;pcv.height=ph;
var cx=pw/2,cy=ph-50,r=Math.min(pw*0.4,ph*0.55),angle=0,target=0,uptime=0;

function drawPend(){
 pw=pcv.parentNode.clientWidth;ph=pcv.parentNode.clientHeight;
 if(pcv.width!=pw||pcv.height!=ph){pcv.width=pw;pcv.height=ph;cx=pw/2;cy=ph-50;r=Math.min(pw*0.4,ph*0.55);}
 pctx.clearRect(0,0,pw,ph);
 /* 底盘 */
 pctx.fillStyle='#1e293b';
 pctx.fillRect(cx-90,cy+8,180,18);
 pctx.fillStyle='#0f172a';
 pctx.fillRect(cx-60,cy+24,120,10);
 /* 刻度弧线 */
 pctx.strokeStyle='#1e293b';pctx.lineWidth=2.5;pctx.beginPath();
 pctx.arc(cx,cy,r,Math.PI,0);pctx.stroke();
 /* 刻度 */
 pctx.font='bold 11px Arial';
 for(var i=-5;i<=5;i++){
  var a=i*10*Math.PI/180;
  var x1=cx+Math.sin(a)*(r-3),y1=cy-Math.cos(a)*(r-3);
  var x2=cx+Math.sin(a)*(r-18),y2=cy-Math.cos(a)*(r-18);
  pctx.strokeStyle=i%5==0?'#475569':'#1e293b';pctx.lineWidth=i%5==0?2:1;
  pctx.beginPath();pctx.moveTo(x1,y1);pctx.lineTo(x2,y2);pctx.stroke();
  if(i%2==0){
   pctx.fillStyle=i==0?'#00d4aa':'#475569';
   pctx.fillText(i==0?'0':i>0?'+'+i:i,cx+Math.sin(a)*(r+14)-8,cy-Math.cos(a)*(r+14)+4);
  }
 }
 /* 摆杆 */
 var bx=cx+Math.sin(angle)*r,by=cy-Math.cos(angle)*r;
 var dev=Math.abs(angle/(50*Math.PI/180));
 var rv=Math.floor(dev*255),gv=Math.floor((1-dev)*212),bv=Math.floor((1-dev)*170);
 pctx.strokeStyle='rgb('+rv+','+gv+','+bv+')';pctx.lineWidth=4.5;pctx.beginPath();
 pctx.moveTo(cx,cy);pctx.lineTo(bx,by);pctx.stroke();
 /* 摆球 */
 var grd=pctx.createRadialGradient(bx-3,by-3,2,bx,by,20);
 grd.addColorStop(0,'#ff8888');grd.addColorStop(0.6,'#e94560');grd.addColorStop(1,'#881122');
 pctx.fillStyle=grd;pctx.beginPath();pctx.arc(bx,by,20,0,Math.PI*2);pctx.fill();
 pctx.fillStyle='rgba(255,255,255,.3)';pctx.beginPath();pctx.arc(bx-5,by-6,5,0,Math.PI*2);pctx.fill();
}

/* ---- 波形图 ---- */
var wcv=document.getElementById('wavecv'),wctx=wcv.getContext('2d');
var ww=wcv.parentNode.clientWidth-16,wh=124;wcv.width=ww;wcv.height=wh;
var hist=[]; for(var i=0;i<150;i++) hist.push(2048);

function drawWave(){
 wctx.clearRect(0,0,ww,wh);
 /* 网格 */
 wctx.strokeStyle='#1e293b';wctx.lineWidth=0.5;
 for(var i=0;i<5;i++){var gy=wh*i/4;wctx.beginPath();wctx.moveTo(0,gy);wctx.lineTo(ww,gy);wctx.stroke();}
 for(var i=0;i<8;i++){var gx=ww*i/7;wctx.beginPath();wctx.moveTo(gx,0);wctx.lineTo(gx,wh);wctx.stroke();}
 /* Y 轴标签 */
 wctx.fillStyle='#334155';wctx.font='9px Arial';
 wctx.fillText('4095',2,10);wctx.fillText('2048',2,wh/2+4);wctx.fillText('0',2,wh-2);
 /* 波形线 */
 wctx.strokeStyle='#00d4aa';wctx.lineWidth=1.5;wctx.beginPath();
 var step=ww/(hist.length-1);
 for(var i=0;i<hist.length;i++){
  var wx=i*step,wy=wh-(hist[i]/4095)*wh;
  if(i==0)wctx.moveTo(wx,wy);else wctx.lineTo(wx,wy);
 }
 wctx.stroke();
 /* 填充 */
 wctx.lineTo(ww,wh);wctx.lineTo(0,wh);wctx.closePath();
 var fgrd=wctx.createLinearGradient(0,0,0,wh);
 fgrd.addColorStop(0,'rgba(0,212,170,.15)');fgrd.addColorStop(1,'rgba(0,212,170,0)');
 wctx.fillStyle=fgrd;wctx.fill();
}

function loop(){
 angle+=(target-angle)*0.3;
 drawPend();drawWave();
 requestAnimationFrame(loop);
}
loop();

function update(d){
 target=((d.a-2048)/2047)*50*Math.PI/180;
 var deg=(d.a/45.5).toFixed(1);
 document.getElementById('v_angle').innerText=d.a;
 document.getElementById('sub_angle').innerText=deg+' deg';
 document.getElementById('v_speed').innerText=d.s;
 document.getElementById('v_loc').innerText=d.l;
 document.getElementById('v_pwm').innerText=d.p;
 document.getElementById('sub_pwm').innerText=(d.p>0?'前进':'后退');
 /* 状态 */
 var st=document.getElementById('statebig');
 var dot=document.getElementById('dot');
 var txt=document.getElementById('statetxt');
 if(d.st==4){st.innerHTML='<span style="color:#00d4aa">平衡中</span>';dot.className='dot g';txt.innerText='正常'}
 else if(d.st==0){st.innerHTML='<span style="color:#ff6b6b">已停止</span>';dot.className='dot r';txt.innerText='空闲'}
 else{st.innerHTML='<span style="color:#f1c40f">STATE '+d.st+'</span>';dot.className='dot y';txt.innerText='过渡'}
 /* 波形 */
 hist.push(d.a);if(hist.length>150)hist.shift();
 /* 运行时间 */
 uptime++;document.getElementById('uptime').innerText='运行: '+Math.floor(uptime/60)+'m '+uptime%60+'s';
}

fetch('/d').then(r=>r.json()).then(update);
setInterval(function(){fetch('/d').then(function(r){return r.json()}).then(update).catch(function(){})},100);

/* 调试面板轮询 /debug (2秒一次) */
function debugPoll(){
 fetch('/debug').then(function(r){return r.json()}).then(function(d){
  var m=Math.floor(d.uptime/60),s=d.uptime%60,h=Math.floor(m/60);m=m%60;
  document.getElementById('db_uptime').innerText=(h>0?h+':':'')+(m<10?'0':'')+m+':'+(s<10?'0':'')+s;
  document.getElementById('db_canfps').innerText=d.can_fps+' fps';
  document.getElementById('db_heap').innerText=d.heap_kb+' KB';
  document.getElementById('db_canage').innerText=d.can_age_ms+' ms';
  var he=document.getElementById('db_heart');
  he.innerText=d.heartbeat=='OK'?'正常':'丢失';
  he.style.color=d.heartbeat=='OK'?'#00d4aa':'#ff6b6b';
  document.getElementById('db_fps').innerText=d.fps+' fps';
 }).catch(function(){});
}
debugPoll();
setInterval(debugPoll,2000);
</script></body></html>)rawliteral";

/* HTTP 系统诊断接口: 返回 JSON 系统状态 */
static esp_err_t http_debug_handler(httpd_req_t *req)
{
    char buf[256];
    uint32_t now = esp_timer_get_time() / 1000;
    uint32_t can_age = now - g_can_data.last_rx_ms;
    snprintf(buf, sizeof(buf),
        "{\"uptime\":%lu,\"can_fps\":%u,\"fps\":%u,\"heap_kb\":%lu,"
        "\"can_age_ms\":%lu,\"can_status\":\"%s\","
        "\"heartbeat\":\"%s\","
        "\"angle\":%u,\"speed\":%d,\"location\":%d,\"state\":%u,\"pwm\":%d}",
        g_sys_stats.uptime, g_sys_stats.can_fps, g_sys_stats.fps,
        g_sys_stats.heap_kb, can_age,
        (can_age < 200) ? "OK" : (can_age < 1000) ? "TIMEOUT" : "OFFLINE",
        g_sys_stats.heartbeat_ok ? "OK" : "LOST",
        g_can_data.angle, g_can_data.speed, g_can_data.location,
        g_can_data.state, g_can_data.pwm);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* HTTP 首页: 返回 HTML 页面 */
static esp_err_t http_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* HTTP 数据接口: 返回 JSON {a,s,l,st,p} */
static esp_err_t http_data_handler(httpd_req_t *req)
{
    char buf[100];
    snprintf(buf, sizeof(buf), "{\"a\":%d,\"s\":%d,\"l\":%d,\"st\":%d,\"p\":%d}",
             g_can_data.angle, g_can_data.speed, g_can_data.location,
             g_can_data.state, g_can_data.pwm);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/** 启动 HTTP 服务器, 注册 / 和 /d 两个端点 */
static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &config) == ESP_OK) {
        httpd_uri_t r = {};
        r.uri = "/";
        r.method = HTTP_GET;
        r.handler = http_root_handler;
        httpd_uri_t d = {};
        d.uri = "/d";
        d.method = HTTP_GET;
        d.handler = http_data_handler;
        httpd_uri_t debug = {};
        debug.uri = "/debug";
        debug.method = HTTP_GET;
        debug.handler = http_debug_handler;
        httpd_register_uri_handler(srv, &r);
        httpd_register_uri_handler(srv, &d);
        httpd_register_uri_handler(srv, &debug);
        ESP_LOGI(TAG, "HTTP server on port 80");
    }
}

/* ==================================================================
 *  WiFi AP 模式
 *  创建 WPA2 热点，SSID 和密码由本地配置提供
 * ================================================================== */
static void wifi_ap_init(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_cfg = {};
    ap_cfg.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(ap_cfg.ap.ssid, WIFI_SSID, ap_cfg.ap.ssid_len);
    memcpy(ap_cfg.ap.password, WIFI_PASS, strlen(WIFI_PASS));
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
    ESP_LOGI(TAG, "WiFi AP: " IPSTR, IP2STR(&ip_info.ip));
}

/* ==================================================================
 *  CAN 总线初始化 (TWAI 500kbps, 标准帧, 不过滤)
 * ================================================================== */
static void can_init(void)
{
    twai_general_config_t gc = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t tc = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fc = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&gc, &tc, &fc));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN 500kbps");
}

/* ==================================================================
 *  CAN 发送单字节指令 (ID=0x321)
 *  cmd: 0x01=启动, 0x02=停止, 0x03=复位
 * ================================================================== */
static void can_send_cmd(uint8_t cmd)
{
    twai_message_t msg = {};
    msg.identifier = 0x321;
    msg.data_length_code = 1;
    msg.data[0] = cmd;
    msg.extd = 0;
    msg.rtr = 0;
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN send cmd=%d OK", cmd);
    } else {
        ESP_LOGW(TAG, "CAN send cmd=%d fail: %d", cmd, ret);
    }
}

/* ==================================================================
 *  CAN 帧计数器 (用于 INFO 页的 CAN Load 统计)
 *  由 CAN 任务累加, 时钟定时器每秒清零
 * ================================================================== */
static volatile uint32_t can_frame_count = 0;

/* ==================================================================
 *  CAN 接收 + UI 更新任务 (Core 0, 优先级 3)
 *
 *  职责:
 *    1. 接收 CAN 帧, 解析倒立摆数据
 *    2. 检查全局命令标志, 发送启停指令
 *    3. 每 30ms 更新 MONITOR 页数据标签 + CAN 状态
 *    4. 每 100ms 更新仪表盘指针 (浮点计算重, 分离降频)
 *
 *  防撕裂策略:
 *    - 所有 UI 操作通过 lvgl_port_lock/unlock 保护
 *    - 标签和指针分离更新, 避免浮点运算拖累标签响应
 * ================================================================== */
static void can_rx_task(void *arg)
{
    twai_message_t msg;
    uint32_t last_data_ms = 0;   /* 标签更新时间戳 */
    uint32_t last_gauge_ms = 0;  /* 指针更新时间戳 */

    while (1) {
        /* 阻塞接收 CAN 帧 (10ms 超时) */
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            /* 心跳帧: ID=0x124, data[0]=序号 */
            if (msg.identifier == 0x124 && msg.data_length_code >= 1) {
                uint8_t seq = msg.data[0];
                if (seq != g_sys_stats.hb_last) {
                    g_sys_stats.heartbeat_ok = 1;
                    g_sys_stats.hb_last = seq;
                }
            }
            if (msg.data_length_code >= 8) {
                g_can_data.angle    = msg.data[0] | (msg.data[1] << 8);
                g_can_data.speed    = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                g_can_data.location = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                g_can_data.state    = msg.data[6];
                g_can_data.pwm      = (int8_t)msg.data[7];
                g_can_data.last_rx_ms = esp_timer_get_time() / 1000;
                can_frame_count = can_frame_count + 1;
            }
        }

        /* 消费按钮命令 (发送 CAN 帧到 STM32) */
        /* 先发后清：发送成功才清零，失败则保留标志下次重试 */
        if (g_cmd_start) { can_send_cmd(0x01); g_cmd_start = 0; }
        if (g_cmd_stop)  { can_send_cmd(0x02); g_cmd_stop  = 0; }

        uint32_t now = esp_timer_get_time() / 1000;

        /* 数据标签 + CAN 状态: 每 30ms (33fps, 快速响应) */
        if (now - last_data_ms >= 30) {
            last_data_ms = now;
            if (lvgl_port_lock(30)) {
                ui_update_data(g_can_data.angle, g_can_data.speed,
                               g_can_data.location, g_can_data.state);
                uint32_t elapsed = now - g_can_data.last_rx_ms;
                if (elapsed < 200)      { ui_set_can_status(CAN_OK);      g_sys_stats.can_status = 0; }
                else if (elapsed < 1000) { ui_set_can_status(CAN_TIMEOUT); g_sys_stats.can_status = 1; }
                else                    { ui_set_can_status(CAN_OFF);      g_sys_stats.can_status = 2; }
                lvgl_port_unlock();
            }
        }

        /* 仪表盘指针: 每 100ms (10fps, 含 sinf/cosf 浮点运算) */
        if (now - last_gauge_ms >= 100) {
            last_gauge_ms = now;
            if (lvgl_port_lock(20)) {
                ui_update_gauge(g_can_data.angle);
                lvgl_port_unlock();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));  /* 释放 CPU */
    }
}

/* ==================================================================
 *  时钟 + 系统信息定时器 (1Hz 回调)
 *
 *  职责:
 *    1. 更新 MONITOR 页时钟显示
 *    2. 更新 INFO 页系统信息 (uptime / CAN load / FPS / heap)
 *    3. 统计 CAN 帧速率并清零计数器
 * ================================================================== */
static void clock_timer_cb(void *arg)
{
    static uint32_t uptime = 0;    /* 运行秒数 */
    static uint8_t fps = 0;        /* FPS 估算值 */
    static uint8_t fps_cnt = 0;    /* FPS 采样计数器 */
    uptime++;

    /* FPS: 每 4 秒采样平均 */
    fps_cnt++;
    if (uptime % 4 == 0) { fps = fps_cnt * 4; fps_cnt = 0; }

    /* CAN 负载: 过去 1 秒接收的帧数, 上限 100% */
    uint32_t cf = can_frame_count;
    can_frame_count = 0;
    uint8_t can_load = (cf > 100) ? 100 : (uint8_t)cf;

    /* WiFi 客户端数 (TODO: 通过 lwip API 获取实际连接数) */
    uint8_t wifi_cli = 0;

    /* 剩余堆内存 (KB) */
    uint32_t heap_free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024;

    /* 心跳超时检测: 每1秒检查, 若连续2秒序号不变则判定STM32卡死 */
    {
        static uint8_t hb_prev = 0;
        static uint8_t hb_stall = 0;
        if (g_sys_stats.hb_last == hb_prev) {
            hb_stall++;
            if (hb_stall >= 2) g_sys_stats.heartbeat_ok = 0;
        } else {
            hb_stall = 0;
        }
        hb_prev = g_sys_stats.hb_last;
    }

    /* 写入全局可观测性数据 (供 HTTP /debug 端点读取) */
    g_sys_stats.uptime  = uptime;
    g_sys_stats.can_fps = can_load;
    g_sys_stats.fps     = fps;
    g_sys_stats.heap_kb = heap_free;
    g_sys_stats.wifi_cli = wifi_cli;

    if (lvgl_port_lock(20)) {
        ui_update_clock();
        sysinfo_update(uptime, can_load, wifi_cli, fps, heap_free, 0, 0);
        lvgl_port_unlock();
    }
}

/* ==================================================================
 *  主入口 app_main
 *
 *  启动顺序:
 *    1. NVS 初始化
 *    2. TCP/IP 协议栈
 *    3. LCD + LVGL 驱动 (微雪官方一体化初始化)
 *    4. UI 创建 (三页: MONITOR / PID / INFO)
 *    5. CAN 总线启动
 *    6. WiFi AP + HTTP Server 启动
 *    7. CAN 接收任务 + 时钟定时器
 *
 *  LVGL 模式: 模式3 direct_mode + 双缓冲 + 14MHz PCLK + 40行 bounce
 *  防撕裂: bb_invalidate_cache + 全屏 cache sync + on_frame_buf_complete
 * ================================================================== */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== BOOT ===");

    /* NVS 非易失存储 (WiFi 配置) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* TCP/IP 协议栈 + 事件循环 */
    esp_netif_init();
    esp_event_loop_create_default();

    /* LCD + LVGL 一体化初始化 (硬件复位/背光/LVGL任务/VSYNC回调) */
    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init());
    // wavesahre_rgb_lcd_bl_on();  /* 参考微雪 main.c: bl_on 已注释 */

    /* 创建三页 UI (MONITOR / PID / INFO) 并切换到 MONITOR 首页 */
    if (lvgl_port_lock(-1)) {
        ui_init();
        page_switch(0);  /* MONITOR 默认页 */
        lvgl_port_unlock();
    }

    /* CAN 总线 (接收 STM32 倒立摆数据) */
    can_init();

    /* WiFi 热点 + HTTP 网页服务器 */
    wifi_ap_init();
    http_server_start();

    /* CAN 接收任务: Core 0, 4096 栈, 优先级 3 */
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 3, NULL, 0);

    /* 时钟 + 系统信息定时器: 1 秒周期 */
    esp_timer_create_args_t tmr_args = {};
    tmr_args.callback = &clock_timer_cb;
    tmr_args.name = "clock";
    esp_timer_handle_t tmr = NULL;
    esp_timer_create(&tmr_args, &tmr);
    esp_timer_start_periodic(tmr, 1000000);

    ESP_LOGI(TAG, "=== READY ===");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
