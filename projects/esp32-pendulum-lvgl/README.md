# STM32 + ESP32-S3 倒立摆监控与调试系统

**项目时间：** 2026.04 - 2026.06<br>
**技术栈：** C/C++、STM32F103、FreeRTOS、ESP32-S3、ESP-IDF、LVGL、CAN/TWAI

## 系统结构

```text
编码器 / 角度传感器
        |
        v
STM32F103 + FreeRTOS
  ControlTask / DisplayTask / KeyTask / SerialTask / CANTask
        |
        | CAN 500 kbit/s
        v
ESP32-S3 + LVGL + TWAI
  Monitor / PID / INFO
        |
        +-- WiFi AP + HTTP JSON telemetry
        +-- Web angle waveform
```

## 已实现

### STM32F103

- FreeRTOS 管理 Control、Display、Key、Serial、CAN 五类任务。
- `xTaskNotify/xTaskNotifyWait` 传递运行状态变化，避免为简单事件额外建立消息队列。
- 使用任务栈高水位检查和栈溢出钩子观察任务资源风险。
- 角度环和位置环双 PID，编码器采样、电机驱动和 1 ms 控制节拍。
- 双缓冲共享状态，避免控制任务与显示/串口任务读取半更新数据。
- 双缓冲通过 Cortex-M3 32 位对齐指针 load/store 发布快照；未使用 C11 `_Atomic`
  或引用计数，不把它外推为通用多读者无锁队列。
- OLED 显示角度、位置、PID 参数和运行状态。
- IWDG 看门狗和带 magic/checksum 的 Flash 故障记录。
- CAN 正常模式收发，与 ESP32-S3 交换控制状态。

### ESP32-S3

- LVGL 三页界面：实时监控、PID 参数调整、系统信息。
- 滑动切页、仪表指针、历史角度波形和状态提示。
- WiFi AP 与 HTTP JSON 遥测接口。
- TWAI/CAN 接收 STM32 数据，并通过 CAN 下发 PID 参数。

## 目录

```text
esp32s3/
  main.cpp        LVGL 界面、WiFi/HTTP、CAN 数据接入和调度
  ui.cpp          Monitor / PID / INFO 三页界面
  ui.h
stm32f103/
  src/            FreeRTOS 控制、CAN 和 Flash 故障记录
  include/        CAN 与故障记录接口
```

## 说明与边界

- 本目录是 2026.04 - 2026.06 本地工程的精选源码快照，不是完整发行包。
- 未包含 ESP-IDF managed components、厂家屏幕移植层、STM32 HAL/CMSIS、
  FreeRTOS 中间件、Keil/CubeMX 工程和烧录产物，因此不能单独编译。
- 仓库不包含可验证的稳态误差、恢复时间或长时间运行原始数据，不把这些代码参数写成
  实测性能结论。
- 公开版本已把测试 WiFi SSID/密码替换为占位符，并移除个人昵称。原测试密码应视为已经暴露，
  不应继续使用。
