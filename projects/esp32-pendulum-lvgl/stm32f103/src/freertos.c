/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

#include "AD.h"
#include "Encoder.h"
#include "Motor.h"
#include "Key.h"
#include "PID.h"
#include "OLED.h"
#include "Serial.h"
#include "LED.h"
#include "flash_fault.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CENTER_ANGLE		2010
#define CENTER_RANGE		500
#define START_PWM			35
#define START_TIME			100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* 任务句柄（由 CubeMX 代码生成器定义） */
osThreadId_t SerialTaskHandle;
const osThreadAttr_t SerialTask_attributes = {
  .name = "SerialTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 256 * 4,  /* 串口任务：栈需求1024字节 */
  .priority = (osPriority_t) osPriorityLow,
  .tz_module = 0,
  .reserved = 0,
};

osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 256 * 4,  /* 控制任务：栈需求1024字节 */
  .priority = (osPriority_t) osPriorityHigh,  /* 高优先级：确保实时控制任务响应及时 */
  .tz_module = 0,
  .reserved = 0,
};

osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
  .tz_module = 0,
  .reserved = 0,
};

osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 256 * 4,  /* 显示任务：栈需求1024字节 */
  .priority = (osPriority_t) osPriorityBelowNormal,
  .tz_module = 0,
  .reserved = 0,
};

osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .attr_bits = osThreadDetached,
  .cb_mem = NULL,
  .cb_size = 0,
  .stack_mem = NULL,
  .stack_size = 256 * 4,  /* CAN任务：栈需求1024字节 */
  .priority = (osPriority_t) osPriorityBelowNormal,
  .tz_module = 0,
  .reserved = 0,
};

/* 任务通知值定义（用于 ControlTask 向 DisplayTask 广播状态） */
#define NOTIFY_INIT_DONE    (1 << 0)
#define NOTIFY_BALANCE_OK   (1 << 1)
#define NOTIFY_FAULT        (1 << 2)

/* 任务通知值定义（用于 KeyTask 向 SerialTask 广播 RunState 变化） */
#define NOTIFY_RUNSTATE_CHANGED (1 << 3)

/* 消息队列句柄（当前已用任务通知替代，保留定义以备扩展） */
osMessageQueueId_t xDataQueueHandle;

/* 互斥量句柄（当前由双缓冲机制替代，保留定义以备扩展） */
osMutexId_t xDataMutexHandle;
const osMutexAttr_t xDataMutex_attr = {
  .name = "DataMutex",
};

/* 控制命令：任务 -> 控制任务（Cortex-M3 上 uint8_t/int16_t 读取是原子的） */
volatile uint8_t  g_CmdFlags = 0;     /* bit0: 切换运行状态 */
volatile int16_t  g_CmdLocDelta = 0;  /* 位置目标值增量 */

/* 外部引用 main.c 中的控制变量（控制逻辑已迁移到本任务的 ControlTask 中） */
extern uint8_t  RunState;
extern uint16_t Angle;
extern int16_t  Speed, Location;
extern PID_t    AnglePID;
extern PID_t    LocationPID;

/* 外部引用 main.c 中的按键变量 */
extern uint8_t KeyNum;

/* 外部引用 main.c 中的IWDG看门狗句柄 */
extern IWDG_HandleTypeDef hiwdg;

/* 双缓冲共享数据结构（用于 ISR/任务 间安全数据传输） */
/* 双缓冲优势：相比互斥量，无需临界区保护，避免优先级反转问题 */
/* 工作原理：ControlTask 写入一个缓冲区后发布新快照指针，       */
/* DisplayTask/SerialTask 读取另一个缓冲区，两者互不干扰。      */
typedef struct {
    uint16_t Angle;
    int16_t  Speed;
    int16_t  Location;
    uint8_t  RunState;
    PID_t    AnglePID;
    PID_t    LocationPID;
} SharedData_t;

SharedData_t g_SharedDataBuf[2];
SharedData_t *volatile g_pDataForISR  = &g_SharedDataBuf[0];
SharedData_t *volatile g_pDataForTask = &g_SharedDataBuf[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  ControlTask线程函数（最高优先级）
  * @note   功能说明：
  *         1. 1kHz实时控制循环，由TIM1中断周期性唤醒
  *         2. 执行ADC采样、编码器读取、PID计算、电机驱动
  *         3. 实现倒立摆状态机（停止、检测、启动、平衡）
  *         4. 双缓冲数据交换，确保数据一致性
  *         5. 任务通知机制，高效唤醒和同步
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
    /* 控制循环静态变量（保持状态跨周期） */
    static uint16_t Count0, Count1, Count2, CountTime;
    static uint16_t Angle0, Angle1, Angle2;
    static uint16_t ledTick = 0;
    uint8_t prevRunState = 0;

    /* 等待最高优先级任务就绪后，才开启 TIM1 中断 */
    /* 确保任务调度器已运行，本任务能立即响应中断唤醒 */
    /* 设计要点：TIM1 中断必须在 ControlTask 启动后才能开启， */
    /* 否则中断可能在任务未就绪时触发，导致系统不稳定。        */
    HAL_TIM_Base_Start_IT(&htim1);

    /* 广播系统初始化完成事件，DisplayTask 收到后开始刷新 */
    xTaskNotify((TaskHandle_t)DisplayTaskHandle, NOTIFY_INIT_DONE, eSetBits);

    for(;;)
    {
        /* 等待 TIM1 中断唤醒（1ms 周期） */
        /* 相比裸机轮询，任务通知机制让 CPU 在低优先级任务运行时休眠，
           收到通知后立即抢占，兼顾实时性与功耗 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 心跳 LED：每 500ms 翻转一次，指示 ControlTask 运行正常 */
        ledTick++;
        if (ledTick >= 500)
        {
            ledTick = 0;
            LED_Turn();
        }

        /* 按键消抖 tick（每 1ms 调用一次） */
        Key_Tick();

        /* 处理任务发来的控制命令（Cortex-M3 上 uint8_t/int16_t 读取是原子的） */
        uint8_t cmd = g_CmdFlags;
        if (cmd & 0x01)
        {
            g_CmdFlags = cmd & ~0x01;
            if (RunState == 0) RunState = 21;
            else RunState = 0;
        }

        int16_t delta = g_CmdLocDelta;
        if (delta != 0)
        {
            g_CmdLocDelta = 0;
            LocationPID.Target += delta;
            if (LocationPID.Target > 4080) LocationPID.Target = 4080;
            if (LocationPID.Target < -4080) LocationPID.Target = -4080;
        }

        /* 传感器采样 */
        Angle = AD_GetValue();
        Speed = Encoder_Get();
        Location += Speed;

        /* ========== 运行状态机 ========== */
        if (RunState == 0)
        {
            Motor_SetPWM(0);

#ifdef FLASH_FAULT_ENABLE
            /* 检测到摆杆倒下，记录故障信息到Flash */
            {
                static uint8_t last_runstate = 0;
                if (last_runstate != 0 && RunState == 0)
                {
                    /* 从非停止状态切换到停止状态，说明摆杆倒下 */
                    Flash_FaultRecord_t fault_record;
                    if (Flash_ReadFault(&fault_record))
                    {
                        fault_record.fall_count++;
                        fault_record.last_angle = Angle;
                        fault_record.last_reason = 1;  /* 原因1：角度偏离过大 */
                        Flash_SaveFault(&fault_record);

                        Serial_Printf("[FAULT] Fall #%d, Angle: %d\n",
                                      fault_record.fall_count, Angle);
                    }
                }
                last_runstate = RunState;
            }
#endif
        }
        else if (RunState == 1)
        {
            Count0++;
            if (Count0 >= 40)
            {
                Count0 = 0;
                Angle2 = Angle1;
                Angle1 = Angle0;
                Angle0 = Angle;

                /* 检测右侧摆动到最高点后下摆，触发正向摆起 */
                if (Angle0 > CENTER_ANGLE + CENTER_RANGE
                 && Angle1 > CENTER_ANGLE + CENTER_RANGE
                 && Angle2 > CENTER_ANGLE + CENTER_RANGE
                 && Angle1 < Angle0
                 && Angle1 < Angle2)
                {
                    RunState = 21;
                }

                /* 检测左侧摆动到最高点后下摆，触发反向摆起 */
                if (Angle0 < CENTER_ANGLE - CENTER_RANGE
                 && Angle1 < CENTER_ANGLE - CENTER_RANGE
                 && Angle2 < CENTER_ANGLE - CENTER_RANGE
                 && Angle1 > Angle0
                 && Angle1 > Angle2)
                {
                    RunState = 31;
                }

                /* 检测摆杆已在平衡范围内，直接进入稳态 */
                if (Angle0 > CENTER_ANGLE - CENTER_RANGE
                 && Angle0 < CENTER_ANGLE + CENTER_RANGE
                 && Angle1 > CENTER_ANGLE - CENTER_RANGE
                 && Angle1 < CENTER_ANGLE + CENTER_RANGE)
                {
                    Location = 0;
                    AnglePID.ErrorInt = 0;
                    LocationPID.ErrorInt = 0;
                    RunState = 4;
                }
            }
        }
        else if (RunState == 21)
        {
            Motor_SetPWM(START_PWM);
            CountTime = START_TIME;
            RunState = 22;
        }
        else if (RunState == 22)
        {
            CountTime--;
            if (CountTime == 0) RunState = 23;
        }
        else if (RunState == 23)
        {
            Motor_SetPWM(-START_PWM);
            CountTime = START_TIME;
            RunState = 24;
        }
        else if (RunState == 24)
        {
            CountTime--;
            if (CountTime == 0)
            {
                Motor_SetPWM(0);
                RunState = 1;
            }
        }
        else if (RunState == 31)
        {
            Motor_SetPWM(-START_PWM);
            CountTime = START_TIME;
            RunState = 32;
        }
        else if (RunState == 32)
        {
            CountTime--;
            if (CountTime == 0) RunState = 33;
        }
        else if (RunState == 33)
        {
            Motor_SetPWM(START_PWM);
            CountTime = START_TIME;
            RunState = 34;
        }
        else if (RunState == 34)
        {
            CountTime--;
            if (CountTime == 0)
            {
                Motor_SetPWM(0);
                RunState = 1;
            }
        }
        else if (RunState == 4)
        {
            /* 平衡中若角度偏离过大，自动切换到停止状态 */
            if (!(Angle > CENTER_ANGLE - CENTER_RANGE
                && Angle < CENTER_ANGLE + CENTER_RANGE))
            {
                RunState = 0;
            }

            /* 角度环 PID：每 5ms 更新一次（1kHz / 5 = 200Hz） */
            Count1++;
            if (Count1 >= 5)
            {
                Count1 = 0;
                AnglePID.Actual = Angle;
                PID_Update(&AnglePID);
                Motor_SetPWM(AnglePID.Out);
            }

            /* 位置环 PID：每 50ms 更新一次（1kHz / 50 = 20Hz） */
            Count2++;
            if (Count2 >= 50)
            {
                Count2 = 0;
                LocationPID.Actual = Location;
                PID_Update(&LocationPID);
                AnglePID.Target = CENTER_ANGLE + LocationPID.Out;
            }
        }

        /* ========== 双缓冲数据交换 ========== */
        /* 写入 ISR 缓冲区（实际上是任务缓冲区，命名保留历史原因） */
        g_pDataForISR->Angle       = Angle;
        g_pDataForISR->Speed       = Speed;
        g_pDataForISR->Location    = Location;
        g_pDataForISR->RunState    = RunState;
        g_pDataForISR->AnglePID    = AnglePID;
        g_pDataForISR->LocationPID = LocationPID;

        /* 原子指针发布：Cortex-M3 上 32 位对齐指针 load/store 是原子的。      */
        /* 读者一次加载指针后快速拷贝快照；两次角色赋值是顺序执行的，          */
        /* 这里不使用 C11 atomic，也不把它外推为通用多读者无锁队列。          */
        /* 切换后：g_pDataForISR 指向旧数据，g_pDataForTask 指向新数据。       */
        SharedData_t *tmp = g_pDataForISR;
        g_pDataForISR = g_pDataForTask;
        g_pDataForTask = tmp;

        /* 检测 RunState 变化，通过任务通知广播给 DisplayTask */
        if (RunState != prevRunState)
        {
            if (RunState == 4)
                xTaskNotify((TaskHandle_t)DisplayTaskHandle, NOTIFY_BALANCE_OK, eSetBits);
            else if (RunState == 0)
                xTaskNotify((TaskHandle_t)DisplayTaskHandle, NOTIFY_FAULT, eSetBits);
            prevRunState = RunState;
        }

        /* ========== 串口指令处理（需启用SERIAL_CMD_ENABLE） ========== */
        /* 非阻塞读取串口指令，更新PID参数 */
        /* ControlTask自己修改自己的变量，无并发冲突，无需互斥量 */
#ifdef SERIAL_CMD_ENABLE
        {
            Serial_Cmd_t cmd;
            if (Serial_GetCmd(&cmd))
            {
                switch (cmd.type)
                {
                    case CMD_SET_KP_A: AnglePID.Kp = cmd.value; break;
                    case CMD_SET_KI_A: AnglePID.Ki = cmd.value; break;
                    case CMD_SET_KD_A: AnglePID.Kd = cmd.value; break;
                    case CMD_SET_KP_L: LocationPID.Kp = cmd.value; break;
                    case CMD_SET_KI_L: LocationPID.Ki = cmd.value; break;
                    case CMD_SET_KD_L: LocationPID.Kd = cmd.value; break;
                    default: break;
                }
            }
        }
#endif

        /* ========== 喂狗操作 ========== */
        /* 每1ms循环末尾喂狗，只要ControlTask正常运行就不会复位 */
        /* 若控制任务死锁或跑飞，1.6秒后系统自动硬件复位 */
        HAL_IWDG_Refresh(&hiwdg);
    }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief  KeyTask线程函数（普通优先级）
* @note   功能说明：
*         1. 10ms周期按键扫描
*         2. 检测按键并发送控制命令（切换运行状态、调整位置目标）
*         3. 检测RunState变化，通过任务通知通知SerialTask
*         4. 使用全局变量传递命令（Cortex-M3上原子操作）
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
	uint8_t PrevRunState = 0;
	uint8_t currRunState;

	for(;;)
	{
		osDelay(10);

		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			g_CmdFlags |= 0x01; /* 请求切换运行状态 */
		}
		if (KeyNum == 2)
		{
			g_CmdLocDelta = 408;
		}
		if (KeyNum == 3)
		{
			g_CmdLocDelta = -408;
		}

		/* 检测 RunState 变化，通过任务通知通知 SerialTask（替代消息队列） */
		currRunState = g_pDataForTask->RunState;
		if (currRunState != PrevRunState)
		{
			/* 使用任务通知代替消息队列，减少上下文切换开销 */
			xTaskNotify((TaskHandle_t)SerialTaskHandle, NOTIFY_RUNSTATE_CHANGED, eSetBits);
			PrevRunState = currRunState;
		}
	}
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief  DisplayTask线程函数（低优先级）
* @note   功能说明：
*         1. 50ms周期OLED显示刷新
*         2. 通过双缓冲读取数据，确保数据一致性
*         3. 接收任务通知，显示系统状态（平衡中/故障）
*         4. 显示角度环和位置环的PID参数和状态
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	SharedData_t LocalData;
	SharedData_t *p;
	uint32_t events;

	/* 等待系统初始化完成事件后再开始刷新（展示任务通知同步机制） */
	xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &events, portMAX_DELAY);

	for(;;)
	{
		osDelay(50);

		/* 检查是否有新通知（非阻塞清零读取） */
		xTaskNotifyWait(0, 0xFFFFFFFF, &events, 0);

		/* 双缓冲读取：直接读取 g_pDataForTask 指向的缓冲区副本 */
		p = g_pDataForTask;
		LocalData = *p;

		OLED_Printf(42, 0, OLED_6X8, "%02d", LocalData.RunState);

		/* 根据任务通知状态在右上角显示系统状态图标 */
		if (events & NOTIFY_BALANCE_OK)
		{
			OLED_Printf(120, 0, OLED_6X8, "*");   /* 平衡中 */
		}
		else if (events & NOTIFY_FAULT)
		{
			OLED_Printf(120, 0, OLED_6X8, "!");   /* 故障/停止 */
		}
		else
		{
			OLED_Printf(120, 0, OLED_6X8, " ");
		}

		OLED_Printf(0, 0, OLED_6X8, "Angle");
		OLED_Printf(0, 12, OLED_6X8, "Kp:%05.3f", LocalData.AnglePID.Kp);
		OLED_Printf(0, 20, OLED_6X8, "Ki:%05.3f", LocalData.AnglePID.Ki);
		OLED_Printf(0, 28, OLED_6X8, "Kd:%05.3f", LocalData.AnglePID.Kd);
		OLED_Printf(0, 40, OLED_6X8, "Tar:%04.0f", LocalData.AnglePID.Target);
		OLED_Printf(0, 48, OLED_6X8, "Act:%04d", LocalData.Angle);
		OLED_Printf(0, 56, OLED_6X8, "Out:%+04.0f", LocalData.AnglePID.Out);

		OLED_Printf(64, 0, OLED_6X8, "Location");
		OLED_Printf(64, 12, OLED_6X8, "Kp:%05.3f", LocalData.LocationPID.Kp);
		OLED_Printf(64, 20, OLED_6X8, "Ki:%05.3f", LocalData.LocationPID.Ki);
		OLED_Printf(64, 28, OLED_6X8, "Kd:%05.3f", LocalData.LocationPID.Kd);
		OLED_Printf(64, 40, OLED_6X8, "Tar:%+05.0f", LocalData.LocationPID.Target);
		OLED_Printf(64, 48, OLED_6X8, "Act:%+05d", LocalData.Location);
		OLED_Printf(64, 56, OLED_6X8, "Out:%+04.0f", LocalData.LocationPID.Out);

		OLED_Update();
	}
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief  SerialTask线程函数（低优先级）
* @note   功能说明：
*         1. 100ms周期串口数据打印
*         2. 通过双缓冲读取数据，确保数据一致性
*         3. 接收任务通知，打印RunState变化事件
*         4. 每5秒打印各任务栈剩余空间（Stack High Water Mark）
*         5. 支持文本和二进制数据帧格式
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void *argument)
{
  /* USER CODE BEGIN StartSerialTask */
	SharedData_t LocalData;
	SharedData_t *p;
	uint32_t statsTick = 0;

	for(;;)
	{
		osDelay(100);

		/* 双缓冲读取 */
		p = g_pDataForTask;
		LocalData = *p;

		/* 保留文本打印（调试用，可删除） */
		Serial_Printf("A:%d,T:%d,S:%d,L:%d,RS:%d,O:%d\n",
					  LocalData.Angle, (int)LocalData.AnglePID.Target,
					  LocalData.Speed, LocalData.Location,
					  LocalData.RunState, (int)LocalData.AnglePID.Out);

		/* 发送二进制数据帧（上位机用，需要时取消注释） */
		// Serial_SendDataFrame(LocalData.Angle,
		// 					 (int16_t)LocalData.AnglePID.Target,
		// 					 LocalData.Speed,
		// 					 LocalData.Location,
		// 					 LocalData.RunState,
		// 					 (int16_t)LocalData.AnglePID.Out,
		// 					 (int16_t)LocalData.LocationPID.Out);

		/* 检测 RunState 变化事件（由 KeyTask 通过任务通知发送） */
		uint32_t notifyValue = 0;
		if (xTaskNotifyWait(0, NOTIFY_RUNSTATE_CHANGED, &notifyValue, 0) == pdTRUE)
		{
			if (notifyValue & NOTIFY_RUNSTATE_CHANGED)
			{
				Serial_Printf("[EVENT] RunState -> %d\n", LocalData.RunState);
			}
		}

		/* 每 5 秒打印一次各任务栈剩余空间（Stack High Water Mark） */
		statsTick++;
		if (statsTick >= 50)
		{
			statsTick = 0;
			UBaseType_t wmSerial  = uxTaskGetStackHighWaterMark((TaskHandle_t)SerialTaskHandle);
			UBaseType_t wmDisplay = uxTaskGetStackHighWaterMark((TaskHandle_t)DisplayTaskHandle);
			UBaseType_t wmKey     = uxTaskGetStackHighWaterMark((TaskHandle_t)KeyTaskHandle);
			UBaseType_t wmControl = uxTaskGetStackHighWaterMark((TaskHandle_t)ControlTaskHandle);
			Serial_Printf("[STACK] Serial:%u Display:%u Key:%u Control:%u words\n",
						  wmSerial, wmDisplay, wmKey, wmControl);
		}
	}
  /* USER CODE END StartSerialTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief  CAN任务线程函数（低优先级）
* @note   功能说明：
*         1. 等待系统稳定后初始化CAN
*         2. 执行CAN回环测试
*         3. 测试完成后删除任务
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
  /* 等待3秒，让系统稳定 */
  osDelay(3000);

  /* 初始化CAN（正常模式，用于与ESP32通信） */
  CAN_Init_Normal();

  /* 循环发送数据给ESP32 */
  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint32_t rxId;
  uint8_t rxData[8];
  uint8_t rxLen;
  char buf[64];
  int n;

  for(;;)
  {
    osDelay(1000);  /* 每1秒发送一次 */

    /* 发送数据给ESP32 */
    CAN_Send_Data(0x123, txData, 8);

    /* 检查是否收到ESP32的回复 */
    if (CAN_Receive_Data(&rxId, rxData, &rxLen))
    {
      n = sprintf(buf, "[CAN] RX ID=0x%03lX Data=%02X %02X %02X %02X\r\n",
                  rxId, rxData[0], rxData[1], rxData[2], rxData[3]);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
    }

    /* 更新发送数据（递增） */
    txData[0]++;
  }
  /* USER CODE END StartCANTask */
}

/**
  * @brief  FreeRTOS初始化函数
  * @note   功能说明：
  *         1. 创建互斥量（当前由双缓冲替代，保留定义）
  *         2. 创建消息队列（当前由任务通知替代，保留定义）
  *         3. 创建所有任务线程
  *         4. 任务优先级分配：ControlTask > KeyTask > DisplayTask > SerialTask
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 创建互斥量（当前由双缓冲机制替代，保留定义以备扩展） */
  /* USER CODE BEGIN RTOS_MUTEX */
  xDataMutexHandle = osMutexNew(&xDataMutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 软件定时器保留占位（LED 心跳已移至 ControlTask） */
  /* USER CODE END RTOS_TIMERS */

  /* 创建消息队列（当前已用任务通知替代，保留定义以备扩展） */
  /* USER CODE BEGIN RTOS_QUEUES */
  xDataQueueHandle = osMessageQueueNew(10, sizeof(uint8_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* 创建任务线程 */
  /* 任务优先级分配：ControlTask(最高) > KeyTask > DisplayTask > SerialTask(最低) */
  /* 优先级设计要点：实时控制任务必须最高优先级，确保1kHz控制周期不被中断 */

  /* 创建串口任务（低优先级，100ms周期） */
  SerialTaskHandle = osThreadNew(StartSerialTask, NULL, &SerialTask_attributes);

  /* 创建控制任务（最高优先级，1kHz周期） */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* 创建按键任务（普通优先级，10ms周期） */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* 创建显示任务（低优先级，50ms周期） */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* 创建CAN任务（低优先级，一次性测试） */
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 任务通知无需额外创建，直接使用 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Application */
/**
  * @brief  栈溢出钩子函数
  * @note   当检测到任务栈溢出时，FreeRTOS会调用此函数
  *         仅允许极简操作：写备份寄存器标记、LED快闪、软件复位
  *         禁止调用printf、OLED_Printf等复杂函数（栈已溢出，再调函数会二次崩溃）
  * @param  xTask: 发生溢出的任务句柄
  * @param  pcTaskName: 发生溢出的任务名称
  * @retval None
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* 写备份寄存器标记（如果支持），记录溢出的任务名称 */
    /* 这里仅用LED快闪指示错误 */

    /* LED快闪指示栈溢出错误 */
    /* 禁止使用printf/OLED_Printf等复杂函数，栈已溢出 */
    __disable_irq();
    for(;;)
    {
        /* 快速闪烁LED指示错误 */
        LED_Turn();
        /* 简单延时 */
        for(volatile uint32_t i = 0; i < 100000; i++);
    }
}

/**
  * @brief  内存分配失败钩子函数
  * @note   当pvPortMalloc()分配失败时调用
  *         仅允许极简操作，禁止调用复杂函数
  * @retval None
  */
void vApplicationMallocFailedHook(void)
{
    /* 内存分配失败，LED快闪指示错误 */
    __disable_irq();
    for(;;)
    {
        LED_Turn();
        for(volatile uint32_t i = 0; i < 200000; i++);
    }
}
/* USER CODE END Application */
