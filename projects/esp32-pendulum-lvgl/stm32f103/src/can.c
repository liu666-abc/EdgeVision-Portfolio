#include "can.h"
#include "Serial.h"

extern UART_HandleTypeDef huart1;

/* 按照can工程的方式，用直接寄存器操作初始化CAN */
void CAN_Init_Loopback(void)
{
    uint32_t t;
    char buf[64];
    int n;

    /* 使能时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* GPIO: PA11=CAN_RX, PA12=CAN_TX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Enter init mode */
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_DBF;
    t = 1000000;
    while (!(CAN1->MSR & CAN_MSR_INAK) && t--);
    n = sprintf(buf, "INAK wait: %lu\r\n", t);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);

    /* BTR: loopback, prescaler=9, BS1=5TQ, BS2=2TQ, SJW=1TQ => 500kbps */
    CAN1->BTR = (1UL << 30) |  /* LBKM=1 (loopback) */
                (0UL << 24) |  /* SJW=1TQ */
                (1UL << 20) |  /* BS2=2TQ */
                (4UL << 16) |  /* BS1=5TQ */
                (9 - 1);       /* Prescaler=9 */

    /* Exit init mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    t = 1000000;
    while ((CAN1->MSR & CAN_MSR_INAK) && t--);
    n = sprintf(buf, "Exit init: %lu MSR=0x%08lX\r\n", t, CAN1->MSR);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);

    if (CAN1->MSR & CAN_MSR_INAK)
        HAL_UART_Transmit(&huart1, (uint8_t *)"ERR: Still in init!\r\n", 21, HAL_MAX_DELAY);
    else
        HAL_UART_Transmit(&huart1, (uint8_t *)"CAN ready\r\n", 11, HAL_MAX_DELAY);

    n = sprintf(buf, "BTR=0x%08lX MCR=0x%08lX\r\n", CAN1->BTR, CAN1->MCR);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);

    /* Filter: direct register config - 32-bit mask mode, accept all */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FM1R &= ~(1UL << 0);
    CAN1->FS1R |= (1UL << 0);
    CAN1->FFA1R &= ~(1UL << 0);
    CAN1->sFilterRegister[0].FR1 = 0x00000000;
    CAN1->sFilterRegister[0].FR2 = 0x00000000;
    CAN1->FA1R |= (1UL << 0);
    CAN1->FMR &= ~CAN_FMR_FINIT;

    HAL_UART_Transmit(&huart1, (uint8_t *)"Filter OK\r\n", 11, HAL_MAX_DELAY);
}

/* 发送测试帧 */
void CAN_Test_Send(void)
{
    uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint8_t rxData[8] = {0};
    char buf[64];
    int n;
    uint32_t timeout;

    HAL_UART_Transmit(&huart1, (uint8_t *)"TX...\r\n", 7, HAL_MAX_DELAY);

    /* Wait for TX mailbox 0 empty */
    timeout = 1000000;
    while (!(CAN1->TSR & CAN_TSR_TME0) && timeout--);
    if (!timeout) { HAL_UART_Transmit(&huart1, (uint8_t *)"ERR:Mailbox\r\n", 13, HAL_MAX_DELAY); return; }

    /* Set TX mailbox 0 */
    CAN1->sTxMailBox[0].TIR = (0x123UL << 21);
    CAN1->sTxMailBox[0].TDTR = 8;
    CAN1->sTxMailBox[0].TDLR = (txData[3]<<24)|(txData[2]<<16)|(txData[1]<<8)|txData[0];
    CAN1->sTxMailBox[0].TDHR = (txData[7]<<24)|(txData[6]<<16)|(txData[5]<<8)|txData[4];
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    /* Wait for TX complete */
    timeout = 1000000;
    while (!(CAN1->TSR & CAN_TSR_RQCP0) && timeout--);
    if (timeout)
    {
        uint32_t tsr = CAN1->TSR;
        n = sprintf(buf, "TX OK TSR=0x%08lX\r\n", tsr);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"TX timeout\r\n", 12, HAL_MAX_DELAY);
        return;
    }

    /* Check RX */
    HAL_Delay(100);

    uint32_t rf0r = CAN1->RF0R;
    n = sprintf(buf, "RF0R=0x%08lX\r\n", rf0r);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);

    uint32_t fmp = rf0r & 0x03;
    if (fmp > 0)
    {
        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
        uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

        uint32_t rx_id = rir >> 21;
        uint32_t rx_dlc = rdtr & 0x0F;

        rxData[0] = rdlr & 0xFF;
        rxData[1] = (rdlr >> 8) & 0xFF;
        rxData[2] = (rdlr >> 16) & 0xFF;
        rxData[3] = (rdlr >> 24) & 0xFF;
        rxData[4] = rdhr & 0xFF;
        rxData[5] = (rdhr >> 8) & 0xFF;
        rxData[6] = (rdhr >> 16) & 0xFF;
        rxData[7] = (rdhr >> 24) & 0xFF;

        CAN1->RF0R |= CAN_RF0R_RFOM0;

        n = sprintf(buf, "RX ID=0x%03lX DLC=%lu\r\n", rx_id, rx_dlc);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
        n = sprintf(buf, "Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    rxData[0], rxData[1], rxData[2], rxData[3],
                    rxData[4], rxData[5], rxData[6], rxData[7]);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);

        int match = 1;
        for (int i = 0; i < 8; i++)
            if (txData[i] != rxData[i]) { match = 0; break; }
        HAL_UART_Transmit(&huart1, (uint8_t *)(match ? "[OK] CAN working!\r\n" : "[ERR] mismatch\r\n"),
                           match ? 20 : 16, HAL_MAX_DELAY);
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"[ERR] No RX\r\n", 13, HAL_MAX_DELAY);
    }
}

/* CAN正常模式初始化（用于与ESP32通信） */
void CAN_Init_Normal(void)
{
    uint32_t t;
    char buf[64];
    int n;

    /* 使能时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* GPIO: PA11=CAN_RX, PA12=CAN_TX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Enter init mode */
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_DBF;
    t = 1000000;
    while (!(CAN1->MSR & CAN_MSR_INAK) && t--);

    /* BTR: normal mode, prescaler=9, BS1=5TQ, BS2=2TQ, SJW=1TQ => 500kbps */
    /* 36MHz / (9 * 8) = 500kbps */
    CAN1->BTR = (0UL << 30) |  /* LBKM=0 (normal mode) */
                (0UL << 24) |  /* SJW=1TQ */
                (1UL << 20) |  /* BS2=2TQ */
                (4UL << 16) |  /* BS1=5TQ */
                (9 - 1);       /* Prescaler=9 */

    /* Exit init mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    t = 1000000;
    while ((CAN1->MSR & CAN_MSR_INAK) && t--);

    /* Filter: accept all */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FM1R &= ~(1UL << 0);
    CAN1->FS1R |= (1UL << 0);
    CAN1->FFA1R &= ~(1UL << 0);
    CAN1->sFilterRegister[0].FR1 = 0x00000000;
    CAN1->sFilterRegister[0].FR2 = 0x00000000;
    CAN1->FA1R |= (1UL << 0);
    CAN1->FMR &= ~CAN_FMR_FINIT;

    n = sprintf(buf, "[CAN] Normal mode ready\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
}

/* 发送CAN数据帧 */
void CAN_Send_Data(uint32_t id, uint8_t *data, uint8_t len)
{
    char buf[64];
    int n;
    uint32_t timeout;

    /* Wait for TX mailbox empty */
    timeout = 1000000;
    while (!(CAN1->TSR & CAN_TSR_TME0) && timeout--);
    if (!timeout) {
        n = sprintf(buf, "[CAN] TX mailbox full\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
        return;
    }

    /* Set TX mailbox 0 */
    CAN1->sTxMailBox[0].TIR = (id << 21);  /* StdId */
    CAN1->sTxMailBox[0].TDTR = len;         /* DLC */

    if (len >= 4) {
        CAN1->sTxMailBox[0].TDLR = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
    }
    if (len > 4) {
        CAN1->sTxMailBox[0].TDHR = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4];
    }

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;  /* Request TX */

    /* Wait for TX complete */
    timeout = 1000000;
    while (!(CAN1->TSR & CAN_TSR_RQCP0) && timeout--);
    if (timeout) {
        n = sprintf(buf, "[CAN] TX OK ID=0x%03lX\r\n", id);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
    } else {
        n = sprintf(buf, "[CAN] TX timeout\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, HAL_MAX_DELAY);
    }
}

/* 检查并接收CAN数据 */
uint8_t CAN_Receive_Data(uint32_t *id, uint8_t *data, uint8_t *len)
{
    uint32_t rf0r = CAN1->RF0R;
    uint32_t fmp = rf0r & 0x03;

    if (fmp > 0)
    {
        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
        uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

        *id = rir >> 21;
        *len = rdtr & 0x0F;

        data[0] = rdlr & 0xFF;
        data[1] = (rdlr >> 8) & 0xFF;
        data[2] = (rdlr >> 16) & 0xFF;
        data[3] = (rdlr >> 24) & 0xFF;
        if (*len > 4) {
            data[4] = rdhr & 0xFF;
            data[5] = (rdhr >> 8) & 0xFF;
            data[6] = (rdhr >> 16) & 0xFF;
            data[7] = (rdhr >> 24) & 0xFF;
        }

        CAN1->RF0R |= CAN_RF0R_RFOM0;  /* Release FIFO */
        return 1;
    }
    return 0;
}
