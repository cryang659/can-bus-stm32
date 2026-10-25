/**
 * CAN 总线练习 —— STM32F103C8T6（标准外设库 V3.5）
 *
 * 硬件：
 *   PA11 = CAN_RX  -> 收发器 RX/CRX
 *   PA12 = CAN_TX  -> 收发器 TX/CTX
 *   收发器：TJA1050（5V）或 SN65HVD230（3.3V）
 *   双节点时：总线两端各接 120Ω，两节点必须共地
 *
 * 波特率：500 kbps
 *   系统 72MHz -> APB1 36MHz
 *   Prescaler=9, BS1=6Tq, BS2=1Tq
 *   1 位 = 1(SYNC)+6+1 = 8Tq，36MHz/9/8 = 500kbps
 *   采样点 = (1+6)/8 = 87.5%
 *
 * 两种模式（改下面的 CAN_WORK_MODE）：
 *   CAN_Mode_LoopBack —— 单板自测，TX 内部环回 RX，不需要收发器/第二块板
 *   CAN_Mode_Normal   —— 双节点，接收发器 + 两端 120Ω + 共地
 *
 * 串口：USART1，PA9(TX) / PA10(RX)，115200 8N1
 */
#include "stm32f10x.h"
#include <stdint.h>

#define CAN_WORK_MODE   CAN_Mode_LoopBack

#define CAN_PRESCALER   9
#define CAN_BS1         CAN_BS1_6tq
#define CAN_BS2         CAN_BS2_1tq

static volatile uint32_t g_tx_count = 0;
static volatile uint32_t g_rx_count = 0;

/* ---------------- 延时（粗略，用轮询循环，不占用 SysTick） ---------------- */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0; i < 18000; i++) {
            __NOP();
        }
    }
}

/* ---------------- USART1 ---------------- */
static void uart_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

static void uart_putc(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
    USART_SendData(USART1, (uint16_t)c);
}

static void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_puthex8(uint8_t v)
{
    const char *h = "0123456789ABCDEF";
    uart_putc(h[v >> 4]);
    uart_putc(h[v & 0x0F]);
}

static void uart_putu32(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    if (v == 0) {
        uart_putc('0');
        return;
    }
    while (v > 0) {
        buf[--i] = (char)('0' + (v % 10));
        v /= 10;
    }
    uart_puts(&buf[i]);
}

/* ---------------- CAN ---------------- */
static void can_init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PA11 = CAN_RX，上拉输入 */
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* PA12 = CAN_TX，复用推挽 */
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(CAN1);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;      /* 自动离线恢复，调试期必须开 */
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;     /* 硬件自动重发 */
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_WORK_MODE;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1;
    can.CAN_BS2 = CAN_BS2;
    can.CAN_Prescaler = CAN_PRESCALER;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success) {
        uart_puts("[CAN ] init FAILED\r\n");
        while (1) { }
    }

    /* 滤波器 0：掩码全 0 = 全部放行（调试阶段） */
    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0x0000;
    filter.CAN_FilterIdLow = 0x0000;
    filter.CAN_FilterMaskIdHigh = 0x0000;
    filter.CAN_FilterMaskIdLow = 0x0000;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* 返回 1=发送成功，0=邮箱满或总线错误 */
static uint8_t can_send(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CanTxMsg tx;
    uint8_t mbox;
    uint32_t timeout = 0;

    tx.StdId = std_id;
    tx.ExtId = 0;
    tx.IDE = CAN_Id_Standard;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = (len > 8) ? 8 : len;
    for (uint8_t i = 0; i < tx.DLC; i++) {
        tx.Data[i] = data[i];
    }

    mbox = CAN_Transmit(CAN1, &tx);
    if (mbox == CAN_TxStatus_NoMailBox) {
        return 0;
    }

    while (timeout++ < 100000) {
        uint8_t st = CAN_TransmitStatus(CAN1, mbox);
        if (st == CAN_TxStatus_Ok) {
            return 1;
        }
        if (st == CAN_TxStatus_Failed) {
            return 0;
        }
    }
    return 0;
}

/* CAN1 RX0 中断：收到一帧就打印 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;

    CAN_Receive(CAN1, CAN_FIFO0, &rx);
    g_rx_count++;

    uart_puts("[RX  ] id=0x");
    uart_puthex8((uint8_t)(rx.StdId >> 8));
    uart_puthex8((uint8_t)(rx.StdId));
    uart_puts(" dlc=");
    uart_putu32(rx.DLC);
    uart_puts(" data=");
    for (uint8_t i = 0; i < rx.DLC; i++) {
        uart_puthex8(rx.Data[i]);
        uart_putc(' ');
    }
    uart_puts("\r\n");
}

int main(void)
{
    uint8_t data[8];
    uint32_t counter = 0;
    uint32_t err_tick = 0;

    uart_init();
    uart_puts("\r\n===== CAN demo (STM32F103C8, 500kbps) =====\r\n");
    uart_puts("[BOOT] SystemCoreClock=");
    uart_putu32(SystemCoreClock);
    uart_puts("\r\n");

#if (CAN_WORK_MODE == CAN_Mode_LoopBack)
    uart_puts("[CAN ] mode=LoopBack (self-test, no transceiver needed)\r\n");
#else
    uart_puts("[CAN ] mode=Normal (two nodes, 120ohm + common GND)\r\n");
#endif

    can_init();
    uart_puts("[CAN ] init OK, filter=accept-all, IRQ=RX0\r\n");

    while (1) {
        data[0] = (uint8_t)(counter >> 24);
        data[1] = (uint8_t)(counter >> 16);
        data[2] = (uint8_t)(counter >> 8);
        data[3] = (uint8_t)(counter);
        data[4] = 0xAA;
        data[5] = 0x55;
        data[6] = (uint8_t)g_rx_count;
        data[7] = 0x01;

        if (can_send(0x123, data, 8)) {
            g_tx_count++;
            uart_puts("[TX  ] id=0x123 cnt=");
            uart_putu32(counter);
            uart_puts(" tx=");
            uart_putu32(g_tx_count);
            uart_puts(" rx=");
            uart_putu32(g_rx_count);
            uart_puts("\r\n");
        } else {
            uart_puts("[TX  ] FAIL (mailbox full / bus error)\r\n");
        }

        counter++;
        delay_ms(500);

        /* 每 5 秒打印一次错误计数 */
        if (++err_tick >= 10) {
            err_tick = 0;
            uart_puts("[ERR ] last=0x");
            uart_puthex8(CAN_GetLastErrorCode(CAN1));
            uart_puts(" tec=");
            uart_putu32(CAN_GetLSBTransmitErrorCounter(CAN1));
            uart_puts(" rec=");
            uart_putu32(CAN_GetReceiveErrorCounter(CAN1));
            uart_puts("\r\n");
        }
    }
}
