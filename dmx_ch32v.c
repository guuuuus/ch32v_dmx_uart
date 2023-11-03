#include <dmx_ch32v.h>
// #include "ltc_ch32v.h"

void dmx_txIRQ(void);
void dmx_rxIRQ(void);
void dmx_irqplaceholder();

volatile unsigned char dmx_data[512];
volatile unsigned short dmx_poscoutner = 0;
volatile unsigned char dmx_bkcounter = 0;
unsigned char dmx_bktime = 6; // can be changed in, 4 sems to go well withing spec,
volatile unsigned short dmx_idlecounter = 0;
unsigned short dmx_idletime = 2;

void (*dmx_interrupt)(void) = &dmx_irqplaceholder;

volatile unsigned char dmx_startcode = 0x00;
volatile unsigned char dmx_newdata = 0x00;
volatile unsigned char dmx_pktsize;
volatile dmx_state_t dmx_state = DMX_IDLE;
volatile dmx_dir_t dmx_dir;

USART_InitTypeDef dmx_uartinit;
GPIO_InitTypeDef dxm_gpiotx;

void dmx_irqplaceholder()
{
    USART_ClearITPendingBit(UARTNUM, 0xffff);
}

void dmx_beginRX()
{
    GPIO_InitTypeDef dxm_gpiorx;

    // dir for it
    dmx_dir = DMX_RX;

    // peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOPORT, ENABLE);
#if defined(DMX_UART1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTNUM, ENABLE);
#endif

    dxm_gpiorx.GPIO_Pin = GPIORXPIN;
    dxm_gpiorx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    dxm_gpiorx.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOPORT, &dxm_gpiorx);

    dmx_uartinit.USART_BaudRate = 250000;
    dmx_uartinit.USART_WordLength = USART_WordLength_8b;
    dmx_uartinit.USART_StopBits = USART_StopBits_1; // is specified with 2, but the second is just idle time in this case.
    dmx_uartinit.USART_Parity = USART_Parity_No;
    dmx_uartinit.USART_Mode = USART_Mode_Rx;
    dmx_uartinit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    dmx_interrupt = &dmx_rxIRQ;

    USART_DeInit(UARTNUM);
    USART_Init(UARTNUM, &dmx_uartinit);
    USART_ITConfig(UARTNUM, USART_IT_RXNE, ENABLE);

    NVIC_EnableIRQ(USARTNUM_IRQn);
    // turn on preemption, set prio low
    NVIC_SetPriority(USARTNUM_IRQn, 0xe0);

    USART_Cmd(UARTNUM, ENABLE);
}

void dmx_beginTX()
{
    // dir for it
    dmx_dir = DMX_TX;

    // peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOPORT, ENABLE);
#if defined(DMX_UART1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTNUM, ENABLE);
#elif defined(DMX_UART2) || (DMX_UART3)
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_USARTNUM, ENABLE);
#endif

    // setup the gpio

    dxm_gpiotx.GPIO_Pin = GPIOTXPIN;
    // dxm_gpiotx.GPIO_Mode = GPIO_Mode_AF_PP;
    dxm_gpiotx.GPIO_Mode = GPIO_Mode_Out_PP;
    dxm_gpiotx.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOPORT, &dxm_gpiotx);

    // seting the config
    dmx_uartinit.USART_BaudRate = 250000;
    dmx_uartinit.USART_WordLength = USART_WordLength_8b;
    dmx_uartinit.USART_StopBits = USART_StopBits_2;
    dmx_uartinit.USART_Parity = USART_Parity_No;
    dmx_uartinit.USART_Mode = USART_Mode_Tx;
    dmx_uartinit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    dmx_interrupt = &dmx_txIRQ;

    // USART_DeInit(UARTNUM);
    USART_Init(UARTNUM, &dmx_uartinit);
    USART_ITConfig(UARTNUM, USART_IT_TXE, ENABLE);

    NVIC_EnableIRQ(USARTNUM_IRQn);
    // turn on preemption, set prio low
    NVIC_SetPriority(USARTNUM_IRQn, 0xe0);


    USART_Cmd(UARTNUM, ENABLE);
}

void dmx_stop()
{
    // itcs
    // USART_ITConfig(UARTNUM, USART_IT_TXE, DISABLE);
    // USART_ITConfig(UARTNUM, USART_IT_RXNE, DISABLE);
    // // uart peri
    // USART_Cmd(UARTNUM, DISABLE);

//     // peripheral clock
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOPORT, DISABLE);
// #if defined(DMX_UART1)
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTNUM, DISABLE);
// #elif defined(DMX_UART2) || (DMX_UART3)
//     RCC_APB1PeriphClockCmd(RCC_APB2Periph_USARTNUM, DISABLE);
// #endif
    NVIC_DisableIRQ(USARTNUM_IRQn);

    dmx_state = DMX_STOP;
    dmx_dir = DMX_DIRNOTSET;
}

void dmx_setBreakTime(unsigned short usec)
{

    if (usec < 132)
    {
        dmx_bktime = 3;
    }
    else if (usec > 10000)
    {
        dmx_bktime = 255;
    }
    else
    {
        dmx_bktime = usec / 44;
    }
}

void dmx_setFPS(dmx_speed_fps f)
{
    switch (f)
    {
    case DMX_FPS_20:
    case DMX_FPS_MIN:
        dmx_bktime = 12;
        dmx_idletime = 605;
        break;

    case DMX_FPS_25:
        dmx_bktime = 12;
        dmx_idletime = 380;
        break;

    case DMX_FPS_30:
        dmx_bktime = 10;
        dmx_idletime = 230;
        break;

    case DMX_FPS_35:
        dmx_bktime = 8;
        dmx_idletime = 122;
        break;

    case DMX_FPS_40:
        dmx_bktime = 6;
        dmx_idletime = 45;
        break;

    case DMX_FPS_44:
    case DMX_FPS_MAX:
        dmx_bktime = 5;
        dmx_idletime = 2;
        break;

    default:
        dmx_bktime = 12;
        dmx_idletime = 230;
        break;
    }
}

signed short dmx_setValue(unsigned short chan, unsigned char value)
{
    if ((chan > 511) || (dmx_dir == DMX_RX)) // out of range or in rx mode
        return -1;
    dmx_data[chan] = value;
    return 0;
}

signed short dmx_setValues(unsigned short startchan, unsigned char *p, unsigned short len)
{
    if (((startchan + len) > 511) || (dmx_dir == DMX_RX)) // out of range or in rx mode
        return -1;
    for (unsigned short i = 0; i < len; i++)
    {
        dmx_data[startchan + i] = p[i];
    }
    return 0;
}

void dmx_setStartcode(unsigned char sc)
{
    dmx_startcode = sc;
}

signed short dmx_getValue(unsigned short chan)
{
    if (chan > 511) // out of range
    {
        return -1;
    }
    return (0x00ff & dmx_data[chan]);
}

signed short dmx_getValues(unsigned short startchan, unsigned char *p, unsigned short len)
{
    if ((startchan + len) > 511) // out of range
        return -1;
    for (unsigned short i = 0; i < len; i++)
    {
        p[i] = dmx_data[startchan + i];
    }
    return 0;
}

unsigned char dmx_getStartcode()
{
    return dmx_startcode;
}

unsigned char dmx_newPacket()
{
    unsigned char r = 0;
    if (dmx_newdata)
    {
        r = 0xff;
        dmx_newdata = 0x00;
    }
    return r;
}

unsigned short dmx_getPacketSize()
{
    return dmx_pktsize;
}

void dmx_txIRQ()
{
    USART_ClearITPendingBit(UARTNUM, 0xffff);
    switch (dmx_state)
    {
    case DMX_IDLE:
        // wait one txe cyle for addr 512 to complete sending
        if (dmx_idlecounter == 1)
        {
            GPIO_WriteBit(GPIOPORT, GPIOTXPIN, 1);
            dxm_gpiotx.GPIO_Mode = GPIO_Mode_Out_PP;
            dxm_gpiotx.GPIO_Speed = GPIO_Speed_2MHz;
            GPIO_Init(GPIOPORT, &dxm_gpiotx);
        }
        if ((dmx_idlecounter > dmx_idletime) && (dmx_idletime > 1))
        {
            dmx_state = DMX_BREAK;
        }
        dmx_idlecounter++;
        USART_SendData(UARTNUM, 0xff);
        break;

    case DMX_BREAK:
        // reset counters
        dmx_poscoutner = 0;
        dmx_idlecounter = 0;
        // set line low
        GPIO_WriteBit(GPIOPORT, GPIOTXPIN, 0);

        dmx_bkcounter++;

        if (dmx_bkcounter > dmx_bktime)
        {
            dmx_state = DMX_START;
            // next byte is going to be in the reg while switching the gpio from io to uart igues...
            USART_SendData(UARTNUM, 0xff);
        }
        else
        {
            USART_SendData(UARTNUM, 0x00);
        }

        break;

    case DMX_START:
        dmx_bkcounter = 0;

        dxm_gpiotx.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOPORT, &dxm_gpiotx);
        USART_SendData(UARTNUM, dmx_startcode);

        dmx_state = DMX_RUN;
        break;

    case DMX_RUN:
        USART_SendData(UARTNUM, dmx_data[dmx_poscoutner]);
        dmx_poscoutner++;
        if (dmx_poscoutner >= 512)
        {
            dmx_state = DMX_IDLE;
            dmx_newdata = 0xff;
            GPIO_WriteBit(GPIOPORT, GPIOTXPIN, 1);
        }
        break;

    default:
        dmx_state = DMX_IDLE;
        break;
    }
}

void dmx_rxIRQ()
{
    unsigned char fe = USART_GetFlagStatus(UARTNUM, USART_FLAG_FE);
    unsigned char data = (0xff & USART_ReceiveData(UARTNUM));

    if (fe)
        dmx_state = DMX_BREAK;

    switch (dmx_state)
    {
    case DMX_BREAK:
        if (fe)
        {
            dmx_poscoutner = 0;
            dmx_state = DMX_START;
            dmx_newdata = 0xff;
        }
        break;

    case DMX_START:
        dmx_startcode = data;
        dmx_state = DMX_RUN;
        break;

    case DMX_RUN:
        dmx_data[dmx_poscoutner] = data;
        dmx_poscoutner++;
        if (dmx_poscoutner >= 512)
        {
            dmx_state = DMX_BREAK;
        }
        break;
    default:
        break;
    }
}
#if defined(DMX_UART1)
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler()
{
    dmx_interrupt();
}
#endif

#if defined(DMX_UART2)
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler()
{
    dmx_interrupt();
}
#endif

#if defined(DMX_UART3)
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler()
{
    dmx_interrupt();
}
#endif