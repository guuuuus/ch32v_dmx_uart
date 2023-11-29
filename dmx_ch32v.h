/*
Guus 2023. channel vals are 0-511 (NOT 1-512)
tested on ch32v003
This library does not set the rs485 driver in the right direction. Do so before starting or permanent by hardware.
*/

#ifndef dmx_ch32v_h
#define dmx_ch32v_h
#define DMX_UART2 // defines the uart to use

#if defined(CH32V00X)
#include <ch32v00x.h>
#include <ch32v00x_gpio.h>
#include <ch32v00x_rcc.h>
#include <ch32v00x_usart.h>
#define GPIOPORT GPIOD
#define RCC_APB2Periph_GPIOPORT RCC_APB2Periph_GPIOD
#define GPIOTXPIN GPIO_Pin_5
#define GPIORXPIN GPIO_Pin_6

#define UARTNUM USART1
#define RCC_APB2Periph_USARTNUM RCC_APB2Periph_USART1
#define USARTNUM_IRQn USART1_IRQn

#if !defined(DMX_UART1)
#define DMX_UART1
#endif

#endif

#if defined(CH32X035)
#include <ch32x035.h>
#include <ch32x035_gpio.h>
#include <ch32x035_rcc.h>
#include <ch32x035_usart.h>
#endif

#if defined(CH32V10X)
#include <ch32v10x.h>
#include <ch32v10x_gpio.h>
#include <ch32v10x_rcc.h>
#include <ch32v10x_usart.h>
#endif

#if defined(CH32V20X)
#include <ch32v20x.h>
#include <ch32v20x_gpio.h>
#include <ch32v20x_rcc.h>
#include <ch32v20x_usart.h>

#endif

#if defined(CH32V30X)
#include <ch32v30x.h>
#include <ch32v30x_gpio.h>
#include <ch32v30x_rcc.h>
#include <ch32v30x_usart.h>
#endif

#if defined CH32V30X || CH32V20X || CH32V30X || CH32X035
#if defined(DMX_UART2)
#define GPIOPORT GPIOA
#define RCC_APB2Periph_GPIOPORT RCC_APB2Periph_GPIOA
#define GPIOTXPIN GPIO_Pin_2
#define GPIORXPIN GPIO_Pin_3

#define UARTNUM USART2
#define RCC_APB2Periph_USARTNUM RCC_APB1Periph_USART2
#define USARTNUM_IRQn USART2_IRQn

#elif defined(DMX_UART3)
#define GPIOPORT GPIOB
#define RCC_APB2Periph_GPIOPORT RCC_APB2Periph_GPIOB
#define GPIOTXPIN GPIO_Pin_9
#define GPIORXPIN GPIO_Pin_10

#define UARTNUM USART3
#define RCC_APB2Periph_USARTNUM RCC_APB1Periph_USART2
#define USARTNUM_IRQn USART3_IRQn

#else // use uart1
#define GPIOPORT GPIOA
#define RCC_APB2Periph_GPIOPORT RCC_APB2Periph_GPIOA
#define GPIOTXPIN GPIO_Pin_9
#define GPIORXPIN GPIO_Pin_10

#define UARTNUM USART1
#define RCC_APB2Periph_USARTNUM RCC_APB2Periph_USART1
#define USARTNUM_IRQn USART1_IRQn
#if !defined(DMX_UART1)
#define DMX_UART1
#endif
#endif
#endif

extern volatile unsigned char dmx_data[512];
extern volatile unsigned char dmx_startcode;
extern volatile unsigned char dmx_newdata;
extern volatile unsigned char dmx_pktsize;

typedef enum
{
    DMX_STOP = 0x00,
    DMX_IDLE = 0x01,
    DMX_BREAK = 0x02,
    DMX_START = 0x04,
    DMX_RUN = 0x05
} dmx_state_t;

typedef enum
{
    DMX_DIRNOTSET = 0x00,
    DMX_RX = 0x01,
    DMX_TX = 0x02,

} dmx_dir_t;

// avail dmx speeds in fps. Changing the speed wil change the idle time between frames.
typedef enum
{

    DMX_FPS_MIN,
    DMX_FPS_20,
    DMX_FPS_25,
    DMX_FPS_30,
    DMX_FPS_35,
    DMX_FPS_40,
    DMX_FPS_44,
    DMX_FPS_MAX,

} dmx_speed_fps;

// starts the transmitter
void dmx_beginTX();
#define dmx_transmitter_begin dmx_beginTX

// starts the receiver
void dmx_beginRX();
#define dmx_receiver_begin dmx_beginRX

// stop
void dmx_stop();

// sets (an approximation) of the breaktime in usec.
void dmx_setBreakTime(unsigned short usec);

// sets the speed of tx mode
void dmx_setFPS(dmx_speed_fps f);

// sets single value to addr
signed short dmx_setValue(unsigned short chan, unsigned char value);

// sets vals from arr to
signed short dmx_setValues(unsigned short startchan, unsigned char *p, unsigned short len);

// set the startcode
void dmx_setStartcode(unsigned char sc);

// get single value from a channel
signed short dmx_getValue(unsigned short chan);

// get values from dmx into arr
signed short dmx_getValues(unsigned short startchan, unsigned char *p, unsigned short len);

// returns last packet startcode
unsigned char dmx_getStartcode();

// returns 0xff if new frame is avail since last check
unsigned char dmx_newPacket();

// returns size of last received packet
unsigned short dmx_getPacketSize();

#if defined(DMX_UART1)
extern void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif

#if defined(DMX_UART2)
extern void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif

#if defined(DMX_UART3)
extern void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
#endif