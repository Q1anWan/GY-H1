/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
 
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include "main.h"

#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))

// Updates the variable SystemCoreClock and must be called 
// whenever the core clock is changed during program execution.
extern void SystemCoreClockUpdate(void);

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;

static void CLOCK_Init(void);
static void GPIO_Init(void);
static void SPI_Init(void);
static void UART_Init(void);
static void DMA_Init(void);
static void NVIC_Init(void);

static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    
    return 0;
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 4096
static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 16K(4096 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    /* System Clock Update */
    SystemCoreClockUpdate();
    
    /* System Tick Configuration */
    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
		
	CLOCK_Init();
	GPIO_Init();
	SPI_Init();
	UART_Init();
	DMA_Init();
	NVIC_Init();	
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

static void CLOCK_Init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_AF);

	rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
	
	rcu_periph_clock_enable(RCU_SPI0);
	rcu_periph_clock_enable(RCU_SPI2);
	
	rcu_periph_clock_enable(RCU_USART0);
	
	rcu_periph_clock_enable(RCU_CAN0);
	
	rcu_periph_clock_enable(RCU_TIMER2);
}
static void GPIO_Init(void)
{
	/*IMU*/
	/*SPI0*/
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, GPIO_PIN_3);//INT2
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, GPIO_PIN_4);//INT1
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_5);//SCL
	gpio_init(GPIOA, GPIO_MODE_AIN, 		GPIO_OSPEED_MAX, GPIO_PIN_6);//MISO
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_7);//MOSI
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_0);//CS
	/*TIM2_CH3*/
	gpio_init(GPIOB, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_1);//HEAT
	
	/*UART0*/
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_9);//TX
	gpio_init(GPIOA, GPIO_MODE_AIN,			GPIO_OSPEED_MAX, GPIO_PIN_10);//RX
	
	/*KEY*/
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_MAX, GPIO_PIN_4);
	
	/*LED*/
	/*SPI2*/
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_6);
	/*CAM*/
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_MAX, GPIO_PIN_8);//RX
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_9);//TX
}
static void DMA_Init(void)
{
	dma_parameter_struct dma_init_struct;
    
    /* SPI2 transmit dma config:DMA1-DMA_CH1  */
    dma_deinit(DMA1, DMA_CH1);
    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI2);
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init(DMA1, DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA1, DMA_CH1);
    dma_memory_to_memory_disable(DMA1, DMA_CH1);
	dma_interrupt_enable(DMA1, DMA_CH1, DMA_INT_FTF);
	
	 /* UARTO transmit dma config:DMA0-DMA_CH3  */
    dma_deinit(DMA0, DMA_CH3);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART0);
	dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.priority     = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);
    dma_memory_to_memory_disable(DMA0, DMA_CH3);
	dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
	
	/* UARTO recieve dma config:DMA0-DMA_CH4  */
    dma_deinit(DMA0, DMA_CH4);
    dma_init_struct.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH4);
    dma_memory_to_memory_disable(DMA0, DMA_CH4);

}
static void SPI_Init(void)
{
	spi_parameter_struct  spi_init_struct;
	
	spi_i2s_deinit(SPI0); 
	spi_quad_disable(SPI0);
	spi_i2s_deinit(SPI2);

	/* SPI0 parameter config */
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_16;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	//15MHz
	spi_init(SPI0, &spi_init_struct);
	//3.75MHz
	spi_init_struct.prescale        	 = SPI_PSC_64;
	spi_init(SPI2, &spi_init_struct);
	spi_dma_enable(SPI2,SPI_DMA_TRANSMIT);
}
static void UART_Init(void)
{
	usart_deinit(USART0);
    usart_baudrate_set(USART0, 512000U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
	usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
	usart_interrupt_enable(USART0,USART_INT_IDLE);
    usart_enable(USART0);
}
static void NVIC_Init(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	
    nvic_irq_enable(DMA1_Channel1_IRQn,2,1);//SPI2 RX
	nvic_irq_enable(DMA0_Channel3_IRQn,0,0);//USART0 TX
	nvic_irq_enable(DMA0_Channel4_IRQn,0,1);//USART0 RX
}