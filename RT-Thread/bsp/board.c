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
#include "Flash_GD.h"

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
static void USB_IRCCLK(void);
static void GPIO_Init(void);
static void SPI_Init(void);
static void UART_Init(void);
static void Timer_Init(void);
static void DMA_Init(void);
static void NVIC_Init(void);
static void Flash_Check(void);

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
#define RT_HEAP_SIZE 8192
static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 32K(8192 * 4)
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
	
	Flash_Check();
	CLOCK_Init();
	NVIC_Init();
	GPIO_Init();
	SPI_Init();
	UART_Init();
	Timer_Init();
	DMA_Init();
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while(delta < us_tick * us);
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
	
	rcu_periph_clock_enable(RCU_TIMER2);
	
	//rcu_periph_clock_enable(RCU_CAN0);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX, GPIO_PIN_12);//D+
	gpio_bit_reset(GPIOA,GPIO_PIN_12);
	rt_hw_us_delay(999);
	gpio_bit_set(GPIOA,GPIO_PIN_12);
	gpio_deinit(GPIOA);
	USB_IRCCLK();
	rcu_usb_clock_config(RCU_CKUSB_CKPLL_DIV1);
	rcu_periph_clock_enable(RCU_USBD);
	

}
static void GPIO_Init(void)
{
	/*IMU*/
	/*SPI0*/
	gpio_init(GPIOA, GPIO_MODE_IPU, 		GPIO_OSPEED_MAX, GPIO_PIN_3);//INT2
	gpio_init(GPIOA, GPIO_MODE_IPU, 		GPIO_OSPEED_MAX, GPIO_PIN_4);//INT1
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_5);//SCL
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, GPIO_PIN_6);//MISO
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_7);//MOSI
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_0);//CS
	
	/*两个外部中断引脚设置*/
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA,GPIO_PIN_SOURCE_3);
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA,GPIO_PIN_SOURCE_4);
	exti_init(EXTI_3,EXTI_INTERRUPT,EXTI_TRIG_FALLING);
	exti_init(EXTI_4,EXTI_INTERRUPT,EXTI_TRIG_FALLING);
	exti_interrupt_disable(EXTI_3);
	exti_interrupt_disable(EXTI_4);
	exti_interrupt_flag_clear(EXTI_3);
	exti_interrupt_flag_clear(EXTI_4);
	
	/*TIM2_CH3*/
	gpio_init(GPIOB, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_1);//HEAT
	
	/*UART0*/
	gpio_init(GPIOA, GPIO_MODE_AF_PP, 		GPIO_OSPEED_MAX, GPIO_PIN_9);//TX
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING,	GPIO_OSPEED_MAX, GPIO_PIN_10);//RX
	
	/*KEY*/
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_MAX, GPIO_PIN_4);
	
	/*LED*/
	/*SPI2*/
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);
	/*CAN*/
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
	spi_i2s_deinit(SPI2);
	
	spi_quad_disable(SPI0);
	/* SPI0 parameter config */
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_16;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	//18MHz
	spi_init(SPI0, &spi_init_struct);
	//2.25MHz
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
static void Timer_Init(void)
{
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
	
	timer_deinit(TIMER2);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 287;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);
	
	 /* CH3 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2,TIMER_CH_3,&timer_ocintpara);
    
    /* CH3 configuration in PWM mode0,duty cycle 0% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
	
	 /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* auto-reload preload enable */
    timer_enable(TIMER2);
}
static void NVIC_Init(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
	
    nvic_irq_enable(DMA1_Channel1_IRQn,2,1);//SPI2 RX
	nvic_irq_enable(DMA0_Channel3_IRQn,1,1);//USART0 TX
	nvic_irq_enable(USART0_IRQn,1,1);//USART0 RX
	nvic_irq_enable(EXTI3_IRQn,0,1);//INT2
	nvic_irq_enable(EXTI4_IRQn,0,0);//INT1

    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1U, 2U);
    nvic_irq_enable(USBD_HP_CAN0_TX_IRQn, 1U, 2U);
}
static void USB_IRCCLK(void)
{
	uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;
	/*启动内部IRC48M振荡器*/
	RCU_ADDCTL |= RCU_ADDCTL_IRC48MEN;
	/*等待振荡器稳定*/
	do{
        timeout++;
        stab_flag = (RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB);
    }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));
	
	/*使用CTC校准振荡器*/
	/*需要外部参考时钟源*/
	
	/*设置USB输入时钟*/
    rcu_ck48m_clock_config(RCU_CK48MSRC_IRC48M);
}
static void Flash_Check(void)
{
	rcu_periph_clock_enable(RCU_CRC);
	
	uint32_t buf[5];
	fmc_read_u32(FLASH_USERDATA_ADDRESS,buf,5);

	crc_data_register_reset();
	if(buf[4]!=crc_block_data_calculate(buf,4))
	{
		fmc_erase_pages(FLASH_USERDATA_ADDRESS,1);
	
		buf[0]=0x01;
		buf[1]=0x00;
		buf[2]=0x00;
		buf[3]=0x00;
		crc_data_register_reset();
		buf[4]=crc_block_data_calculate(buf,4);
	
		fmc_program(FLASH_USERDATA_ADDRESS,buf,5);
		fmc_program_check(FLASH_USERDATA_ADDRESS,5,buf);
	}
}