#include "gd32f30x_it.h"
#include "main.h"
#include "arm_math.h"
#include <rthw.h>
#include "rtthread.h"

#include "arm_math.h"

#include "WS281x.h"


cWS281x LED;

void CLOCK_Init(void);
void GPIO_Init(void);
void SPI_Init(void);
void DMA_Init(void);
void NVIC_Init(void);

static rt_thread_t LED_thread = RT_NULL;
static void LEDThread(void* parameter);

int main(void)
{
	CLOCK_Init();
	GPIO_Init();
	SPI_Init();
	DMA_Init();
	NVIC_Init();
	LED_thread =                          /* 线程控制块指针 */
	rt_thread_create( "led1",              /* 线程名字 */
										LEDThread,   /* 线程入口函数 */
										RT_NULL,             /* 线程入口函数参数 */
										256,                 /* 线程栈大小 */
										3,                   /* 线程的优先级 */
										20);                 /* 线程时间片 */
								 
	/* 启动线程，开启调度 */
	rt_thread_startup(LED_thread);
	return 0;
}


static void LEDThread(void* parameter)
{	
	LEDColor_t Color={0};
	uint8_t Color_buf[10]={0};
	rt_tick_t ticks;

	float VFie  = PI/127;
	float VTheta = PI/163;
	
	float Theta = PI/7;
	float Fie = 0;
	uint16_t CubeHalfHeight = 40;
	
	LED.cSPI::SPI_Init(SPI2,GPIOB,GPIO_PIN_6,DMA1,DMA_CH1);
	LED.Init(Color_buf);
	LED.LED_UpdateDMA(&Color,1);
	while (1)
	{	
		ticks = rt_tick_get();
	
		Theta += VTheta;
		Fie	  += VFie;
		
		if(Theta>2*PI){Theta-=(2*PI);}
		else if(Theta<0){Theta+=(2*PI);}
		
		if(Fie>2*PI){Fie-=(2*PI);}
		else if(Fie<0){Fie+=(2*PI);}
		
		Color.GRB[0] = CubeHalfHeight*(arm_sin_f32(Fie)+1);
		Color.GRB[1] = CubeHalfHeight*(arm_sin_f32(Theta)+1);
		Color.GRB[2] = CubeHalfHeight*(arm_cos_f32(Theta)+1);
		
		LED.LED_UpdateDMA(&Color,1); 
		rt_thread_delay_until(&ticks,10);
	}
}
void DMA1_Channel1_IRQHandler(void)
{
	LED.cSPI::IRQ_Tx();
}

void CLOCK_Init(void)
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
void GPIO_Init(void)
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
void DMA_Init(void)
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
}
void SPI_Init(void)
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
void NVIC_Init(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(DMA1_Channel1_IRQn,1,1);
}