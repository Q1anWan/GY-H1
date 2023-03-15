#include "Controller.h"
/*系统设置进程控制指针*/
rt_thread_t Config_thread = RT_NULL;
rt_mailbox_t Config_mailbox = RT_NULL;

/*
	系统设置控制进程
	接收到来自串口或者CDC的控制命令后，修改系统设置
*/
void ConfigThread(void* parameter)
{
	uint32_t RecBuf = 0;
	for(;;)
	{
		rt_mb_recv(Config_mailbox,(rt_ubase_t*)&RecBuf,RT_WAITING_FOREVER);
		switch(RecBuf>>8)
		{
			case 0x00:
				switch(RecBuf&0xFF)
				{
					case 0x00://重启
						NVIC_SystemReset();
						while(1);
					break;
					case 0x01:
						
					break;				
					case 0x02:
						
					break;
					case 0x03:
						
					break;				
				}
			break;
			default:
				
				
			break;
		
		}
	}
}