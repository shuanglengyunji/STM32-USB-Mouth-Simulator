#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"   
#include "mouse.h"   
//ALIENTEK Mini STM32开发板范例代码25
//PS2鼠标实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司 

//显示鼠标的坐标值
//x,y:在LCD上显示的坐标位置
//pos:坐标值
void Mouse_Show_Pos(u16 x,u16 y,short pos)
{
	if(pos<0)
	{			  
		LCD_ShowChar(x,y,'-',16,0);		//显示负号
		pos=-pos;						//转为正数
	}else LCD_ShowChar(x,y,' ',16,0);	//去掉负号
	LCD_ShowNum(x+8,y,pos,5,16);		//显示值				  
}
int main(void)
{
	u8 errcnt=0;		
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 	//串口初始化为9600
	LED_Init();		  		//初始化与LED连接的硬件接口
	LCD_Init();			   	//初始化LCD   
	
	//初始化鼠标
 	while( Init_Mouse() )	//检查鼠标是否在位.	
	{
		LCD_ShowString(60,130,200,16,16,(u8*)"Mouse Error");
		delay_ms(400);
		LCD_Fill(60,130,239,130+16,WHITE);	 
		delay_ms(100);
	}								   
	
	//显示MOUSE_ID
 	LCD_ShowNum(132,150,MOUSE_ID,3,16);//填充模式
	
	while(1)
	{
		//如果收到了数据
		if(PS2_Status&0x80)//得到了一次数据
		{
			//显示返回的数值
			LCD_ShowNum(56+30,170,PS2_DATA_BUF[0],3,16);//填充模式
			LCD_ShowNum(56+30,186,PS2_DATA_BUF[1],3,16);//填充模式
			LCD_ShowNum(56+30,202,PS2_DATA_BUF[2],3,16);//填充模式
			if(MOUSE_ID==3)LCD_ShowNum(56+30,218,PS2_DATA_BUF[3],3,16);//填充模式

			//处理数据
			Mouse_Data_Pro();
			
			//显示处理后的坐标
			Mouse_Show_Pos(146+30,170,MouseX.x_pos);				//X坐标
			Mouse_Show_Pos(146+30,186,MouseX.y_pos);				//Y坐标
			if(MOUSE_ID==3)Mouse_Show_Pos(146+30,202,MouseX.z_pos);	//滚轮位置

			//左键
		    if(MouseX.bt_mask&0x01)
				LCD_ShowString(146+30,218,200,16,16,(u8*)"LEFT"); 
			else 
				LCD_ShowString(146+30,218,200,16,16,(u8*)"    "); 
			
			//右键
		    if(MouseX.bt_mask&0x02)
				LCD_ShowString(146+30,234,200,16,16,(u8*)"RIGHT"); 
			else 
				LCD_ShowString(146+30,234,200,16,16,(u8*)"     ");
			
			//中键
			if(MouseX.bt_mask&0x04)
				LCD_ShowString(146+30,250,200,16,16,(u8*)"MIDDLE"); 
			else 
				LCD_ShowString(146+30,250,200,16,16,(u8*)"      ");
			
			PS2_Status=MOUSE;
			PS2_En_Data_Report();//使能数据报告
		}
		else if(PS2_Status&0x40)	//如果没有收到数据
		{
			errcnt++;
			PS2_Status=MOUSE;
			LCD_ShowNum(86+30,234,errcnt,3,16);//填充模式
		}
	}
}
