#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"   
#include "mouse.h"   
//ALIENTEK Mini STM32�����巶������25
//PS2���ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾ 

//��ʾ��������ֵ
//x,y:��LCD����ʾ������λ��
//pos:����ֵ
void Mouse_Show_Pos(u16 x,u16 y,short pos)
{
	if(pos<0)
	{			  
		LCD_ShowChar(x,y,'-',16,0);		//��ʾ����
		pos=-pos;						//תΪ����
	}else LCD_ShowChar(x,y,' ',16,0);	//ȥ������
	LCD_ShowNum(x+8,y,pos,5,16);		//��ʾֵ				  
}
int main(void)
{
	u8 errcnt=0;		
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(9600);	 	//���ڳ�ʼ��Ϊ9600
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	LCD_Init();			   	//��ʼ��LCD   
	
	//��ʼ�����
 	while( Init_Mouse() )	//�������Ƿ���λ.	
	{
		LCD_ShowString(60,130,200,16,16,(u8*)"Mouse Error");
		delay_ms(400);
		LCD_Fill(60,130,239,130+16,WHITE);	 
		delay_ms(100);
	}								   
	
	//��ʾMOUSE_ID
 	LCD_ShowNum(132,150,MOUSE_ID,3,16);//���ģʽ
	
	while(1)
	{
		//����յ�������
		if(PS2_Status&0x80)//�õ���һ������
		{
			//��ʾ���ص���ֵ
			LCD_ShowNum(56+30,170,PS2_DATA_BUF[0],3,16);//���ģʽ
			LCD_ShowNum(56+30,186,PS2_DATA_BUF[1],3,16);//���ģʽ
			LCD_ShowNum(56+30,202,PS2_DATA_BUF[2],3,16);//���ģʽ
			if(MOUSE_ID==3)LCD_ShowNum(56+30,218,PS2_DATA_BUF[3],3,16);//���ģʽ

			//��������
			Mouse_Data_Pro();
			
			//��ʾ����������
			Mouse_Show_Pos(146+30,170,MouseX.x_pos);				//X����
			Mouse_Show_Pos(146+30,186,MouseX.y_pos);				//Y����
			if(MOUSE_ID==3)Mouse_Show_Pos(146+30,202,MouseX.z_pos);	//����λ��

			//���
		    if(MouseX.bt_mask&0x01)
				LCD_ShowString(146+30,218,200,16,16,(u8*)"LEFT"); 
			else 
				LCD_ShowString(146+30,218,200,16,16,(u8*)"    "); 
			
			//�Ҽ�
		    if(MouseX.bt_mask&0x02)
				LCD_ShowString(146+30,234,200,16,16,(u8*)"RIGHT"); 
			else 
				LCD_ShowString(146+30,234,200,16,16,(u8*)"     ");
			
			//�м�
			if(MouseX.bt_mask&0x04)
				LCD_ShowString(146+30,250,200,16,16,(u8*)"MIDDLE"); 
			else 
				LCD_ShowString(146+30,250,200,16,16,(u8*)"      ");
			
			PS2_Status=MOUSE;
			PS2_En_Data_Report();//ʹ�����ݱ���
		}
		else if(PS2_Status&0x40)	//���û���յ�����
		{
			errcnt++;
			PS2_Status=MOUSE;
			LCD_ShowNum(86+30,234,errcnt,3,16);//���ģʽ
		}
	}
}
