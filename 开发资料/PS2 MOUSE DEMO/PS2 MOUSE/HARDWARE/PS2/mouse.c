#include "mouse.h"
#include "usart.h"
#include "lcd.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK mini�SSTM32������
//��� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

u8 MOUSE_ID;//����������ID 
PS2_Mouse MouseX;
//����MOUSE������	 
void Mouse_Data_Pro(void)
{			    			    
    MouseX.x_pos+=(signed char)PS2_DATA_BUF[1];
    MouseX.y_pos+=(signed char)PS2_DATA_BUF[2];
    MouseX.z_pos+=(signed char)PS2_DATA_BUF[3];		  
	MouseX.bt_mask=PS2_DATA_BUF[0]&0X07;//ȡ������
}

//��ʼ�����
//����:0,��ʼ���ɹ�
//����:�������
//CHECK OK 2010/5/2
u8 Init_Mouse(void)
{
	u8 t;
	
	//��ʼ��PS2������IO���жϵĳ�ʼ��
	PS2_Init();
	
	//�ȴ��ϵ縴λ���
	delay_ms(800);
	
	//��������ģʽ
	PS2_Status=CMDMODE;
	
	//��λ���
	t=PS2_Send_Cmd(PS_RESET); 
	if(t!=0)return 1;
	t=PS2_Get_Byte();			  
    if(t!=0XFA)return 2;
	
	//�ȴ���λ��� 
	t=0;
	while((PS2_Status&0x80)==0)
	{
		t++;
		delay_ms(10);
		if(t>50)return 3;	//��ʱ
	}
	
	PS2_Get_Byte();//�õ�0XAA
	PS2_Get_Byte();//�õ�ID 0X00
	
	//////////////////////////////////////////////////////////////
	//�������ģʽ�������ʼ������
	
	PS2_Send_Cmd(SET_SAMPLE_RATE);//�������ò�����
    if(PS2_Get_Byte()!=0XFA)return 4;//����ʧ��
	
	PS2_Send_Cmd(0XC8);//������200
    if(PS2_Get_Byte()!=0XFA)return 5;//����ʧ��
	
	PS2_Send_Cmd(SET_SAMPLE_RATE);//�������ò�����
    if(PS2_Get_Byte()!=0XFA)return 6;//����ʧ��
	PS2_Send_Cmd(0X64);//������100
	
    if(PS2_Get_Byte()!=0XFA)return 7;//����ʧ��
	PS2_Send_Cmd(SET_SAMPLE_RATE);//�������ò�����
    if(PS2_Get_Byte()!=0XFA)return 8;//����ʧ��
	
	PS2_Send_Cmd(0X50);//������80
    if(PS2_Get_Byte()!=0XFA)return 9;//����ʧ��
	
	//�������
	//////////////////////////////////////////////////////////////
	
	PS2_Send_Cmd(GET_DEVICE_ID); //��ȡID
    if(PS2_Get_Byte()!=0XFA)return 10;//����ʧ��
	MOUSE_ID=PS2_Get_Byte();//�õ�MOUSE ID	 

	PS2_Send_Cmd(SET_SAMPLE_RATE);//�ٴν������ò�����
    if(PS2_Get_Byte()!=0XFA)return 11;//����ʧ��
	
	PS2_Send_Cmd(0X0A);//������10
    if(PS2_Get_Byte()!=0XFA)return 12;//����ʧ��
	
	PS2_Send_Cmd(GET_DEVICE_ID); //��ȡID
    if(PS2_Get_Byte()!=0XFA)return 13;//����ʧ��
	MOUSE_ID=PS2_Get_Byte();//�õ�MOUSE ID		 

	PS2_Send_Cmd(SET_RESOLUTION);  //���÷ֱ���
    if(PS2_Get_Byte()!=0XFA)return 14;//����ʧ��   
 	
	PS2_Send_Cmd(0X03);//8��/mm
    if(PS2_Get_Byte()!=0XFA)return 15;//����ʧ�� 
	
	PS2_Send_Cmd(SET_SCALING11);   //�������ű���Ϊ1:1
    if(PS2_Get_Byte()!=0XFA)return 16;//����ʧ�� 
	  
 	PS2_Send_Cmd(SET_SAMPLE_RATE); //���ò�����
    if(PS2_Get_Byte()!=0XFA)return 17;//����ʧ��   
 	
	PS2_Send_Cmd(0X28);//40
    if(PS2_Get_Byte()!=0XFA)return 18;//����ʧ�� 
	   
	PS2_Send_Cmd(EN_DATA_REPORT);   //ʹ�����ݱ���
    if(PS2_Get_Byte()!=0XFA)return 19;//����ʧ��

	PS2_Status=MOUSE;//�������ģʽ
	return 0;//�޴���,��ʼ���ɹ�
}




