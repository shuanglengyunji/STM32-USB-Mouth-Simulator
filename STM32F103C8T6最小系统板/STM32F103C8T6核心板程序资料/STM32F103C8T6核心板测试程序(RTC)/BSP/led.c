//=============================================================================
//�ļ�����:led.h
//�ļ���Ҫ:LED��ʼ��
//��汾��V3.5.0
//��Ȩ����:Դ�ع����� http://www.vcc-gnd.com/  ���� http://vcc-gnd.taobao.com/
//�汾����:2013-10-09 V1.0
//=============================================================================


//ͷ�ļ�
#include "led.h"


//=============================================================================
//��������: LED_GPIO_Config(void)
//���ܸ�Ҫ:LED����������
//��������:��
//��������:��
//=============================================================================
void LED_GPIO_Config(void)
{	
	//����һ��GPIO_InitTypeDef ���͵Ľṹ�壬���ֽ�GPIO_InitStructure 
	GPIO_InitTypeDef  GPIO_InitStructure;
	//ʹ��GPIOC������ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	//ѡ��Ҫ�õ�GPIO����		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_13;
	///��������ģʽΪ�������ģʽ			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	//���������ٶ�Ϊ50MHZ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//���ÿ⺯������ʼ��GPIO
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

