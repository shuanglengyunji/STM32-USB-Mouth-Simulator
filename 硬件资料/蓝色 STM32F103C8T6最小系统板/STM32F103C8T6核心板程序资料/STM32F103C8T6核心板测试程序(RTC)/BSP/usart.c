//=============================================================================
//�ļ�����:usart.c
//���ܸ�Ҫ:��printf�����ض���USART1�������Ϳ�����printf��������Ƭ��������
//         ��ӡ��PC�ϵĳ����ն˻򴮿ڵ������֡�
//��汾��V3.5.0
//��Ȩ����:Դ�ع�����www.vcc-gnd.com
//�汾����:2013-02-20 v1.0
//���Է�ʽ:J-LINK-OB
//=============================================================================

//ͷ�ļ�
#include "usart.h"


//=============================================================================
//��������:USART1_Config
//���ܸ�Ҫ:USART1 GPIO ����,����ģʽ���á�115200 8-N-1
//����˵��:��
//��������:��
//=============================================================================
void USART1_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;	
    USART_InitTypeDef USART_InitStructure;  //���崮�ڳ�ʼ���ṹ��
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);
  //��������ʹ��ʱ�ӣ������У�RCC_APB2Periph_USART1�Ǳز����ٵģ����˻��ʣ����ڴ����õ���PA9��
  //PA10����ʹ��ʱ������ʵ����USART1Ĭ�ϵľ���PA9��PA10��������һ�������ˣ���Ȼ��Ҫ�Ǽ���
  //���|RCC_APB2Periph_GPIOAҲ�ǲ�����ģ�ֻ���ظ��ˡ�
  /*  USART1_TX -> PA9 */			
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	       //ѡ�д���Ĭ������ܽ�         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�������������� 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����ܽ�9��ģʽ  
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //���ú������ѽṹ�����������г�ʼ��		   
  /*  USART1_RX ->	PA10*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600; //������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ1λ
  USART_InitStructure.USART_Parity = USART_Parity_No;		//У��λ ��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//ʹ�ܽ��պͷ�������

  USART_Init(USART1, &USART_InitStructure); //�����ϸ���ֵ�Ľṹ�����⺯��USART_Init���г�ʼ��
  USART_ClearFlag(USART1,USART_FLAG_TC);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_Cmd(USART1, ENABLE);//����USART1��ע��������RCC_APB2PeriphClockCmd()���õ�����
}

//=============================================================================
//��������:fputc
//���ܸ�Ҫ:�ض���c�⺯��printf��USART
//����˵��:��
//��������:��
//ע��   :��printf����
//=============================================================================
int fputc(int ch, FILE *f)
{
//��Printf���ݷ������� 
  USART_SendData(USART1, (unsigned char) ch);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
  return (ch);
}
//=============================================================================
//��������:USART1_Putc
//���ܸ�Ҫ:��USART1_Putc�������ݴ�ӡ������
//����˵��:��
//��������:��
//ע��   :��
//=============================================================================
void USART1_Putc(unsigned char c)
{
    USART_SendData(USART1, c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
}

//=============================================================================
//��������:USART1_Putc
//���ܸ�Ҫ:��USART1_Putc�������ݴ�ӡ������
//����˵��:��
//��������:��
//ע��    :��
//=============================================================================
void USART1_Puts(char * str)
{
    while(*str)
    {
        USART_SendData(USART1, *str++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}
