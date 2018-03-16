
#include "interface.h" 

#include "stdio.h"
/* �ӿ��������� */
/* A0    --- PF0
   D0-D1 --- PD14-PD15
   D2-D3 --- PD0-PD1
   D4-D7 --- PE7-PE10
   CS    --- PG9(NE2) BANK1��NE1~NE4
   RD    --- PD4(NOE)
   WR    --- PD5(NWE)
   INT   --- PA3  
*/

#define CH375_INT_WIRE			PAin(3)
#define CH37x_CMD_PORT  *((volatile unsigned char  *)(0x64000001)) 	       /* CH375����˿ڵ�I/O��ַ */
#define CH37x_DAT_PORT	*((volatile unsigned char  *)(0x64000000)) 	       /* CH375���ݶ˿ڵ�I/O��ַ */


void CH375_PORT_INIT( void )
{
	//��ʱ��
	RCC ->AHB1ENR |= (0xF<<3);                               //�����˿�ʹ�� D E F G
	RCC ->AHB1ENR |= (1<<0);                                 //ʹ�� PortA
	RCC ->AHB3ENR |= 1<<0;                                   //ʹ��FMC
	//����IO�븴��ӳ��
	GPIO_Set(GPIOF,1<<0 ,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //AO
	GPIO_Set(GPIOD,(3<<14)+(3<<0) ,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //D0-D3
	GPIO_Set(GPIOE,(0xF<<7) ,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //D4-D7
	GPIO_Set(GPIOD,(3<<4) ,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //RD WR
	GPIO_Set(GPIOG,(1<<9) ,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //CS
	GPIO_Set(GPIOA,1<<3 ,GPIO_MODE_IN,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);  //INT
	GPIO_AF_Set(GPIOF,0,12);
	GPIO_AF_Set(GPIOD,14,12);
	GPIO_AF_Set(GPIOD,15,12);
	GPIO_AF_Set(GPIOD,0,12);
	GPIO_AF_Set(GPIOD,1,12);
	GPIO_AF_Set(GPIOE,7,12);
	GPIO_AF_Set(GPIOE,8,12);
	GPIO_AF_Set(GPIOE,9,12);
	GPIO_AF_Set(GPIOE,10,12);	
	GPIO_AF_Set(GPIOD,4,12);
	GPIO_AF_Set(GPIOD,5,12);
	GPIO_AF_Set(GPIOG,9,12);
	//����FMC
	FMC_Bank1->BTCR[2] = 0x00000000;
	FMC_Bank1->BTCR[3] = 0x00000000;
	FMC_Bank1E->BWTR[2] = 0x00000000;
	FMC_Bank1->BTCR[2]|=1<<14;		//��дʹ�ò�ͬ��ʱ��
	FMC_Bank1->BTCR[2]|=1<<12;		//�洢��дʹ��
	
	FMC_Bank1->BTCR[3]|=(16<<8);    //�� ���ݽ���ʱ�䣨70ns��
	FMC_Bank1->BTCR[3]|=(2<<0);    //�� ��ַ����ʱ�䣨10ns��
	
	FMC_Bank1E->BWTR[2]|=16<<8;   	//д���ݱ���ʱ��(DATAST)	
	FMC_Bank1E->BWTR[2]|=2<<0;		//д��ַ����ʱ��(ADDSET)

	FMC_Bank1->BTCR[2]|=1<<0;		//ʹ��BANK1������2	
}
//д����
void xWriteCH375Cmd( uint8_t cmd ) { 				 /* ��CH375������˿�д������,���ڲ�С��2uS,�����Ƭ���Ͽ�����ʱ */
	CH37x_CMD_PORT=cmd;
	delay_us(2);
}
//д����
void xWriteCH375Data( uint8_t dat ) { 				 /* ��CH375�����ݶ˿�д������,���ڲ�С��1uS,�����Ƭ���Ͽ�����ʱ */
	CH37x_DAT_PORT=dat;
	delay_us(1);
}
//������
uint8_t xReadCH375Data(void) {  				     /* ��CH375�����ݶ˿ڶ�������,���ڲ�С��1uS,�����Ƭ���Ͽ�����ʱ */
	delay_us(1);
	return( CH37x_DAT_PORT );
}
//��״̬
uint8_t xReadCH375Status() {  				         /* ��CH375�����ݶ˿ڶ�������,���ڲ�С��1uS,�����Ƭ���Ͽ�����ʱ */
	delay_us(1);
	return( CH37x_CMD_PORT );
}


/* ��ѯCH375�ж�(INT#�͵�ƽ) */
uint8_t	Query375Interrupt( void )
{
	/* ���������CH375���ж�������ֱ�Ӳ�ѯ�ж����� */
	/* ���δ����CH375���ж��������ѯ״̬�˿� */
#ifdef	CH375_INT_WIRE
	return( CH375_INT_WIRE ? FALSE : TRUE ); 
#else
	return( xReadCH375Status( ) & 0x80 ? FALSE : TRUE );  
#endif	
}

/* CH375��ʼ������ */
uint8_t	mInitCH375Host( void )  /* ��ʼ��CH375 */
{
	uint8_t	res;	
	delay_ms(50);        /* �ϵ��������ʱ50ms���� */
	CH375_PORT_INIT( );  /* �ӿ�Ӳ����ʼ�� */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  /* ���Ե�Ƭ����CH375֮���ͨѶ�ӿ� */
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
	{
		printf ("check erro:%02x\n",(uint16_t)res );
		return( ERR_USB_UNKNOWN );  /* ͨѶ�ӿڲ�����,����ԭ����:�ӿ������쳣,�����豸Ӱ��(Ƭѡ��Ψһ),���ڲ�����,һֱ�ڸ�λ,���񲻹��� */
	}
	xWriteCH375Cmd( CMD_SET_USB_MODE );  /* �豸USB����ģʽ */
	xWriteCH375Data( 0x06 );
	delay_us( 20 );
	res = xReadCH375Data( );
	if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
	else 
	{
		printf ("mode erro:%02x\n",(uint16_t)res );
		return( ERR_USB_UNKNOWN );  /* ����ģʽ���� */	
	}
	
}





