#include "interface.h" 

/* UART2    9600
   PD5  TX
   PD6  RX
   PA3  INT
*/

void CH375_PORT_PRE_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure USART2 Rx (PA.03 as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_3);	//��λ�ڼ䣬CH375��Tx��Ӧ��Ϊ�ߣ��������ܽ��봮��ģʽ
}

void CH375_COM_INIT( u32 band )
{	
	// USART2
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//���ȹص�����
	USART_Cmd(USART2, DISABLE);
	
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART2 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure USART2 Rx (PA.03 as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = band;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

//�ӿ�Ӳ����ʼ��
void CH375_PORT_INIT( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// GPIO INT -- PA0
	
	/*���������˿ڣ�PA����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	///////////////////////////////////////////////////////////////////
	
	CH375_COM_INIT(9600);	//�ϵ��ʼ��������Ϊ9600
	
}

//д����
void xWriteCH375Cmd( uint8_t cmd ) 		/* ��CH375������˿�д������,���ڲ�С��2uS,�����Ƭ���Ͽ�����ʱ */	
{
	while((USART2->SR&0x40)==0);
	USART2->DR = ((uint16_t)cmd | 0x0100 );
	delay_us(2);
}

//д����
void xWriteCH375Data( uint8_t dat ) 	/* ��CH375�����ݶ˿�д������,���ڲ�С��1uS,�����Ƭ���Ͽ�����ʱ */
{
	while((USART2->SR&0x40)==0);   
	USART2->DR = (uint16_t)dat;
	delay_us(1);
}

//������
uint8_t xReadCH375Data(void) 			/* ��CH375�����ݶ˿ڶ�������,���ڲ�С��1uS,�����Ƭ���Ͽ�����ʱ */
{
	uint32_t i;
	for(i=0;i<500000;i++)                      //����500ms���ڽ��ճ�ʱ
	{
		if(USART2->SR&0x20)    //RXNE
		{
			return ((uint8_t)USART2->DR);
		}
		delay_us(1);	//��������Ծ�ȷ��1usȷ��ʱ��
	}
	return ERR_USB_UNKNOWN;
}

/* ��ѯCH375�ж�(INT#�͵�ƽ) */
uint8_t	Query375Interrupt( void )
{
	if( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1) 
	{
		return FALSE ;
	}
	else
	{
		return TRUE ;
	}
}

/* CH375��ʼ������ */
uint8_t	mInitCH375Host( void )  /* ��ʼ��CH375 */
{
	uint8_t	res;
	
	//CH375��TX��������
	CH375_PORT_PRE_INIT();
	
	/* �ϵ��������ʱ50ms���� */
	delay_ms(50);        
	
	/* �ӿ�Ӳ����ʼ�� */
	CH375_PORT_INIT( );
	
	/* ���Ե�Ƭ����CH375֮���ͨѶ�ӿ� */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
		return( ERR_USB_UNKNOWN );  /* ͨѶ�ӿڲ�����,����ԭ����:�ӿ������쳣,�����豸Ӱ��(Ƭѡ��Ψһ),���ڲ�����,һֱ�ڸ�λ,���񲻹��� */
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	/* �����ڲ�������� */
	xWriteCH375Cmd( CMD_SET_BAUDRATE );  //����USB������
	xWriteCH375Data( 0x03 );	//0x03 0xCC -- 115200     0x03 0xF3 -- 460800
	xWriteCH375Data( 0xCC );
	CH375_COM_INIT(115200);		//�л����ش��ڲ�����
	delay_ms(1);	//��ʱ1ms
	res = xReadCH375Data( );
	if ( res != CMD_RET_SUCCESS ) 
		return( ERR_USB_UNKNOWN );
	
	/* ���²��Ե�Ƭ����CH375֮���ͨѶ�ӿ� */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
		return( ERR_USB_UNKNOWN );  /* ͨѶ�ӿڲ�����,����ԭ����:�ӿ������쳣,�����豸Ӱ��(Ƭѡ��Ψһ),���ڲ�����,һֱ�ڸ�λ,���񲻹��� */
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	/* �豸USB����ģʽ */
	xWriteCH375Cmd( CMD_SET_USB_MODE );  
	xWriteCH375Data( 0x06 );
	delay_us( 20 );
	res = xReadCH375Data( );
	if ( res == CMD_RET_SUCCESS ) 
		return( USB_INT_SUCCESS );
	else 
		return( ERR_USB_UNKNOWN );  /* ����ģʽ���� */		
}
