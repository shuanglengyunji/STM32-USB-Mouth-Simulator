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
	
	GPIO_SetBits(GPIOA, GPIO_Pin_3);	//复位期间，CH375的Tx口应该为高，这样才能进入串口模式
}

void CH375_COM_INIT( u32 band )
{	
	// USART2
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//首先关掉串口
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

//接口硬件初始化
void CH375_PORT_INIT( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// GPIO INT -- PA0
	
	/*开启按键端口（PA）的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	///////////////////////////////////////////////////////////////////
	
	CH375_COM_INIT(9600);	//上电初始化波特率为9600
	
}

//写命令
void xWriteCH375Cmd( uint8_t cmd ) 		/* 向CH375的命令端口写入命令,周期不小于2uS,如果单片机较快则延时 */	
{
	while((USART2->SR&0x40)==0);
	USART2->DR = ((uint16_t)cmd | 0x0100 );
	delay_us(2);
}

//写数据
void xWriteCH375Data( uint8_t dat ) 	/* 向CH375的数据端口写入数据,周期不小于1uS,如果单片机较快则延时 */
{
	while((USART2->SR&0x40)==0);   
	USART2->DR = (uint16_t)dat;
	delay_us(1);
}

//读数据
uint8_t xReadCH375Data(void) 			/* 从CH375的数据端口读出数据,周期不小于1uS,如果单片机较快则延时 */
{
	uint32_t i;
	for(i=0;i<500000;i++)                      //设置500ms串口接收超时
	{
		if(USART2->SR&0x20)    //RXNE
		{
			return ((uint8_t)USART2->DR);
		}
		delay_us(1);	//这里用相对精确的1us确定时间
	}
	return ERR_USB_UNKNOWN;
}

/* 查询CH375中断(INT#低电平) */
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

/* CH375初始化代码 */
uint8_t	mInitCH375Host( void )  /* 初始化CH375 */
{
	uint8_t	res;
	
	//CH375的TX引脚拉高
	CH375_PORT_PRE_INIT();
	
	/* 上电后至少延时50ms操作 */
	delay_ms(50);        
	
	/* 接口硬件初始化 */
	CH375_PORT_INIT( );
	
	/* 测试单片机与CH375之间的通讯接口 */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
		return( ERR_USB_UNKNOWN );  /* 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作 */
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	/* 将串口波特率提高 */
	xWriteCH375Cmd( CMD_SET_BAUDRATE );  //设置USB波特率
	xWriteCH375Data( 0x03 );	//0x03 0xCC -- 115200     0x03 0xF3 -- 460800
	xWriteCH375Data( 0xCC );
	CH375_COM_INIT(115200);		//切换本地串口波特率
	delay_ms(1);	//延时1ms
	res = xReadCH375Data( );
	if ( res != CMD_RET_SUCCESS ) 
		return( ERR_USB_UNKNOWN );
	
	/* 重新测试单片机与CH375之间的通讯接口 */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
		return( ERR_USB_UNKNOWN );  /* 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作 */
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	/* 设备USB工作模式 */
	xWriteCH375Cmd( CMD_SET_USB_MODE );  
	xWriteCH375Data( 0x06 );
	delay_us( 20 );
	res = xReadCH375Data( );
	if ( res == CMD_RET_SUCCESS ) 
		return( USB_INT_SUCCESS );
	else 
		return( ERR_USB_UNKNOWN );  /* 设置模式错误 */		
}
