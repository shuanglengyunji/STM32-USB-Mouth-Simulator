
#include "interface.h" 

#include "stdio.h"
/* 接口配置描述 */
/* A0    --- PF0
   D0-D1 --- PD14-PD15
   D2-D3 --- PD0-PD1
   D4-D7 --- PE7-PE10
   CS    --- PG9(NE2) BANK1的NE1~NE4
   RD    --- PD4(NOE)
   WR    --- PD5(NWE)
   INT   --- PA3  
*/

#define CH375_INT_WIRE			PAin(3)
#define CH37x_CMD_PORT  *((volatile unsigned char  *)(0x64000001)) 	       /* CH375命令端口的I/O地址 */
#define CH37x_DAT_PORT	*((volatile unsigned char  *)(0x64000000)) 	       /* CH375数据端口的I/O地址 */


void CH375_PORT_INIT( void )
{
	//开时钟
	RCC ->AHB1ENR |= (0xF<<3);                               //开启端口使能 D E F G
	RCC ->AHB1ENR |= (1<<0);                                 //使能 PortA
	RCC ->AHB3ENR |= 1<<0;                                   //使能FMC
	//配置IO与复用映射
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
	//配置FMC
	FMC_Bank1->BTCR[2] = 0x00000000;
	FMC_Bank1->BTCR[3] = 0x00000000;
	FMC_Bank1E->BWTR[2] = 0x00000000;
	FMC_Bank1->BTCR[2]|=1<<14;		//读写使用不同的时序
	FMC_Bank1->BTCR[2]|=1<<12;		//存储器写使能
	
	FMC_Bank1->BTCR[3]|=(16<<8);    //读 数据建立时间（70ns）
	FMC_Bank1->BTCR[3]|=(2<<0);    //读 地址建立时间（10ns）
	
	FMC_Bank1E->BWTR[2]|=16<<8;   	//写数据保存时间(DATAST)	
	FMC_Bank1E->BWTR[2]|=2<<0;		//写地址建立时间(ADDSET)

	FMC_Bank1->BTCR[2]|=1<<0;		//使能BANK1，区域2	
}
//写命令
void xWriteCH375Cmd( uint8_t cmd ) { 				 /* 向CH375的命令端口写入命令,周期不小于2uS,如果单片机较快则延时 */
	CH37x_CMD_PORT=cmd;
	delay_us(2);
}
//写数据
void xWriteCH375Data( uint8_t dat ) { 				 /* 向CH375的数据端口写入数据,周期不小于1uS,如果单片机较快则延时 */
	CH37x_DAT_PORT=dat;
	delay_us(1);
}
//读数据
uint8_t xReadCH375Data(void) {  				     /* 从CH375的数据端口读出数据,周期不小于1uS,如果单片机较快则延时 */
	delay_us(1);
	return( CH37x_DAT_PORT );
}
//读状态
uint8_t xReadCH375Status() {  				         /* 从CH375的数据端口读出数据,周期不小于1uS,如果单片机较快则延时 */
	delay_us(1);
	return( CH37x_CMD_PORT );
}


/* 查询CH375中断(INT#低电平) */
uint8_t	Query375Interrupt( void )
{
	/* 如果连接了CH375的中断引脚则直接查询中断引脚 */
	/* 如果未连接CH375的中断引脚则查询状态端口 */
#ifdef	CH375_INT_WIRE
	return( CH375_INT_WIRE ? FALSE : TRUE ); 
#else
	return( xReadCH375Status( ) & 0x80 ? FALSE : TRUE );  
#endif	
}

/* CH375初始化代码 */
uint8_t	mInitCH375Host( void )  /* 初始化CH375 */
{
	uint8_t	res;	
	delay_ms(50);        /* 上电后至少延时50ms操作 */
	CH375_PORT_INIT( );  /* 接口硬件初始化 */
	xWriteCH375Cmd( CMD_CHECK_EXIST );  /* 测试单片机与CH375之间的通讯接口 */
	xWriteCH375Data( 0x65 );
	res = xReadCH375Data( );
	if ( res != 0x9A ) 
	{
		printf ("check erro:%02x\n",(uint16_t)res );
		return( ERR_USB_UNKNOWN );  /* 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作 */
	}
	xWriteCH375Cmd( CMD_SET_USB_MODE );  /* 设备USB工作模式 */
	xWriteCH375Data( 0x06 );
	delay_us( 20 );
	res = xReadCH375Data( );
	if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
	else 
	{
		printf ("mode erro:%02x\n",(uint16_t)res );
		return( ERR_USB_UNKNOWN );  /* 设置模式错误 */	
	}
	
}





