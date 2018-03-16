#include "HOST_SYS.H"
#include "string.h"

DEVICE_INFO_CH375  USBD;

void CH375InitSysVar( void )
{
	memset(&USBD,0,sizeof(DEVICE_INFO_CH375));
	USBD.endp0_maxpack = 8;   //初始化默认值
}
uint8_t CH375GetIntStatus( void )
{
	uint8_t res;
	xWriteCH375Cmd( CMD_GET_STATUS );
	res = xReadCH375Data();
	return ( res );
}
uint8_t CH375WaitInterrupt( void )
{
	uint32_t i;
	for ( i = 0; i < 5000000; i ++ )    /* 计数防止超时,默认的超时时间,与单片机主频有关 */
	{    
		if ( Query375Interrupt() )
			return( CH375GetIntStatus() );
	}
	return( ERR_USB_UNKNOWN );  	
}
uint8_t CH375ReadBlock( uint8_t * buf ) /* 从当前主机端点的接收缓冲区读取数据块,返回长度 */
{
	uint8_t s,l;
	xWriteCH375Cmd( CMD_RD_USB_DATA );
	s = l = xReadCH375Data( );          /* 长度 */
	if( l )
	{
		do{
			*buf = xReadCH375Data( );
			buf++;
		} while( -- l);
	}
	return ( s );
}
void CH375WriteBlock( uint8_t *buf,uint8_t len )  /* 向当前主机端点缓冲区写数据 */
{
	xWriteCH375Cmd( CMD_WR_USB_DATA7 );
	xWriteCH375Data( len );
	while( len-- ) xWriteCH375Data( *buf++ );
}
uint8_t CH375StartTrans( uint8_t endp, uint8_t tog, uint8_t pid )
{
	xWriteCH375Cmd( CMD_ISSUE_TKN_X );
	if( tog )
		xWriteCH375Data( 0xC0 );
	else
		xWriteCH375Data( 0x00 );
	xWriteCH375Data( (endp<<4)|pid  );
	return ( CH375WaitInterrupt() );
}
/*bit7 bit6:NAK重试
   1     0  对NAK无限重试
   1     1  对NAK重试200ms-2s
   0     x  直接返回NAK
bit5-bit0 (0-63) :响应超时重试次数（针对所有传输，超时5次）
*/
/* times的值： 0：NAK不重试  1：重试200-2s  >2:无限次重试NAK */
void CH375SetRetry( uint8_t times )
{
	xWriteCH375Cmd( CMD_SET_RETRY );
	xWriteCH375Data( 0x25 );
	if( times==0 )
		xWriteCH375Data( 0x05 );
	else if(times==1)
		xWriteCH375Data( 0xC5 );
	else
		xWriteCH375Data( 0x85 );
}
uint8_t CH375SetMode( uint8_t mode )
{
	uint8_t res;
	xWriteCH375Cmd( CMD_SET_USB_MODE );
	xWriteCH375Data( mode );
	delay_us( 20 );
	res = xReadCH375Data( );
	if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
	else 
	{
		return( ERR_USB_UNKNOWN ); 
	}		
}
uint8_t CH375DeviceConnect( void )
{
	uint8_t res;
	if( Query375Interrupt() ) CH375GetIntStatus();
	xWriteCH375Cmd( CMD_TEST_CONNECT );
	delay_us(5);
	res = xReadCH375Data();
	if( res == USB_INT_CONNECT || res == USB_INT_USB_READY )
		return ( TRUE );
	else 
		return ( FALSE );
}

uint8_t CH375CheckConnect( void )           //返回磁盘状态
{
	if( CH375DeviceConnect() )  
	{
		if( USBD.status == 0 )
		{
			delay_ms(250);               /* 等待设备稳定 */
			if( CH375DeviceConnect() ) 
			{
				USBD.status = 1;         /* 刚连接，未初始化 */
			}					
		}
	}
	else
	{
		if( USBD.status > 0 )
		{					
			CH375InitSysVar();                /* 设备拔出，恢复内部变量默认值 */
		}
	}
	return ( USBD.status );
}

uint8_t CH375GetFreq( void )
{
	uint8_t res;
	res = CH375SetMode( 5 );
	if(res != USB_INT_SUCCESS ) return ( USB_CMD_ERR );
	xWriteCH375Cmd( CMD_GET_DEV_RATE );   
	xWriteCH375Data( 0x07 );
	res = xReadCH375Data();
	if( res & 0x10 )
		USBD.speed = LOW_SPEED;
	else
		USBD.speed = FULL_SPEED;
	return ( res );

}
void CH375SetFreq( void )
{
	xWriteCH375Cmd( CMD_SET_USB_SPEED );  
	if( USBD.speed == LOW_SPEED )
	{
		xWriteCH375Data( 0x02 );
	}
	else
	{
		xWriteCH375Data( 0x00 );
	}
}
uint8_t CH375BusReset( void )
{
	uint8_t res;
	if ( CH375GetFreq() == USB_CMD_ERR )   /* 获取设备速度 */
		return USB_CMD_ERR;
	
	res = CH375SetMode( 7 );         /* 总线复位 */
	if(res != USB_INT_SUCCESS ) 
		return ( USB_CMD_ERR );	
	delay_ms(15);
	res = CH375SetMode( 6 );         /* 主机产生SOF包 */
	if(res != USB_INT_SUCCESS ) 
		return ( USB_CMD_ERR );	
	res = CH375WaitInterrupt();      /* 等待设备重连 */
	if( res == USB_INT_CONNECT )
	{
		USBD.addr = 0;
		res = USB_INT_SUCCESS ;
	}
	CH375SetFreq();
	return res;
}

uint8_t CH375CtlTrans( uint8_t *setp,uint8_t *datbuf,uint16_t *len )
{
	uint8_t res,l;
	uint8_t tog = 0;
	uint8_t *p = datbuf;
	uint16_t req_len,real_len = 0;
	CH375SetRetry(0x02);                         //对NAK无限次重试
	req_len = ((uint16_t)setp[7]<<8) | setp[6];
	CH375WriteBlock( setp,8 ) ;
	res = CH375StartTrans( 0, tog, DEF_USB_PID_SETUP );
	if( res == USB_INT_SUCCESS )
	{
		tog ^= 1; 
		if( setp[0]&0x80 )           //IN数据
		{
			while( req_len )
			{		
				res = CH375StartTrans( 0, tog, DEF_USB_PID_IN );
				if( res == USB_INT_SUCCESS )
				{
					tog ^= 1; 
					l = CH375ReadBlock( p );
					real_len += l;
					if( l< USBD.endp0_maxpack )  //短包
					{
						break;
					}
					p += l;
					req_len -= l;
				}
				else return( res );
			}			
		}
		else                         //OUT数据
		{
			while(req_len)
			{		
				l = (req_len>USBD.endp0_maxpack)? USBD.endp0_maxpack:req_len;
				CH375WriteBlock( p,l ); 
				res = CH375StartTrans( 0, tog, DEF_USB_PID_OUT );								
				if( res == USB_INT_SUCCESS )
				{
					tog ^= 1; 
					real_len += l;
					p += l;
					req_len -= l;
				}
				else return( res );
			}			
		}
		tog = 1;
		if( setp[0]&0x80 )		
		{
			CH375WriteBlock( p,0 ); 
			res = CH375StartTrans( 0, tog, DEF_USB_PID_OUT );
		}
		else
		{
			res = CH375StartTrans( 0, tog, DEF_USB_PID_IN );
		}
		*len = real_len;		
		return res;
	}
	return res;      /* 不应该执行这里 */
}
uint8_t CH375FindEndp( uint8_t endp, uint8_t *itfnum,uint8_t *edpnum )
{
	uint8_t itf,edp;
	for(itf=0;itf<USBD.itfmount;itf++)
	{
		for(edp=0;edp<USBD.itf[itf].edpmount;edp++)
		{
			if( endp == USBD.itf[itf].edp[edp].edpnum )   //找到匹配端点
			{
				*itfnum = itf;
				*edpnum = edp;
				return USB_INT_SUCCESS;
			}
		}
	}
	return ERRO_PRAM;
}
uint8_t CH375ClearEdpFeature( uint8_t edp )
{
	uint8_t res;
	uint8_t itfnum,edpnum;
	uint8_t ClearEdpReq[]={0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
	ClearEdpReq[4] = edp;
	res = CH375FindEndp( edp, &itfnum,&edpnum );
	if( res != USB_INT_SUCCESS ) return res;
	res = CH375CtlTrans( ClearEdpReq,NULL,NULL );	
	if( res== USB_INT_SUCCESS )
	{
		USBD.itf[itfnum].edp[edpnum].tog = 0;    //复位翻转标志位
	}
	return res;
}
uint8_t CH375InTrans( uint8_t endp,uint8_t *datbuf,uint16_t *len,uint16_t timeout )
{
	uint8_t res;
	uint8_t *p = datbuf;	
	uint8_t itfnum,edpnum;	
	CH375SetRetry(0x00);                         //对NAK不重试
	res = CH375FindEndp( endp|0x80, &itfnum,&edpnum );
	if( res != USB_INT_SUCCESS ) return res;
	while( 1 )
	{
		res = CH375StartTrans( endp,USBD.itf[itfnum].edp[edpnum].tog, DEF_USB_PID_IN );
		if( res == USB_INT_SUCCESS )
		{
			USBD.itf[itfnum].edp[edpnum].tog ^= 1;
			*len = CH375ReadBlock( p );	
			break;
		}	
		else if( res == ( DEF_USB_PID_STALL|0x20 ))   //端点返回STALL
		{
			res = CH375ClearEdpFeature( endp|0x80 );			
			break;
		}
		else if( res == ( DEF_USB_PID_NAK|0x20 ))
		{
			if( timeout == 0 ) break;
			if( timeout < 0xFFFF ) timeout--;			
		}
		else
			break;
	}
	return res;	
}
uint8_t CH375OutTrans( uint8_t endp,uint8_t *datbuf,uint16_t len,uint16_t timeout )
{
	uint8_t res;
	uint8_t *p = datbuf;	
	uint8_t itfnum,edpnum;	
	CH375SetRetry(0x00);                         //对NAK不重试
	res = CH375FindEndp( endp, &itfnum,&edpnum );
	if( res != USB_INT_SUCCESS ) return res;
	while( 1 )
	{
		CH375WriteBlock( p,len ); 
		res = CH375StartTrans( endp,USBD.itf[itfnum].edp[edpnum].tog, DEF_USB_PID_OUT );
		if( res == USB_INT_SUCCESS )
		{
			USBD.itf[itfnum].edp[edpnum].tog ^= 1;
			break;
		}	
		else if( res == ( DEF_USB_PID_STALL|0x20 ))   //端点返回STALL
		{
			res = CH375ClearEdpFeature( endp );			
			break;
		}
		else if( res == ( DEF_USB_PID_NAK|0x20 ))
		{
			if( timeout == 0 ) break;
			if( timeout < 0xFFFF ) timeout--;			
		}
		else
			break;
	}
	return res;		
}
#ifdef UseSysGetDevDesc
uint8_t CH375GetDeviceDesc( uint8_t *buf, uint16_t *len )  /* 获取设备描述符 */
{
	uint8_t res;
	xWriteCH375Cmd( CMD_GET_DESCR );
	xWriteCH375Data( 1 );
	res = CH375WaitInterrupt();
	if( res == USB_INT_SUCCESS )
	{
		*len = CH375ReadBlock( buf );
		USBD.endp0_maxpack = buf[7];
		USBD.dvid =  ((uint16_t)buf[9]<<8) + buf[8];
		USBD.dpid =  ((uint16_t)buf[11]<<8) + buf[10];
	}
	return res;
}
#else
uint8_t CH375GetDeviceDesc( uint8_t *buf, uint16_t *len )  /* 获取设备描述符 */
{
	uint8_t res;
	uint8_t GetDeviceReq[]={0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00};
	GetDeviceReq[6] = 0x08;
	res = CH375CtlTrans( GetDeviceReq,buf,len );	
	if( res== USB_INT_SUCCESS )
	{
		USBD.endp0_maxpack = buf[7];
		GetDeviceReq[6] = 0x12;
		res = CH375CtlTrans( GetDeviceReq,buf,len );
		USBD.dvid =  ((uint16_t)buf[9]<<8) + buf[8];
		USBD.dpid =  ((uint16_t)buf[11]<<8) + buf[10];
	}
	return res;
}
#endif

#ifdef UseSysSetDevAddr
uint8_t CH375SetDeviceAddr( uint8_t addr )
{
	uint8_t res;
	xWriteCH375Cmd( CMD_SET_ADDRESS );  
	xWriteCH375Data( addr );                 /* 地址, 从1到127之间的任意值, 常用2到20 */
	res = CH375WaitInterrupt();
	if ( res==USB_INT_SUCCESS ) { 
		xWriteCH375Cmd( CMD_SET_USB_ADDR );  /* 设置USB主机端的USB地址 */
		xWriteCH375Data( addr );             /* 当目标USB设备的地址成功修改后,应该同步修改主机端的USB地址 */
		USBD.addr = addr;                    /* 更新设备地址信息 */
	}
	return( res );
}
#else
uint8_t CH375SetDeviceAddr( uint8_t addr )
{
	uint8_t res;
	uint8_t SetDevAddr[]={0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00};	
	SetDevAddr[2] = addr;
	res = CH375CtlTrans( SetDevAddr,NULL,NULL );
	if( res == USB_INT_SUCCESS )
	{
		xWriteCH375Cmd( CMD_SET_USB_ADDR );  /* 设置USB主机端的USB地址 */
		xWriteCH375Data( addr );             /* 当目标USB设备的地址成功修改后,应该同步修改主机端的USB地址 */
		USBD.addr = addr;                    /* 更新设备地址信息 */		
	}
	return res;		
}
#endif

uint8_t CH375AnalyDevInfo( uint8_t *buf,uint16_t len )
{
	uint16_t i = 0;
	uint8_t iftmount = 0;
	uint8_t edpmount = 0;
	while( i<len )	
	{
		if( buf[i+1] == 2 )   //配置描述符
		{
			USBD.itfmount = buf[i+4];
			if( USBD.itfmount > MAXITFMOUNT )
				return ( ITFMOUNTOVER );
			USBD.cgfvalue = buf[i+5];
		}
		if( buf[i+1] == 4 )  //接口描述符
		{
			if( iftmount != USBD.itfmount )
			{
				edpmount = 0;       //端点数从0开始记
				iftmount++;
				USBD.itf[ iftmount-1 ].iftnum = buf[i+2];
				USBD.itf[ iftmount-1 ].edpmount = buf[i+4];
				if( USBD.itf[ iftmount-1 ].edpmount > MAXEDPMOUNT )
					return ( EDPMOUNTOVER );
				USBD.itf[ iftmount-1 ].dclass = buf[i+5];
				USBD.itf[ iftmount-1 ].dsubclass = buf[i+6];
				USBD.itf[ iftmount-1 ].dprotocol = buf[i+7];
			}
			else return ERRO_UNKNOWN;
		}	
		if( buf[i+1] == 5)   //端点描述符
		{
			if( edpmount != USBD.itf[ iftmount-1 ].edpmount )
			{
				edpmount++;
				USBD.itf[ iftmount-1 ].edp[ edpmount-1 ].edpnum = buf[i+2];
				USBD.itf[ iftmount-1 ].edp[ edpmount-1 ].attr = buf[i+3];
				USBD.itf[ iftmount-1 ].edp[ edpmount-1 ].maxpack = buf[i+4];
			}
			else return ERRO_UNKNOWN;
		}
		i += buf[i];
	}
//	//分析设备类型(此段代码不严谨，只做简单分析)
//	for( i=0;i!=USBD.itfmount;i++ )
//	{
//		if( USBD.itf[i].dclass )   
//		{
//			if( USBD.devtype )
//			{
//				USBD.devtype <<= 4;				
//			}
//			USBD.devtype |= USBD.itf[i].dclass;
//		}
//	}
	return USB_INT_SUCCESS;
}

#ifdef UseSysGetCfgDesc
uint8_t CH375GetConfDesc( uint8_t *buf, uint16_t *len )  /* 获取配置描述符 */
{
	uint8_t res;
	xWriteCH375Cmd( CMD_GET_DESCR );
	xWriteCH375Data( 2 );
	res = CH375WaitInterrupt();
	if( res == USB_INT_SUCCESS )
	{
		*len = CH375ReadBlock( buf );
		res = CH375AnalyDevInfo( buf,*len );
	}
	return res;	
}
#else
uint8_t CH375GetConfDesc( uint8_t *buf, uint16_t *len )  /* 获取配置描述符 */
{
	uint8_t res;
	uint8_t GetConfReq[]={0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};	
	GetConfReq[6] = 0x04;
	res = CH375CtlTrans( GetConfReq,buf,len );
	if( res== USB_INT_SUCCESS )
	{
		GetConfReq[6] = buf[2];
		GetConfReq[7] = buf[3];	
		res = CH375CtlTrans( GetConfReq,buf,len );
		if( res == USB_INT_SUCCESS )
			res = CH375AnalyDevInfo( buf,*len );
	}
	return res;
}
#endif
		
#ifdef UseSysSetDevConf
uint8_t CH375SetDeviceConf( uint8_t cfg )
{
	uint8_t res,i;
	uint8_t SetIdle[]={0x21,0x0a,0x00,0x00,0x00,0x00,0x00,0x00};	
	xWriteCH375Cmd( CMD_SET_CONFIG ); 
	xWriteCH375Data( cfg ); 
	res = CH375WaitInterrupt();
	if( res == USB_INT_SUCCESS )
	{
		for( i=0;i!=USBD.itfmount;i++ )
		{
			if( USBD.itf[i].dclass == USB_DEV_CLASS_HID )    //对于HID的类命令
			{
				SetIdle[4] = USBD.itf[i].iftnum;
				res = CH375CtlTrans( SetIdle,NULL,NULL );
			}
		}
		USBD.status = USBD_READY;
	}
	return res;	
}
#else
uint8_t CH375SetDeviceConf( uint8_t cfg )
{
	uint8_t res,i;
	uint8_t SetIdle[]={0x21,0x0a,0x00,0x00,0x00,0x00,0x00,0x00};
	uint8_t SetConfReq[]={0x00,0x09,0x00,0x00,0x00,0x00,0x00,0x00};	
	SetConfReq[2] = cfg;
	res = CH375CtlTrans( SetConfReq,NULL,NULL );
	if( res == USB_INT_SUCCESS )
	{
		for( i=0;i!=USBD.itfmount;i++ )
		{
			if( USBD.itf[i].dclass == USB_DEV_CLASS_HID )   //对于HID的类命令
			{
				SetIdle[4] = USBD.itf[i].iftnum;
				res = CH375CtlTrans( SetIdle,NULL,NULL );
			}
		}
		USBD.status = USBD_READY;
	}
	return res;	
}
#endif




























