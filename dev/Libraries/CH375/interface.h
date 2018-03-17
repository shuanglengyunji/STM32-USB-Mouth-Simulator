#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include "CH375INC.H"
#include "stm32f10x.h"
#include "hw_config.h"

#ifndef		TRUE
#define		TRUE	1
#define		FALSE	0
#endif

#define ERR_USB_UNKNOWN  0xFA

void CH375_PORT_INIT( void );  		/* CH375通讯接口初始化 */
void CH375_COM_INIT( u32 band );  		/* 串口通信波特率 */
void xWriteCH375Cmd( uint8_t mCmd );	/* 向CH375写命令 */
void xWriteCH375Data( uint8_t mData );	/* 向CH375写数据 */
uint8_t xReadCH375Data( void );			/* 从CH375读数据 */
uint8_t Query375Interrupt( void );		/* 查询CH375中断(INT#引脚为低电平) */
uint8_t mInitCH375Host( void );			/* 初始化CH375 */

#endif

