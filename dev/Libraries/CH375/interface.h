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

void CH375_PORT_INIT( void );  		/* CH375ͨѶ�ӿڳ�ʼ�� */
void CH375_COM_INIT( u32 band );  		/* ����ͨ�Ų����� */
void xWriteCH375Cmd( uint8_t mCmd );	/* ��CH375д���� */
void xWriteCH375Data( uint8_t mData );	/* ��CH375д���� */
uint8_t xReadCH375Data( void );			/* ��CH375������ */
uint8_t Query375Interrupt( void );		/* ��ѯCH375�ж�(INT#����Ϊ�͵�ƽ) */
uint8_t mInitCH375Host( void );			/* ��ʼ��CH375 */

#endif

