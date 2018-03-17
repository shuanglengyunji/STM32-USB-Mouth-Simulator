#ifndef _HOST_SYS_H_
#define _HOST_SYS_H_

#include "interface.h"       //�ײ�ӿں���

#ifndef     NULL
#define     NULL  0
#endif
#ifndef		TRUE
#define		TRUE	1
#define		FALSE	0
#endif

//��������ֵ
#define ERRO_PRAM      0xF9
#define ERRO_UNKNOWN   0xFB
#define EDPMOUNTOVER   0xFC
#define ITFMOUNTOVER   0xFE
#define USB_CMD_ERR    0xFF

/* �豸�ٶ� */
#define LOW_SPEED    1
#define FULL_SPEED   0

/* �豸���� */
#define USB_DEV_CLASS_RESERVED  0x00
#define USB_DEV_CLASS_AUDIO     0x01
#define USB_DEV_CLASS_COMMUNIC  0x02
#define USB_DEV_CLASS_HID       0x03
#define USB_DEV_CLASS_MONITOR   0x04
#define USB_DEV_CLASS_PHYSIC_IF 0x05
#define USB_DEV_CLASS_POWER     0x06
#define USB_DEV_CLASS_PRINTER   0x07
#define USB_DEV_CLASS_STORAGE   0x08
#define USB_DEV_CLASS_HUB       0x09
#define USB_DEV_CLASS_VEN_SPEC  0xFF


/* �豸״̬ */
#define USBD_UNCONNECT   0
#define USBD_CONNECT     1
#define USBD_READY       2

/* �û������� */
#define  MAXEDPMOUNT     2       //���˵���Ŀ
#define  MAXITFMOUNT     2       //���ӿ�����
//������Ӧ�ģ�ѡ��ʹ�ü���������ʽ������ʹ�ÿ��ƴ��䷽ʽ
#define UseSysGetDevDesc         //��ȡ�豸�����������鶨�壩
#define UseSysSetDevAddr         //���õ�ַ
//#define UseSysGetCfgDesc         //��ȡ�����������������鶨�壬��ֹĳЩ������̫�������»����������
#define UseSysSetDevConf         //��������


/* �豸��Ϣ */
typedef struct ENDP_INFO_
{
	uint8_t edpnum;   //�˵��
	uint8_t tog;      //ͬ����־λ
	uint8_t attr;     //�˵�����
	uint8_t maxpack;  //������С�����֧��64��
} ENDP_INFO;
typedef struct ITF_INFO_
{
	uint8_t iftnum;    //�ӿں�
	uint8_t dclass;    //�豸��
	uint8_t dsubclass; //�豸����
	uint8_t dprotocol; //Э����
	uint8_t edpmount;  //�˵�����
	ENDP_INFO edp[ MAXEDPMOUNT ]; //�˵�ṹ��
} ITF_INFO;
typedef struct DEVICE_INFO_
{
	uint8_t status;   //0:δ����  1�����ӵ�δ��ʼ��  2�����Ӳ����Ѿ���ʼ��
	uint8_t speed;    //0:ȫ��    1������
	uint8_t addr;     //�豸��ַ
	uint8_t endp0_maxpack;  //�˵�0������С 
//	uint8_t devtype;  //�豸����
	uint8_t cgfvalue; //����ֵ
	uint16_t dvid;    //����
	uint16_t dpid;    //��Ʒ
	uint8_t itfmount; //�ӿ�����
	ITF_INFO itf[ MAXITFMOUNT ];  //�ӿڽṹ��
} DEVICE_INFO_CH375;


extern DEVICE_INFO_CH375  USBD;        //��HOST_SYSά�����ڲ��������ⲿ���Զ�����������д��


/* �û����� */
extern void CH375InitSysVar(void);                                             //�豸�ṹ��ʼ��       
extern uint8_t CH375CheckConnect( void );                                      //���ش���״̬
extern uint8_t CH375BusReset( void );                                          //���߸�λ���Զ�����ٶȲ����ã�
extern uint8_t CH375CtlTrans( uint8_t *setp,uint8_t *datbuf,uint16_t *len );   //�������ƴ���
extern uint8_t CH375InTrans( uint8_t endp,uint8_t *datbuf,uint16_t *len,uint16_t timeout ); //������IN��
extern uint8_t CH375OutTrans( uint8_t endp,uint8_t *datbuf,uint16_t len,uint16_t timeout ); //������OUT��
extern uint8_t CH375GetDeviceDesc( uint8_t *buf, uint16_t *len );              //��ȡ�豸������
extern uint8_t CH375SetDeviceAddr( uint8_t addr );                             //�����豸��ַ
extern uint8_t CH375GetConfDesc( uint8_t *buf, uint16_t *len );                //��ȡ����������
extern uint8_t CH375SetDeviceConf( uint8_t cfg );                              //��������




#endif
