#ifndef _HOST_SYS_H_
#define _HOST_SYS_H_

#include "interface.h"       //底层接口函数

#ifndef     NULL
#define     NULL  0
#endif
#ifndef		TRUE
#define		TRUE	1
#define		FALSE	0
#endif

//函数返回值
#define ERRO_PRAM      0xF9
#define ERRO_UNKNOWN   0xFB
#define EDPMOUNTOVER   0xFC
#define ITFMOUNTOVER   0xFE
#define USB_CMD_ERR    0xFF

/* 设备速度 */
#define LOW_SPEED    1
#define FULL_SPEED   0

/* 设备类型 */
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


/* 设备状态 */
#define USBD_UNCONNECT   0
#define USBD_CONNECT     1
#define USBD_READY       2

/* 用户配置区 */
#define  MAXEDPMOUNT     2       //最大端点数目
#define  MAXITFMOUNT     2       //最大接口数量
//定义相应的，选择使用集成命令形式，否则使用控制传输方式
#define UseSysGetDevDesc         //获取设备描述符（建议定义）
#define UseSysSetDevAddr         //设置地址
//#define UseSysGetCfgDesc         //获取配置描述符（不建议定义，防止某些描述符太长，导致缓冲区溢出）
#define UseSysSetDevConf         //设置配置


/* 设备信息 */
typedef struct ENDP_INFO_
{
	uint8_t edpnum;   //端点号
	uint8_t tog;      //同步标志位
	uint8_t attr;     //端点属性
	uint8_t maxpack;  //最大包大小（最大支持64）
} ENDP_INFO;
typedef struct ITF_INFO_
{
	uint8_t iftnum;    //接口号
	uint8_t dclass;    //设备类
	uint8_t dsubclass; //设备子类
	uint8_t dprotocol; //协议码
	uint8_t edpmount;  //端点数量
	ENDP_INFO edp[ MAXEDPMOUNT ]; //端点结构体
} ITF_INFO;
typedef struct DEVICE_INFO_
{
	uint8_t status;   //0:未连接  1：连接但未初始化  2：连接并且已经初始化
	uint8_t speed;    //0:全速    1：低速
	uint8_t addr;     //设备地址
	uint8_t endp0_maxpack;  //端点0最大包大小 
//	uint8_t devtype;  //设备类型
	uint8_t cgfvalue; //配置值
	uint16_t dvid;    //厂商
	uint16_t dpid;    //产品
	uint8_t itfmount; //接口数量
	ITF_INFO itf[ MAXITFMOUNT ];  //接口结构体
} DEVICE_INFO;


extern DEVICE_INFO  USBD;        //由HOST_SYS维护的内部变量（外部可以读，但不可以写）


/* 用户函数 */
extern void CH375InitSysVar(void);                                             //设备结构初始化       
extern uint8_t CH375CheckConnect( void );                                      //返回磁盘状态
extern uint8_t CH375BusReset( void );                                          //总线复位（自动检测速度并配置）
extern uint8_t CH375CtlTrans( uint8_t *setp,uint8_t *datbuf,uint16_t *len );   //基本控制传输
extern uint8_t CH375InTrans( uint8_t endp,uint8_t *datbuf,uint16_t *len,uint16_t timeout ); //主机发IN包
extern uint8_t CH375OutTrans( uint8_t endp,uint8_t *datbuf,uint16_t len,uint16_t timeout ); //主机发OUT包
extern uint8_t CH375GetDeviceDesc( uint8_t *buf, uint16_t *len );              //获取设备描述符
extern uint8_t CH375SetDeviceAddr( uint8_t addr );                             //设置设备地址
extern uint8_t CH375GetConfDesc( uint8_t *buf, uint16_t *len );                //获取配置描述符
extern uint8_t CH375SetDeviceConf( uint8_t cfg );                              //设置配置




#endif

