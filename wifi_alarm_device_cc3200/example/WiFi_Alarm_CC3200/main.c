/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
◆项目-Project:	      		Wi-Fi 报警器
◆设计者-Author:		BlueS 林
◆处理器-Processor: 		CC3200   
◆编译器-Complier: 		IAR Embedded Workbench for ARM 7.20.1.7307  or higher
◆仿真器-IDE: 			IAR Embedded Workbench for ARM 7.20.1.7307  or higher
◆版本-Version: 			V1.0
◆日期-Date: 			2014-08-01
①②③④⑤⑥⑦⑧⑨⑩
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 文件名-FileName:			 Main.c
* 附属文件-Dependencies:  	  
* 文件描述-File Description:	 ( 源程序-Source File)
	■01)   CC3200  系统主频率: 80 MHz
			外接晶振: 32.768 kHz 晶体振荡器（）--未使用
			
	■02) 系统定时中断： 

	■03)  UART模块(串口输出WIFI的调试信息)：
	           注：使用的是"CC3200板载仿真器"自带的虚拟串口 
	            波特率：115200 ； 轮询（polled）模式；  从数据的低位开始传输； 
                   无奇偶校验；        8位数据位；           1位停止位； 

	■04) IIC模块（与BMA222三轴加速度传感器通信）：速度为：400 KHz(高速)

	■05)  IO口外部中断模块：

	■06)   	

	■07)  系统总中断，在默认情况下，是使能的。故在编程时，
	             最好先把总中断禁用后，再初始化程序， 初始化完后，再重新打开总中断。
	
	■08)  WEB服务器相关信息:	网址：  http://emlab.sinaapp.com
	                                                                                      http://emlab.sinaapp.com/alarm.php
	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 修改记录-Change History:   
	作者      时间            版本    内容描述
	Author   	   Date		      Rev          Comment
	-------------------------------------------------------------------------------
	BlueS 林  2014-08-01	      1.0	   
			   xxxx-xx-xx	      x.x	   
			   xxxx-xx-xx	      x.x				
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* 公司-Company: 			CS-EMLAB  Co. , Ltd.
* 软件许可协议-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include <stdio.h>    //sprintf是C语言标准库提供的函数, 包含在stdio.h中
#include <stdlib.h>
#include <string.h>

// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"

// common interface includes 
#include "udma_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif


#include "pinmux.h"
#include "gpio.h"
#include "gpio_if.h"

#include "i2c_if.h"
#include "button_if.h"

#include "bma222.h"    //"bma222三轴加速度传感器" -驱动程序-头文件(外部资源)
#include "MyProject.h"	//具体项目: 总功能程序-头文件



#ifdef NOTERM
#define UART_PRINT(x, ...)
#else
#define UART_PRINT Report
#endif

#define APPLICATION_NAME        "TCP Socket"
#define APPLICATION_VERSION     "1.0.0"
#define SUCCESS                 0

//
// Values for below macros shall be modified as per access-point(AP) properties
// SimpleLink device will connect to following AP when application is executed
//
//
//#define SSID_NAME           "STJUSE"   //AP SSID名称（即无线路由器的Wi-FI热点） /* AP SSID */
//#define SECURITY_TYPE       SL_SEC_TYPE_OPEN //加密方式：OPEN、WEP、WPA /* Security type (OPEN or WEP or WPA)*/
//#define SECURITY_KEY        ""         //无线路由器热点的密码     /* Password of the secured AP */


#define SSID_NAME           "cc3200blues"   //AP SSID名称（即无线路由器的Wi-FI热点） /* AP SSID */
#define SECURITY_TYPE       SL_SEC_TYPE_WPA //加密方式：OPEN、WEP、WPA /* Security type (OPEN or WEP or WPA)*/
#define SECURITY_KEY        "1234567890"         //无线路由器热点的密码     /* Password of the secured AP */
#define SSID_LEN_MAX        (32)
#define BSSID_LEN_MAX       (6)   



//阿里云服务器固定IP (实验室的)
#define IP_ADDR             0x72D7ABDA   //IP地址(云中的WEB服务器)=114.215.171.218 

//新浪云服务器固定IP (晶晶的)
//#define IP_ADDR             0xdcb58819   //IP地址(云中的WEB服务器)= 220.181.136.25

#define PORT_NUM            80       //端口号
//#define IP_ADDR             0xc0a80164 //IP地址(服务器)= 192.168.1.100
//#define PORT_NUM            5001       //端口号
#define SL_STOP_TIMEOUT     30
#define BUF_SIZE            1400
#define TCP_PACKET_COUNT    10

//使用UART，显示调试信息
//当使用下面宏定义时，串口输出相关调试信息(用于细节调试)
//当屏蔽下面宏定义时，不使用串口模块(用在最后的产品)
//#define UART_DEBUG_ENABLE   1  



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//BlueS/////////////////////////////////////////////////////////////////////
#define DELAY_COUNT_SOCKECT    (800000) //约5ms//Socket发送完数据后，要有短暂的延时(实测,这样才能保证数据发出成功)


#define USER_FILE_NAME          "Alarm_File.txt"



//
// GLOBAL VARIABLES -- Start
//
unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM;
unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;

char TxBuffer[1000];
unsigned int TxCounter;
//BlueS/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////







// Loop forever, user can change it as per application's requirement
#define LOOP_FOREVER(line_number) \
            {\
                while(1); \
            }

// check the error code and handle it
#define ASSERT_ON_ERROR(line_number, error_code) \
            {\
                if (error_code < 0) return error_code;\
            }

//Status bits - These are used to set/reset the corresponding bits in
// given variable
typedef enum{
    STATUS_BIT_CONNECTION =  0, // If this bit is Set SimpleLink device is
                                // connected to the AP

    STATUS_BIT_IP_AQUIRED       // If this bit is Set SimpleLink device has
                                 // acquired IP

}e_StatusBits;

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define SET_STATUS_BIT(status_variable, bit)  status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)  status_variable &= ~(1<<(bit))
#define CLR_STATUS_BIT_ALL(status_variable)   (status_variable = 0)
#define GET_STATUS_BIT(status_variable, bit)  (0 != (status_variable & \
                                                                (1<<(bit))))

#define IS_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, \
                                                       STATUS_BIT_CONNECTION)
#define IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable, \
                                                       STATUS_BIT_IP_AQUIRED)


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************


static INT32 WlanConnect();

static void BoardInit();


//////////////////////////////////////////////////
//BlueS////////////////////////////////////////

static void AppVariables_Initial();  // 初始化相关变量(网络相关的变量:IP、端口号、SSID等)

void WiFi_Combined_Data_Packet(void);  // 组装数据包(TCP+Http)


void WiFi_Initial(void);   //初始化CC3200 Wi-Fi模块(配置CC3200恢复到其默认的状态)

void WiFi_Start(void);   //启动 CC3200 Wi-Fi模块

void WiFi_Connect_To_AP(void) ;  //将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址?

int WiFi_Send_Data_TcpClient(void);  //将采集到的数据，通过WIFI模块发送出去

void WiFi_Stop(void);   //停止CC3200 Wi-Fi模块

void System_DeepSleep(void);   //MCU 进入到"深度睡眠"模?







#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//
// GLOBAL VARIABLES -- End
//



//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'sl_protocol_wlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  sl_protocol_wlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
                        " BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            sl_protocol_wlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                            "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_ACQUIRED:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_ulIpAddr = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                        "Gateway=%d.%d.%d.%d\n\r",
							SL_IPV4_BYTE(g_ulIpAddr,3),
							SL_IPV4_BYTE(g_ulIpAddr,2),
							SL_IPV4_BYTE(g_ulIpAddr,1),
							SL_IPV4_BYTE(g_ulIpAddr,0),
							SL_IPV4_BYTE(g_ulGatewayIP,3),
							SL_IPV4_BYTE(g_ulGatewayIP,2),
							SL_IPV4_BYTE(g_ulGatewayIP,1),
							SL_IPV4_BYTE(g_ulGatewayIP,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //

}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************



//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param[in]    None
//!
//! \return None
//!
//*****************************************************************************
static void AppVariables_Initial()   //初始化相关变量(网络相关的变量:IP、端口号、SSID等)
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    
    //void *memset(void *s, int ch, size_t n);  
    //函数解释：将s中前n个字节 （typedef unsigned int size_t）用 ch 替换并返回 s 。
    //memset：作用是在一段内存块中填充某个给定的值，它是对较大的结构体或数组进行清零操作的一种最快方法。
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    
    g_ulDestinationIp = IP_ADDR;
    g_uiPortNum = PORT_NUM;
    g_ulPacketCount = TCP_PACKET_COUNT;
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - TBD - Unregister mDNS services
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
//配置CC3200恢复到其默认的状态：Station工作模式、连接策略：Auto和SmartConfig、
//删除所有存储的WLAN配置文件、启用DHCP、禁用扫描策略 、设置Tx功率最大 、
//设置电源策略为标准模式、（待定）注销mDNS服务 
 //注:01)之前的WLAN配置文件将全部被删除  
//02)如果CC3200处于默认状态，则可以跳过下面恢复默认程序
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    //△SimpleLink库函数“sl_Start”，请查阅“使用到的库函数说明”章节
    //启动 SimpleLink 设备：
    //初始化Wi-FI通信接口，设置Wi-FI使能引脚，返回设置状态（是否启动成功）
    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(__LINE__, lMode);

    // Get the device's version-information
    // 获取设备的版本信息
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    
    //△SimpleLink库函数“sl_DevGet”，请查阅“使用到的库函数说明”章节
    //内部函数，用于获取设备配置信息
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(__LINE__, lRetVal);

	
#ifdef UART_DEBUG_ENABLE
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);
#endif

    // Set connection policy to Auto + SmartConfig (Device's default connection policy)
    //      
    //△SimpleLink库函数“sl_WlanPolicySet”，请查阅“使用到的库函数说明”章节
    //设置WLAN的相关策略
    //设置连接策略：Auto和SmartConfig （CC3200默认下的连接策略）    
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);
  //△SimpleLink库函数“SL_CONNECTION_POLICY”，请查阅“使用到的库函数说明”章节
    //设置连接策略：Auto和SmartConfig （CC3200默认下的连接策略）   

    
    // Remove all profiles
    //△SimpleLink库函数“sl_WlanProfileDel”，请查阅“使用到的库函数说明”章节
    // 删除所有存储的WLAN配置文件
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // If the device is not in station-mode, try putting it in staion-mode
    //先判断CC3200当前的工作模式是否为Station模式，如果不是，则切换到station模式
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event before doing anything
            //
            // 如果CC3200当前的工作模式为AP模式时，循环等待并查询相关状态
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              //△SimpleLink库函数“_SlNonOsMainLoopTask”，
              //请查阅“使用到的库函数说明”章节
              //在没有使用OS（操作系统）的情况下，该函数必须在循环中被调用。
              //返回0：表示已完成操作；返回1：表示操作仍在进行中。
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        //△SimpleLink库函数“sl_WlanSetMode”，请查阅“使用到的库函数说明”章节
        //将CC3200的Wlan工作模式，切换到station模式
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        //△SimpleLink库函数“sl_Stop”，请查阅“使用到的库函数说明”章节
        // 停止SimpleLink功能：
        //清除相关使能的引脚、关闭通信接口、调用完成停止的回调函数。
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // reset status bits
        // 清除（复位）所有状态位
        CLR_STATUS_BIT_ALL(g_ulStatus);

        //△SimpleLink库函数“sl_Start”，请查阅“使用到的库函数说明”章节
        //启动 SimpleLink 设备：
        //初始化Wi-FI通信接口，设置Wi-FI使能引脚，返回设置状态（是否启动成功）
        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // Check if the device is in station again
        // 再次判断CC3200的WLAN工作模式为station模式
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not up in STA-mode
            // 如果工作模式不是station模式时，返回 DEVICE_NOT_IN_STATION_MODE
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    
    //△SimpleLink库函数“sl_WlanDisconnect”，请查阅“使用到的库函数说明”章节
    // 断开WLAN连接
    //返回0：表示正在断开中；返回其他值：表示已经断开连接
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait// 等待断开WLAN连接
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    //△SimpleLink库函数“sl_NetCfgSet”，请查阅“使用到的库函数说明”章节
    //内部函数，用于设置网络配置
    //启用DHCP
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Disable scan
    //△SimpleLink库函数“SL_SCAN_POLICY”，请查阅“使用到的库函数说明”章节
    //禁用扫描策略
    ucConfigOpt = SL_SCAN_POLICY(0);
    //△SimpleLink库函数“sl_WlanPolicySet”，请查阅“使用到的库函数说明”章节
    //设置WLAN的相关策略
    //设置WLAN的扫描策略：不扫描
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    //设置Station模式下的Tx功率: 最大(0表示功率最大)
    ucPower = 0;
    
    //△SimpleLink库函数“sl_WlanSet”，请查阅“使用到的库函数说明”章节
    // 内部函数，用于设置WLAN的配置
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set PM policy to normal
    //△SimpleLink库函数“sl_WlanPolicySet”，请查阅“使用到的库函数说明”章节
    //设置WLAN的相关策略
    //设置电源策略为: 标准模式
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Unregister mDNS services
    //△SimpleLink库函数“sl_NetAppMDNSUnRegisterService”，
    //请查阅“使用到的库函数说明”章节
    // 注销mDNS服务 (删除的mDNS软件包和数据库的mDNS服务)
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    //△SimpleLink库函数“sl_Stop”，请查阅“使用到的库函数说明”章节
    // 停止SimpleLink功能：
    //清除相关使能的引脚、关闭通信接口、调用完成停止的回调函数。
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    AppVariables_Initial();  //初始化相关变量(网络相关的变量:IP、端口号、SSID等)

    return lRetVal; // Success
}






//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return     Status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
 //将CC3200开发板连接到指定的AP（WLAN）（例:无线路由器）,并获取到相应的IP地址。
//如果IP地址没有获取成功，程序将停留在这个函数的循环里。
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    INT32 retVal = 0;

     // 设置要连接AP的配置安全参数
    secParams.Key = SECURITY_KEY;   //无线路由器热点的密码
    secParams.KeyLen = strlen(SECURITY_KEY);  //密码 长度
    secParams.Type = SECURITY_TYPE;  //加密方式：OPEN or WEP or WPA

    //△SimpleLink库函数“sl_WlanConnect”，请查阅“使用到的库函数说明”章节
    //芯片以Station模式连接到 WLAN 网络。连接成功，返回0；连接失败，返回负数。
    retVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(__LINE__, retVal);

    /* Wait */ //循环等待，直到连接成功，并获取到相应的IP址  
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
           //△SimpleLink库函数“_SlNonOsMainLoopTask”，
            //请查阅“使用到的库函数说明”章节
           //在没有使用OS（操作系统）的情况下，该函数必须在循环中被调用。
           //返回0：表示已完成操作；返回1：表示操作仍在进行中。
           _SlNonOsMainLoopTask();
#endif
    }

    return SUCCESS;

}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
//如果使用TI-RTOS,中断向量表由操作系统自己初始化
//如果不使用TI-RTOS,中断向量表由下面程序进行初始化
#ifndef USE_TIRTOS
  //
  // Set vector table base
  // 根据所使用的开发环境（CCS或IAR）来设置相应的中断向量表
#if defined(ccs)
        //△库函数“IntVTableBaseSet”，请查阅“使用到的库函数说明”章节
        //设置NVIC的中断向量表（VTable）基地址。
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
  //
  // Enable Processor
    //△库函数“IntMasterEnable”，请查阅“使用到的库函数说明”章节
    //使能处理器总中断
  MAP_IntMasterEnable();
  
    //△库函数“IntEnable”，请查阅“使用到的库函数说明”章节
    //使能Systick时钟错误中断
  MAP_IntEnable(FAULT_SYSTICK);

    //△库函数“PRCMCC3200MCUInit”，请查阅“使用到的库函数说明”章节
    // MCU初始化: 时钟初始化、振荡器配置、调试模式配置等
  PRCMCC3200MCUInit();
}



/****************************************************************************
*函数名-Function:		void leds_init(void) 
*描述- Description:	       LED 灯IO口初始化
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 	▲02)    ▲03)    ▲04)  
*****************************************************************************/
void leds_init(void)     //LED 灯IO口初始化
{
	Led_PinMuxConfig(); // 配置引脚映射(配置LED  GPIO的引脚和时钟)

    	//根据引脚编号值，返回编号指定的IO口的端口号和引脚号
   	 GPIO_IF_LedConfigure(LED1|LED2|LED3);

	// 先熄灭LED1(红色)、LED2(黄色)、LED3(绿色)
	GPIO_IF_LedOff(MCU_ALL_LED_IND);
}


/****************************************************************************
*函数名-Function:		void SW2_Int_Handler(void)
*描述- Description:	       SW2(Pin15--GPIO22)按键中断子程序
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void SW2_Int_Handler(void )   //
{
	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //翻转LED1(红色)	
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //熄灭LED1(红色)	


	//System_DeepSleep();  //MCU 进入到"深度睡眠"模式
	
}


/****************************************************************************
*函数名-Function:		void SW3_BMA222_Int_Handler(void)
*描述- Description:	       SW3(Pin4--GPIO13)按键、BMA222加速度传感器中断子程序
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void SW3_BMA222_Int_Handler(void )   //
{
	WorkState = WORK_Alarm;  //Work▓报警(工作时)


//GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)

	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //翻转LED1(红色)	
}




/****************************************************************************
*函数名-Function:		void System_Initial(void)
*描述- Description:		系统初使化 :单片机内部资源+外部基本资源的初使化
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 把不使用的内部资源和外部资源用"//"屏蔽掉，以免误动作   
	▲02) 初使化顺序要求:
	⑴内部资源:	先关所有中断→晶振→AD口设置→IO口→Timer0
		→UART串口通信 →其他功能模块→复位寄存器→看门狗
	⑵内部资源: ①LCD 液晶模块 ②  ③ ④⑤⑥	
	⑶中断初使化:使能要使用的中断
	▲03)    ▲04)  
*****************************************************************************/
void System_Initial(void)  //  系统初使化 :单片机内部资源+外部基本资源的初使化
{
	BoardInit(); // 开发板初始化配置(中断配置、MCU初始化等)

//	UDMAInit(); // 初始化 uDMA 功能模块
	
	leds_init() ;    //LED 灯IO口初始化	
}

/****************************************************************************
*函数名-Function:		void WiFi_Combined_Data_Packet(void)
*描述- Description:	       组装数据包(TCP+Http)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) ARM-GCC 的sprintf函数，没有把"浮点数"转换为"字符串"的功能，但可用下面程序实现
	▲02)    ▲03)    ▲04)  
*****************************************************************************/
void WiFi_Combined_Data_Packet(void)  // 组装数据包(TCP+Http)
{
	// 下面数组，因为与WEB 服务器通信，故得在TCP/IP数据前，添加HTTP的数据头(用POST  方式)
	//sprintf是C语言标准库提供的函数, 包含在stdio.h中, 只要在文件头#include <stdio.h>即可.
	//原型为int sprintf ( char * str, const char * format, ... ); 
	//用于按格式化方式(%d %f %c %s等)将数据写入字符串.

	//阿里云服务器  // http://cloud.emlab.net/iot/alarm/   
		//01号报警器
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02号报警器
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//03号报警器
	sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 



	//新浪云服务器//http://emlab.sinaapp.com/alarm.php
		//01号报警器
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02号报警器
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//02号报警器
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 
     

	TxCounter=0;
	while(TxCounter < BUF_SIZE && TxBuffer[TxCounter]!='\0')
	{ 
		TxCounter++;
	}

}



/****************************************************************************
*函数名-Function:		void WiFi_Initial(void)
*描述- Description:	       初始化CC3200 Wi-Fi模块(配置CC3200恢复到其默认的状态)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void WiFi_Initial(void)   //初始化CC3200 Wi-Fi模块(配置CC3200恢复到其默认的状态)
{
	long retVal = -1;
	
	//配置CC3200恢复到其默认的状态：Station工作模式、连接策略：Auto和SmartConfig、
	//删除所有存储的WLAN配置文件、启用DHCP、禁用扫描策略 、设置Tx功率最大 、
	//设置电源策略为标准模式、（待定）注销mDNS服务 
	//注:01)之前的WLAN配置文件将全部被删除  
	//02)如果CC3200处于默认状态，则可以跳过下面恢复默认程序
	retVal = ConfigureSimpleLinkToDefaultState();

	if(retVal < 0)
	{
		if (DEVICE_NOT_IN_STATION_MODE == retVal)
		{
#ifdef UART_DEBUG_ENABLE
			UART_PRINT("Failed to configure the device in its default state \n\r");
#endif		
		}
		
		LOOP_FOREVER(__LINE__);
	}

#ifdef UART_DEBUG_ENABLE
	UART_PRINT("Device is configured in default state \n\r");
#endif
	
}



/****************************************************************************
*函数名-Function:		void WiFi_Start(void)
*描述- Description:	       启动 CC3200 Wi-Fi模块
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void WiFi_Start(void)   //启动 CC3200 Wi-Fi模块
{
	long retVal = -1;
	
	//△SimpleLink库函数“sl_Start”，请查阅“使用到的库函数说明”章节
	//启动 SimpleLink 设备：
	//初始化Wi-FI通信接口，设置Wi-FI使能引脚，返回设置状态（是否启动成功）
	retVal = sl_Start(0, 0, 0);
	
	if (retVal < 0)
	{
#ifdef UART_DEBUG_ENABLE
		UART_PRINT("Failed to start the device \n\r");
#endif
		LOOP_FOREVER(__LINE__);
	}

#ifdef UART_DEBUG_ENABLE
	UART_PRINT("Device started as STATION \n\r");
	UART_PRINT("Connecting to AP: %s ...\r\n",SSID_NAME);
#endif

}


/****************************************************************************
*函数名-Function:		void WiFi_Connect_To_AP(void)
*描述- Description:	       将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址。
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void WiFi_Connect_To_AP(void)   //将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址。
{
	//将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址。
	//如果IP地址没有获取成功，程序将停留在这个函数的循环里。
	WlanConnect();

#ifdef UART_DEBUG_ENABLE
	UART_PRINT("Connected to AP: %s \n\r",SSID_NAME);

	UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r",
	          SL_IPV4_BYTE(g_ulIpAddr,3),
	          SL_IPV4_BYTE(g_ulIpAddr,2),
	          SL_IPV4_BYTE(g_ulIpAddr,1),
	          SL_IPV4_BYTE(g_ulIpAddr,0));
#endif
}


/****************************************************************************
*函数名-Function:		int WiFi_Send_Data_TcpClient(void); 
*描述- Description:	      将采集到的数据，通过WIFI模块发送出去
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 	▲02)    ▲03)    ▲04)  
*****************************************************************************/
//打开一个TCP客户端套接字，连接到指定的服务器，发送数据。
//服务器IP址为：由宏定义IP_ADDR决定，监听端口号：由宏定义PORT_NUM决定
//“本地”TCP客户端连接上服务器后，
//本地客户端向服务器发送TCP数据包
//返回“0”：表示发送成功； 返回“-1”：表示发送失败。
//注：在进行TCP客户端调试之前，请在PC机上建好一个TCP服务器，监听PORT_NUM端口。
int WiFi_Send_Data_TcpClient(void)  //将采集到的数据，通过WIFI模块发送出去
{
	SlSockAddrIn_t  sAddr;
	int             iAddrSize;
	int             iSockID;
	int             iStatus;
	


	//filling the TCP server socket address
	//初始化“远端机” TCP服务器套接字的相关参数（结构体）
	sAddr.sin_family = SL_AF_INET;  //IPv4 套接字 (UDP, TCP, etc) 
	//重新排列16位无符号数的字节顺序 (数据大小端问题) //设置16位的端口号
	sAddr.sin_port = sl_Htons((unsigned short)g_uiPortNum);
	//重新排列32位无符号数的字节顺序 (数据大小端问题) //设置32位的IP地址
	sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

	iAddrSize = sizeof(SlSockAddrIn_t);


	//△SimpleLink库函数“sl_Socket”，请查阅“使用到的库函数说明”章节
	//创建一个套接字(创建一个通信端点),并分配相应的系统资源。
	//由应用调用该函数，从而获得一个套接字句柄。
	//此函数功能：创建一个TCP套接字（使用 IPv4 协议）
	//返回“正数”：表示创建成功，此正数为套接字句柄。
	//返回“负数”：表示创建失败，不同负数值，表示不同的错误类型。
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
	if( iSockID < 0 ) //返回“负数”时：表示创建失败。
	{
		return -1; //出错，返回负值“-1”。
	}

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //延时子程序(短暂延时)  

	//△SimpleLink库函数“sl_Connect”，请查阅“使用到的库函数说明”章节
	//创建与指定外部端口（网络地址）的连接。
	//返回“0”：表示连接成功；返回“非零”：表示连接失败。 
	iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
	if( iStatus < 0 ) //返回“负数”时：表示连接失败。
	{
		// error
		//△SimpleLink库函数“sl_Close”，请查阅“使用到的库函数说明”章节
		//关闭此套接字，并释放该套接字的相关资源。
		//返回“0”：套接字关闭成功。
		//返回“负数”：套接字关闭失败。
		sl_Close(iSockID);
		return -1;  //出错，返回负值“-1”。
	}

        

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //延时子程序(短暂延时)  
	
	//△SimpleLink库函数“sl_Send”，请查阅“使用到的库函数说明”章节
	//用于向一个已经连接的socket发送数据。
	//返回“正整数”：发送成功，为发送的字节个数；返回“-1”：发送失败。
	iStatus = sl_Send(iSockID, TxBuffer, TxCounter, 0 );
	if( iStatus <= 0 )
	{
		// error
		//△SimpleLink库函数“sl_Close”，请查阅“使用到的库函数说明”章节
		//关闭此套接字，并释放该套接字的相关资源。
		//返回“0”：套接字关闭成功。
		//返回“负数”：套接字关闭失败。 
		sl_Close(iSockID);
		return -1;  //出错，返回负值“-1”。
	}
MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //延时子程序(短暂延时)  
	
#ifdef UART_DEBUG_ENABLE
	Report("Sent packet successfully\n\r"); //串口提示TCP 数据包发送成功
#endif
	
	//△SimpleLink库函数“sl_Close”，请查阅“使用到的库函数说明”章节
	//发送完TCP数据包之后，
	//关闭此套接字，并释放该套接字的相关资源。
	//返回“0”：套接字关闭成功。
	//返回“负数”：套接字关闭失败。
	sl_Close(iSockID);

	return 0;
}



/****************************************************************************
*函数名-Function:		void WiFi_Stop(void)  
*描述- Description:	       停止CC3200 Wi-Fi模块
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void WiFi_Stop(void)   //停止CC3200 Wi-Fi模块
{
	//△SimpleLink库函数“sl_Stop”，请查阅“使用到的库函数说明”章节
	//停止SimpleLink功能：
	//清除相关使能的引脚、关闭通信接口、调用完成停止的回调函数。
	sl_Stop(SL_STOP_TIMEOUT);
}



/****************************************************************************
*函数名-Function:		void Send_Alarm_To_Cloud(void)
*描述- Description:	       CC3200发送报警信息给云服务器
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void Send_Alarm_To_Cloud(void)   //CC3200发送报警信息给云服务器
{
//完整的Wi-Fi 工作流程/////////////////////////////////////////////////
//	WiFi_Start();  //启动 CC3200 Wi-Fi模块

//  	WiFi_Connect_To_AP();  //将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址。


	WiFi_Send_Data_TcpClient();  //将采集到的数据，通过WIFI模块发送出去

//	WiFi_Stop();  //停止CC3200 Wi-Fi模块
/////////////////////////////////////////////////////////////////////////////////////
  
}



/****************************************************************************
*函数名-Function:		void System_DeepSleep(void)
*描述- Description:	       MCU 进入到"深度睡眠"模式
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_DeepSleep(void)   //MCU 进入到"深度睡眠"模式
{

	//进入休眠状态时，关掉不用的时钟:UART、GPIOA2、IIC(BMA222)
	//使系统进入深度睡眠状态。
	MAP_PRCMDeepSleepEnter();
}






/****************************************************************************
*函数名-Function:		void WiFi_XXX(void)
*描述- Description:	       
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   ▲02)    ▲03)    ▲04)  
*******************************************
void WiFi_XXX(void)   //
{
	long retVal = -1;

}
**********************************/




/****************************************************************************
*函数名-Function:		int main(void)
*描述- Description:		主函数
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)     MAP_UtilsDelay(8000000); //延时约为500ms //延时子程序(短暂延时)
	▲02)    GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)
        		GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO); //点亮LED2(黄色)
        		GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  //点亮LED3(绿色)
	▲03)     ▲04)   ▲05)    ▲06)  
*****************************************************************************/

#define PIN_SW2  22
int main()
{
	System_Initial();  //  系统初使化 :单片机内部资源+外部基本资源的初使化


	// 使能GPIOA1 模块的深度睡眠时钟
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);


Button_PinMuxConfig();  // 配置引脚映射(配置SW2(Pin15--GPIO22)、SW3(Pin4--GPIO13)按键的引脚和时钟)

unsigned int uiGPIOPort;
unsigned char pucGPIOPin;
unsigned char ucPinValue;
    //Read GPIO
GPIO_IF_GetPortNPin(PIN_SW2,&uiGPIOPort,&pucGPIOPin);
ucPinValue = GPIO_IF_Get(PIN_SW2,uiGPIOPort,pucGPIOPin);
        
    //如果开机时，SW2被按下，进入配置SSID和密码
if(ucPinValue == 1)
{
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)

	


}
GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //熄灭LED1(红色)	




//Button_IF_Init_sw2(SW2_Int_Handler);      //SW2按键初始化:IO中断

//care: BMA222的中断引脚连接到SW3(故BMA222的IO中断相关，直接在SW3中设置Button_IF_Init)
Button_IF_Init_sw3(SW3_BMA222_Int_Handler);//SW3按键初始化:IO中断
   

/*****************/
//#ifdef UART_DEBUG_ENABLE
	Uart_PinMuxConfig();   // 配置引脚映射(配置UART 的引脚和时钟)
	InitTerm();  // 配置UART 相关参数：波特率为115200，8位数据，1位停止位，无奇偶校验。
//#endif	


/******************

//==Test==//////////////////////////////////////////////////////////////////////////
//==Test==//////////////////////////////////////////////////////////////////////////
	long lFileHandle;
	unsigned long ulToken;
	int iRetVal;
	unsigned char policyVal;
	unsigned char AlarmFile[] = "0123456789";
	unsigned char TempFile[100];

    // Initializing the CC3200 networking layers
    //初始化CC3200网络层
    //△SimpleLink库函数"sl_Start"，请查阅"使用到的库函数说明"章节
    //启动 SimpleLink 设备：
    //初始化Wi-FI通信接口，设置Wi-FI使能引脚，返回设置状态（是否启动成功）
    iRetVal = sl_Start(NULL, NULL, NULL);
    if(iRetVal < 0)
    {
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    //
    // reset all network policies
    //△SimpleLink库函数"sl_WlanPolicySet"，请查阅"使用到的库函数说明"章节
    //设置WLAN的相关策略
    //设置连接策略：NULL    
    sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                    SL_CONNECTION_POLICY(0,0,0,0,0),
                    &policyVal,
                    1 ); //PolicyValLen
      //△SimpleLink库函数"SL_CONNECTION_POLICY"，请查阅"使用到的库函数说明"章节
    //设置连接策略：Auto和SmartConfig （CC3200默认下的连接策略）    



    //  create a user file
    //△SimpleLink库函数"sl_FsOpen"，请查阅"使用到的库函数说明"章节
    //打开要写入的用户文件(外部Flash中)
    //△SimpleLink库函数"FS_MODE_OPEN_CREATE"，请查阅"使用到的库函数说明"章节
    //新建一个用户文件(外部Flash中)//大小为65536
    iRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
    			FS_MODE_OPEN_CREATE(65536, \
                          _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                        &ulToken,
                        &lFileHandle);
    if(iRetVal < 0)
    {
        //
        // File may already be created
	//△SimpleLink库函数"sl_FsClose"，请查阅"使用到的库函数说明"章节
	//关闭用户文件(外部Flash中)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    }
    else
    {
        //
        // close the user file
        //关闭新建的用户文件(外部Flash中)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        if (SL_RET_CODE_OK != iRetVal)
        {
        	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -1;
        }
    }
    
    //
    //  open a user file for writing
    //打开新建的用户文件(外部Flash中)，并使能写操作
    iRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        FS_MODE_OPEN_WRITE, 
                        &ulToken,
                        &lFileHandle);
	if(iRetVal < 0)
	{
	    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
	    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
	    return -1;
	}

   	 //△SimpleLink库函数"sl_FsWrite"，请查阅"使用到的库函数说明"章节
   	 //向用户文件中的指定地址，写入指定大小的文件
        iRetVal = sl_FsWrite(lFileHandle, 0, 
                    (unsigned char *)AlarmFile, sizeof(AlarmFile));
        if (iRetVal < 0)
        {
            iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -1;
        }
    
    //
    // close the user file
    //关闭用户文件(外部Flash中)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    // open a user file for reading
    //打开新建的用户文件(外部Flash中)，并使能读操作
    iRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
    					FS_MODE_OPEN_READ,
                        &ulToken,
                        &lFileHandle);
    if(iRetVal < 0)
    {
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }
	
        iRetVal = sl_FsRead(lFileHandle, 0, 
                     TempFile, sizeof(AlarmFile));
        if ((iRetVal < 0) || (iRetVal != sizeof(AlarmFile)))
        {
            iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -1;
        }

	//比较两个数组是否相同
        iRetVal = memcmp(AlarmFile, 
                         TempFile, 
                         sizeof(AlarmFile));
        if (iRetVal != 0)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -2;
        }


    // close the user file
    //关闭用户文件(外部Flash中)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }
    
    //
    // turn ON the green LED indicating success
    //绿色LED亮:表示读写新建的用户文件操作成功
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    iRetVal = sl_Stop(SL_STOP_TIMEOUT);
    if(iRetVal < 0)
    {
        LOOP_FOREVER(__LINE__);
        //return -1; //Will not reach here
    }

	while (1)
	{
		
	}
//==Test==//////////////////////////////////////////////////////////////////////////
***********/


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
	IIC_PinMuxConfig();  // 配置引脚映射(配置IIC 的引脚和时钟)
	I2C_IF_Open(I2C_MASTER_MODE_FST);  //初始化 I2C 模块 （快速模式：400kbps）


	bma222_init();   //初始化bma222 (bma222三轴加速度传感器) 
	
	//care: BMA222的中断引脚连接到SW3(故BMA222的IO中断相关，直接在SW3中设置Button_IF_Init)
	//使能INT1,用于"任意动作"的中断
	bma222_set_RegVaule(BMA222_19_INTR_MAP,BMA222_INT1_SLOPE ); 


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

	AppVariables_Initial();  //初始化相关变量(网络相关的变量:IP、端口号、SSID等)

	WiFi_Combined_Data_Packet();  // 组装数据包(TCP+Http)

/************/

WiFi_Initial();  //初始化CC3200 Wi-Fi模块(配置CC3200恢复到其默认的状态)

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
WiFi_Start();  //启动 CC3200 Wi-Fi模块

WiFi_Connect_To_AP();  //将CC3200开发板连接到指定的AP（WLAN）,并获取到相应的IP地址。


	//红色LED 闪烁两次(每秒闪烁一次)，表示CC3200已连接到指定AP(路由器)
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)
	MAP_UtilsDelay(20000000); //约2s//延时子程序(短暂延时)
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //熄灭LED1(红色)	




/************

	System_DeepSleep();  //MCU 进入到"深度睡眠"模式
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)

System_Initial();  //  系统初使化 :单片机内部资源+外部基本资源的初使化

// 使能GPIOA1 模块的深度睡眠时钟
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // 配置引脚映射(配置SW2(Pin15--GPIO22)、SW3(Pin4--GPIO13)按键的引脚和时钟)
//care: BMA222的中断引脚连接到SW3(故BMA222的IO中断相关，直接在SW3中设置Button_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2、SW3按键初始化:IO中断




	System_DeepSleep();  //MCU 进入到"深度睡眠"模式
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)


System_Initial();  //  系统初使化 :单片机内部资源+外部基本资源的初使化

// 使能GPIOA1 模块的深度睡眠时钟
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // 配置引脚映射(配置SW2(Pin15--GPIO22)、SW3(Pin4--GPIO13)按键的引脚和时钟)
//care: BMA222的中断引脚连接到SW3(故BMA222的IO中断相关，直接在SW3中设置Button_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2、SW3按键初始化:IO中断



	System_DeepSleep();  //MCU 进入到"深度睡眠"模式
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //点亮LED1(红色)

	
	while (1)
	{
	
	}
	*************/
	//初始化"工作"状态值
//WorkState = WORK_ReadAllVaule_BMA222;  //Work▓读取BMA222所有加速度数据(工作时)

	while (1)
	{

	////////////////////////////////////////////////////////////////////////////
	//==**"处理"各工作Work▓"状态"下的事务**===============//
		Deal_WorkState(); // "处理"各工作Work▓"状态"下的事务 (状态机)


		//MAP_UtilsDelay(10000000); //延时子程序(短暂延时)
		//在没有使用OS（操作系统）的情况下，该函数必须在循环中被调用。
		//返回0：表示已完成操作；返回1：表示操作仍在进行中。
		_SlNonOsMainLoopTask();
	}
}


	//MAP_UtilsDelay(100000); //延时子程序(短暂延时)
//bma222_data->scaled = 1;  //为1时，得到最后的实际物理量
//bma222_read(bma222_hal, SENSOR_READ_ACCELERATION, bma222_data);


//#ifdef UART_DEBUG_ENABLE  
	//UART_PRINT("x:  %d    y:  %d    z:  %d\n\r",bma222_data->axis.x, bma222_data->axis.y, bma222_data->axis.z);
//#endif	



