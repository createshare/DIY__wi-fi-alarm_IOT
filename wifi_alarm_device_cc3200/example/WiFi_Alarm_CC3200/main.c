/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
����Ŀ-Project:	      		Wi-Fi ������
�������-Author:		BlueS ��
��������-Processor: 		CC3200   
��������-Complier: 		IAR Embedded Workbench for ARM 7.20.1.7307  or higher
��������-IDE: 			IAR Embedded Workbench for ARM 7.20.1.7307  or higher
���汾-Version: 			V1.0
������-Date: 			2014-08-01
�٢ڢۢܢݢޢߢ���
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* �ļ���-FileName:			 Main.c
* �����ļ�-Dependencies:  	  
* �ļ�����-File Description:	 ( Դ����-Source File)
	��01)   CC3200  ϵͳ��Ƶ��: 80 MHz
			��Ӿ���: 32.768 kHz ������������--δʹ��
			
	��02) ϵͳ��ʱ�жϣ� 

	��03)  UARTģ��(�������WIFI�ĵ�����Ϣ)��
	           ע��ʹ�õ���"CC3200���ط�����"�Դ������⴮�� 
	            �����ʣ�115200 �� ��ѯ��polled��ģʽ��  �����ݵĵ�λ��ʼ���䣻 
                   ����żУ�飻        8λ����λ��           1λֹͣλ�� 

	��04) IICģ�飨��BMA222������ٶȴ�����ͨ�ţ����ٶ�Ϊ��400 KHz(����)

	��05)  IO���ⲿ�ж�ģ�飺

	��06)   	

	��07)  ϵͳ���жϣ���Ĭ������£���ʹ�ܵġ����ڱ��ʱ��
	             ����Ȱ����жϽ��ú��ٳ�ʼ������ ��ʼ����������´����жϡ�
	
	��08)  WEB�����������Ϣ:	��ַ��  http://emlab.sinaapp.com
	                                                                                      http://emlab.sinaapp.com/alarm.php
	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* �޸ļ�¼-Change History:   
	����      ʱ��            �汾    ��������
	Author   	   Date		      Rev          Comment
	-------------------------------------------------------------------------------
	BlueS ��  2014-08-01	      1.0	   
			   xxxx-xx-xx	      x.x	   
			   xxxx-xx-xx	      x.x				
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* ��˾-Company: 			CS-EMLAB  Co. , Ltd.
* ������Э��-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include <stdio.h>    //sprintf��C���Ա�׼���ṩ�ĺ���, ������stdio.h��
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

#include "bma222.h"    //"bma222������ٶȴ�����" -��������-ͷ�ļ�(�ⲿ��Դ)
#include "MyProject.h"	//������Ŀ: �ܹ��ܳ���-ͷ�ļ�



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
//#define SSID_NAME           "STJUSE"   //AP SSID���ƣ�������·������Wi-FI�ȵ㣩 /* AP SSID */
//#define SECURITY_TYPE       SL_SEC_TYPE_OPEN //���ܷ�ʽ��OPEN��WEP��WPA /* Security type (OPEN or WEP or WPA)*/
//#define SECURITY_KEY        ""         //����·�����ȵ������     /* Password of the secured AP */


#define SSID_NAME           "cc3200blues"   //AP SSID���ƣ�������·������Wi-FI�ȵ㣩 /* AP SSID */
#define SECURITY_TYPE       SL_SEC_TYPE_WPA //���ܷ�ʽ��OPEN��WEP��WPA /* Security type (OPEN or WEP or WPA)*/
#define SECURITY_KEY        "1234567890"         //����·�����ȵ������     /* Password of the secured AP */
#define SSID_LEN_MAX        (32)
#define BSSID_LEN_MAX       (6)   



//�����Ʒ������̶�IP (ʵ���ҵ�)
#define IP_ADDR             0x72D7ABDA   //IP��ַ(���е�WEB������)=114.215.171.218 

//�����Ʒ������̶�IP (������)
//#define IP_ADDR             0xdcb58819   //IP��ַ(���е�WEB������)= 220.181.136.25

#define PORT_NUM            80       //�˿ں�
//#define IP_ADDR             0xc0a80164 //IP��ַ(������)= 192.168.1.100
//#define PORT_NUM            5001       //�˿ں�
#define SL_STOP_TIMEOUT     30
#define BUF_SIZE            1400
#define TCP_PACKET_COUNT    10

//ʹ��UART����ʾ������Ϣ
//��ʹ������궨��ʱ�����������ص�����Ϣ(����ϸ�ڵ���)
//����������궨��ʱ����ʹ�ô���ģ��(�������Ĳ�Ʒ)
//#define UART_DEBUG_ENABLE   1  



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//BlueS/////////////////////////////////////////////////////////////////////
#define DELAY_COUNT_SOCKECT    (800000) //Լ5ms//Socket���������ݺ�Ҫ�ж��ݵ���ʱ(ʵ��,�������ܱ�֤���ݷ����ɹ�)


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

static void AppVariables_Initial();  // ��ʼ����ر���(������صı���:IP���˿ںš�SSID��)

void WiFi_Combined_Data_Packet(void);  // ��װ���ݰ�(TCP+Http)


void WiFi_Initial(void);   //��ʼ��CC3200 Wi-Fiģ��(����CC3200�ָ�����Ĭ�ϵ�״̬)

void WiFi_Start(void);   //���� CC3200 Wi-Fiģ��

void WiFi_Connect_To_AP(void) ;  //��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ�

int WiFi_Send_Data_TcpClient(void);  //���ɼ��������ݣ�ͨ��WIFIģ�鷢�ͳ�ȥ

void WiFi_Stop(void);   //ֹͣCC3200 Wi-Fiģ��

void System_DeepSleep(void);   //MCU ���뵽"���˯��"ģ�







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
static void AppVariables_Initial()   //��ʼ����ر���(������صı���:IP���˿ںš�SSID��)
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    
    //void *memset(void *s, int ch, size_t n);  
    //�������ͣ���s��ǰn���ֽ� ��typedef unsigned int size_t���� ch �滻������ s ��
    //memset����������һ���ڴ�������ĳ��������ֵ�����ǶԽϴ�Ľṹ�������������������һ����췽����
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
//����CC3200�ָ�����Ĭ�ϵ�״̬��Station����ģʽ�����Ӳ��ԣ�Auto��SmartConfig��
//ɾ�����д洢��WLAN�����ļ�������DHCP������ɨ����� ������Tx������� ��
//���õ�Դ����Ϊ��׼ģʽ����������ע��mDNS���� 
 //ע:01)֮ǰ��WLAN�����ļ���ȫ����ɾ��  
//02)���CC3200����Ĭ��״̬���������������ָ�Ĭ�ϳ���
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    //��SimpleLink�⺯����sl_Start��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //���� SimpleLink �豸��
    //��ʼ��Wi-FIͨ�Žӿڣ�����Wi-FIʹ�����ţ���������״̬���Ƿ������ɹ���
    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(__LINE__, lMode);

    // Get the device's version-information
    // ��ȡ�豸�İ汾��Ϣ
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    
    //��SimpleLink�⺯����sl_DevGet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //�ڲ����������ڻ�ȡ�豸������Ϣ
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
    //��SimpleLink�⺯����sl_WlanPolicySet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //����WLAN����ز���
    //�������Ӳ��ԣ�Auto��SmartConfig ��CC3200Ĭ���µ����Ӳ��ԣ�    
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);
  //��SimpleLink�⺯����SL_CONNECTION_POLICY��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //�������Ӳ��ԣ�Auto��SmartConfig ��CC3200Ĭ���µ����Ӳ��ԣ�   

    
    // Remove all profiles
    //��SimpleLink�⺯����sl_WlanProfileDel��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // ɾ�����д洢��WLAN�����ļ�
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // If the device is not in station-mode, try putting it in staion-mode
    //���ж�CC3200��ǰ�Ĺ���ģʽ�Ƿ�ΪStationģʽ��������ǣ����л���stationģʽ
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event before doing anything
            //
            // ���CC3200��ǰ�Ĺ���ģʽΪAPģʽʱ��ѭ���ȴ�����ѯ���״̬
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              //��SimpleLink�⺯����_SlNonOsMainLoopTask����
              //����ġ�ʹ�õ��Ŀ⺯��˵�����½�
              //��û��ʹ��OS������ϵͳ��������£��ú���������ѭ���б����á�
              //����0����ʾ����ɲ���������1����ʾ�������ڽ����С�
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        //��SimpleLink�⺯����sl_WlanSetMode��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
        //��CC3200��Wlan����ģʽ���л���stationģʽ
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        //��SimpleLink�⺯����sl_Stop��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
        // ֹͣSimpleLink���ܣ�
        //������ʹ�ܵ����š��ر�ͨ�Žӿڡ��������ֹͣ�Ļص�������
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // reset status bits
        // �������λ������״̬λ
        CLR_STATUS_BIT_ALL(g_ulStatus);

        //��SimpleLink�⺯����sl_Start��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
        //���� SimpleLink �豸��
        //��ʼ��Wi-FIͨ�Žӿڣ�����Wi-FIʹ�����ţ���������״̬���Ƿ������ɹ���
        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // Check if the device is in station again
        // �ٴ��ж�CC3200��WLAN����ģʽΪstationģʽ
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not up in STA-mode
            // �������ģʽ����stationģʽʱ������ DEVICE_NOT_IN_STATION_MODE
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    
    //��SimpleLink�⺯����sl_WlanDisconnect��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // �Ͽ�WLAN����
    //����0����ʾ���ڶϿ��У���������ֵ����ʾ�Ѿ��Ͽ�����
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait// �ȴ��Ͽ�WLAN����
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    //��SimpleLink�⺯����sl_NetCfgSet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //�ڲ�����������������������
    //����DHCP
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Disable scan
    //��SimpleLink�⺯����SL_SCAN_POLICY��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //����ɨ�����
    ucConfigOpt = SL_SCAN_POLICY(0);
    //��SimpleLink�⺯����sl_WlanPolicySet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //����WLAN����ز���
    //����WLAN��ɨ����ԣ���ɨ��
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    //����Stationģʽ�µ�Tx����: ���(0��ʾ�������)
    ucPower = 0;
    
    //��SimpleLink�⺯����sl_WlanSet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // �ڲ���������������WLAN������
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set PM policy to normal
    //��SimpleLink�⺯����sl_WlanPolicySet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //����WLAN����ز���
    //���õ�Դ����Ϊ: ��׼ģʽ
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Unregister mDNS services
    //��SimpleLink�⺯����sl_NetAppMDNSUnRegisterService����
    //����ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // ע��mDNS���� (ɾ����mDNS����������ݿ��mDNS����)
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    //��SimpleLink�⺯����sl_Stop��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // ֹͣSimpleLink���ܣ�
    //������ʹ�ܵ����š��ر�ͨ�Žӿڡ��������ֹͣ�Ļص�������
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    AppVariables_Initial();  //��ʼ����ر���(������صı���:IP���˿ںš�SSID��)

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
 //��CC3200���������ӵ�ָ����AP��WLAN������:����·������,����ȡ����Ӧ��IP��ַ��
//���IP��ַû�л�ȡ�ɹ�������ͣ�������������ѭ���
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    INT32 retVal = 0;

     // ����Ҫ����AP�����ð�ȫ����
    secParams.Key = SECURITY_KEY;   //����·�����ȵ������
    secParams.KeyLen = strlen(SECURITY_KEY);  //���� ����
    secParams.Type = SECURITY_TYPE;  //���ܷ�ʽ��OPEN or WEP or WPA

    //��SimpleLink�⺯����sl_WlanConnect��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //оƬ��Stationģʽ���ӵ� WLAN ���硣���ӳɹ�������0������ʧ�ܣ����ظ�����
    retVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(__LINE__, retVal);

    /* Wait */ //ѭ���ȴ���ֱ�����ӳɹ�������ȡ����Ӧ��IPַ  
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
           //��SimpleLink�⺯����_SlNonOsMainLoopTask����
            //����ġ�ʹ�õ��Ŀ⺯��˵�����½�
           //��û��ʹ��OS������ϵͳ��������£��ú���������ѭ���б����á�
           //����0����ʾ����ɲ���������1����ʾ�������ڽ����С�
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
//���ʹ��TI-RTOS,�ж��������ɲ���ϵͳ�Լ���ʼ��
//�����ʹ��TI-RTOS,�ж������������������г�ʼ��
#ifndef USE_TIRTOS
  //
  // Set vector table base
  // ������ʹ�õĿ���������CCS��IAR����������Ӧ���ж�������
#if defined(ccs)
        //���⺯����IntVTableBaseSet��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
        //����NVIC���ж�������VTable������ַ��
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
  //
  // Enable Processor
    //���⺯����IntMasterEnable��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //ʹ�ܴ��������ж�
  MAP_IntMasterEnable();
  
    //���⺯����IntEnable��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    //ʹ��Systickʱ�Ӵ����ж�
  MAP_IntEnable(FAULT_SYSTICK);

    //���⺯����PRCMCC3200MCUInit��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
    // MCU��ʼ��: ʱ�ӳ�ʼ�����������á�����ģʽ���õ�
  PRCMCC3200MCUInit();
}



/****************************************************************************
*������-Function:		void leds_init(void) 
*����- Description:	       LED ��IO�ڳ�ʼ��
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) 	��02)    ��03)    ��04)  
*****************************************************************************/
void leds_init(void)     //LED ��IO�ڳ�ʼ��
{
	Led_PinMuxConfig(); // ��������ӳ��(����LED  GPIO�����ź�ʱ��)

    	//�������ű��ֵ�����ر��ָ����IO�ڵĶ˿ںź����ź�
   	 GPIO_IF_LedConfigure(LED1|LED2|LED3);

	// ��Ϩ��LED1(��ɫ)��LED2(��ɫ)��LED3(��ɫ)
	GPIO_IF_LedOff(MCU_ALL_LED_IND);
}


/****************************************************************************
*������-Function:		void SW2_Int_Handler(void)
*����- Description:	       SW2(Pin15--GPIO22)�����ж��ӳ���
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void SW2_Int_Handler(void )   //
{
	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //��תLED1(��ɫ)	
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ϩ��LED1(��ɫ)	


	//System_DeepSleep();  //MCU ���뵽"���˯��"ģʽ
	
}


/****************************************************************************
*������-Function:		void SW3_BMA222_Int_Handler(void)
*����- Description:	       SW3(Pin4--GPIO13)������BMA222���ٶȴ������ж��ӳ���
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void SW3_BMA222_Int_Handler(void )   //
{
	WorkState = WORK_Alarm;  //Work������(����ʱ)


//GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)

	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //��תLED1(��ɫ)	
}




/****************************************************************************
*������-Function:		void System_Initial(void)
*����- Description:		ϵͳ��ʹ�� :��Ƭ���ڲ���Դ+�ⲿ������Դ�ĳ�ʹ��
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) �Ѳ�ʹ�õ��ڲ���Դ���ⲿ��Դ��"//"���ε�����������   
	��02) ��ʹ��˳��Ҫ��:
	���ڲ���Դ:	�ȹ������жϡ������AD�����á�IO�ڡ�Timer0
		��UART����ͨ�� ����������ģ�����λ�Ĵ��������Ź�
	���ڲ���Դ: ��LCD Һ��ģ�� ��  �� �ܢݢ�	
	���жϳ�ʹ��:ʹ��Ҫʹ�õ��ж�
	��03)    ��04)  
*****************************************************************************/
void System_Initial(void)  //  ϵͳ��ʹ�� :��Ƭ���ڲ���Դ+�ⲿ������Դ�ĳ�ʹ��
{
	BoardInit(); // �������ʼ������(�ж����á�MCU��ʼ����)

//	UDMAInit(); // ��ʼ�� uDMA ����ģ��
	
	leds_init() ;    //LED ��IO�ڳ�ʼ��	
}

/****************************************************************************
*������-Function:		void WiFi_Combined_Data_Packet(void)
*����- Description:	       ��װ���ݰ�(TCP+Http)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) ARM-GCC ��sprintf������û�а�"������"ת��Ϊ"�ַ���"�Ĺ��ܣ��������������ʵ��
	��02)    ��03)    ��04)  
*****************************************************************************/
void WiFi_Combined_Data_Packet(void)  // ��װ���ݰ�(TCP+Http)
{
	// �������飬��Ϊ��WEB ������ͨ�ţ��ʵ���TCP/IP����ǰ�����HTTP������ͷ(��POST  ��ʽ)
	//sprintf��C���Ա�׼���ṩ�ĺ���, ������stdio.h��, ֻҪ���ļ�ͷ#include <stdio.h>����.
	//ԭ��Ϊint sprintf ( char * str, const char * format, ... ); 
	//���ڰ���ʽ����ʽ(%d %f %c %s��)������д���ַ���.

	//�����Ʒ�����  // http://cloud.emlab.net/iot/alarm/   
		//01�ű�����
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02�ű�����
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//03�ű�����
	sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 



	//�����Ʒ�����//http://emlab.sinaapp.com/alarm.php
		//01�ű�����
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02�ű�����
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//02�ű�����
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 
     

	TxCounter=0;
	while(TxCounter < BUF_SIZE && TxBuffer[TxCounter]!='\0')
	{ 
		TxCounter++;
	}

}



/****************************************************************************
*������-Function:		void WiFi_Initial(void)
*����- Description:	       ��ʼ��CC3200 Wi-Fiģ��(����CC3200�ָ�����Ĭ�ϵ�״̬)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void WiFi_Initial(void)   //��ʼ��CC3200 Wi-Fiģ��(����CC3200�ָ�����Ĭ�ϵ�״̬)
{
	long retVal = -1;
	
	//����CC3200�ָ�����Ĭ�ϵ�״̬��Station����ģʽ�����Ӳ��ԣ�Auto��SmartConfig��
	//ɾ�����д洢��WLAN�����ļ�������DHCP������ɨ����� ������Tx������� ��
	//���õ�Դ����Ϊ��׼ģʽ����������ע��mDNS���� 
	//ע:01)֮ǰ��WLAN�����ļ���ȫ����ɾ��  
	//02)���CC3200����Ĭ��״̬���������������ָ�Ĭ�ϳ���
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
*������-Function:		void WiFi_Start(void)
*����- Description:	       ���� CC3200 Wi-Fiģ��
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void WiFi_Start(void)   //���� CC3200 Wi-Fiģ��
{
	long retVal = -1;
	
	//��SimpleLink�⺯����sl_Start��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//���� SimpleLink �豸��
	//��ʼ��Wi-FIͨ�Žӿڣ�����Wi-FIʹ�����ţ���������״̬���Ƿ������ɹ���
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
*������-Function:		void WiFi_Connect_To_AP(void)
*����- Description:	       ��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ��
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void WiFi_Connect_To_AP(void)   //��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ��
{
	//��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ��
	//���IP��ַû�л�ȡ�ɹ�������ͣ�������������ѭ���
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
*������-Function:		int WiFi_Send_Data_TcpClient(void); 
*����- Description:	      ���ɼ��������ݣ�ͨ��WIFIģ�鷢�ͳ�ȥ
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) 	��02)    ��03)    ��04)  
*****************************************************************************/
//��һ��TCP�ͻ����׽��֣����ӵ�ָ���ķ��������������ݡ�
//������IPַΪ���ɺ궨��IP_ADDR�����������˿ںţ��ɺ궨��PORT_NUM����
//�����ء�TCP�ͻ��������Ϸ�������
//���ؿͻ��������������TCP���ݰ�
//���ء�0������ʾ���ͳɹ��� ���ء�-1������ʾ����ʧ�ܡ�
//ע���ڽ���TCP�ͻ��˵���֮ǰ������PC���Ͻ���һ��TCP������������PORT_NUM�˿ڡ�
int WiFi_Send_Data_TcpClient(void)  //���ɼ��������ݣ�ͨ��WIFIģ�鷢�ͳ�ȥ
{
	SlSockAddrIn_t  sAddr;
	int             iAddrSize;
	int             iSockID;
	int             iStatus;
	


	//filling the TCP server socket address
	//��ʼ����Զ�˻��� TCP�������׽��ֵ���ز������ṹ�壩
	sAddr.sin_family = SL_AF_INET;  //IPv4 �׽��� (UDP, TCP, etc) 
	//��������16λ�޷��������ֽ�˳�� (���ݴ�С������) //����16λ�Ķ˿ں�
	sAddr.sin_port = sl_Htons((unsigned short)g_uiPortNum);
	//��������32λ�޷��������ֽ�˳�� (���ݴ�С������) //����32λ��IP��ַ
	sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

	iAddrSize = sizeof(SlSockAddrIn_t);


	//��SimpleLink�⺯����sl_Socket��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//����һ���׽���(����һ��ͨ�Ŷ˵�),��������Ӧ��ϵͳ��Դ��
	//��Ӧ�õ��øú������Ӷ����һ���׽��־����
	//�˺������ܣ�����һ��TCP�׽��֣�ʹ�� IPv4 Э�飩
	//���ء�����������ʾ�����ɹ���������Ϊ�׽��־����
	//���ء�����������ʾ����ʧ�ܣ���ͬ����ֵ����ʾ��ͬ�Ĵ������͡�
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
	if( iSockID < 0 ) //���ء�������ʱ����ʾ����ʧ�ܡ�
	{
		return -1; //�������ظ�ֵ��-1����
	}

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //��ʱ�ӳ���(������ʱ)  

	//��SimpleLink�⺯����sl_Connect��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//������ָ���ⲿ�˿ڣ������ַ�������ӡ�
	//���ء�0������ʾ���ӳɹ������ء����㡱����ʾ����ʧ�ܡ� 
	iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
	if( iStatus < 0 ) //���ء�������ʱ����ʾ����ʧ�ܡ�
	{
		// error
		//��SimpleLink�⺯����sl_Close��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
		//�رմ��׽��֣����ͷŸ��׽��ֵ������Դ��
		//���ء�0�����׽��ֹرճɹ���
		//���ء����������׽��ֹر�ʧ�ܡ�
		sl_Close(iSockID);
		return -1;  //�������ظ�ֵ��-1����
	}

        

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //��ʱ�ӳ���(������ʱ)  
	
	//��SimpleLink�⺯����sl_Send��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//������һ���Ѿ����ӵ�socket�������ݡ�
	//���ء��������������ͳɹ���Ϊ���͵��ֽڸ��������ء�-1��������ʧ�ܡ�
	iStatus = sl_Send(iSockID, TxBuffer, TxCounter, 0 );
	if( iStatus <= 0 )
	{
		// error
		//��SimpleLink�⺯����sl_Close��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
		//�رմ��׽��֣����ͷŸ��׽��ֵ������Դ��
		//���ء�0�����׽��ֹرճɹ���
		//���ء����������׽��ֹر�ʧ�ܡ� 
		sl_Close(iSockID);
		return -1;  //�������ظ�ֵ��-1����
	}
MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //��ʱ�ӳ���(������ʱ)  
	
#ifdef UART_DEBUG_ENABLE
	Report("Sent packet successfully\n\r"); //������ʾTCP ���ݰ����ͳɹ�
#endif
	
	//��SimpleLink�⺯����sl_Close��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//������TCP���ݰ�֮��
	//�رմ��׽��֣����ͷŸ��׽��ֵ������Դ��
	//���ء�0�����׽��ֹرճɹ���
	//���ء����������׽��ֹر�ʧ�ܡ�
	sl_Close(iSockID);

	return 0;
}



/****************************************************************************
*������-Function:		void WiFi_Stop(void)  
*����- Description:	       ֹͣCC3200 Wi-Fiģ��
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void WiFi_Stop(void)   //ֹͣCC3200 Wi-Fiģ��
{
	//��SimpleLink�⺯����sl_Stop��������ġ�ʹ�õ��Ŀ⺯��˵�����½�
	//ֹͣSimpleLink���ܣ�
	//������ʹ�ܵ����š��ر�ͨ�Žӿڡ��������ֹͣ�Ļص�������
	sl_Stop(SL_STOP_TIMEOUT);
}



/****************************************************************************
*������-Function:		void Send_Alarm_To_Cloud(void)
*����- Description:	       CC3200���ͱ�����Ϣ���Ʒ�����
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void Send_Alarm_To_Cloud(void)   //CC3200���ͱ�����Ϣ���Ʒ�����
{
//������Wi-Fi ��������/////////////////////////////////////////////////
//	WiFi_Start();  //���� CC3200 Wi-Fiģ��

//  	WiFi_Connect_To_AP();  //��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ��


	WiFi_Send_Data_TcpClient();  //���ɼ��������ݣ�ͨ��WIFIģ�鷢�ͳ�ȥ

//	WiFi_Stop();  //ֹͣCC3200 Wi-Fiģ��
/////////////////////////////////////////////////////////////////////////////////////
  
}



/****************************************************************************
*������-Function:		void System_DeepSleep(void)
*����- Description:	       MCU ���뵽"���˯��"ģʽ
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*****************************************************************************/
void System_DeepSleep(void)   //MCU ���뵽"���˯��"ģʽ
{

	//��������״̬ʱ���ص����õ�ʱ��:UART��GPIOA2��IIC(BMA222)
	//ʹϵͳ�������˯��״̬��
	MAP_PRCMDeepSleepEnter();
}






/****************************************************************************
*������-Function:		void WiFi_XXX(void)
*����- Description:	       
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   ��02)    ��03)    ��04)  
*******************************************
void WiFi_XXX(void)   //
{
	long retVal = -1;

}
**********************************/




/****************************************************************************
*������-Function:		int main(void)
*����- Description:		������
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)     MAP_UtilsDelay(8000000); //��ʱԼΪ500ms //��ʱ�ӳ���(������ʱ)
	��02)    GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)
        		GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO); //����LED2(��ɫ)
        		GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  //����LED3(��ɫ)
	��03)     ��04)   ��05)    ��06)  
*****************************************************************************/

#define PIN_SW2  22
int main()
{
	System_Initial();  //  ϵͳ��ʹ�� :��Ƭ���ڲ���Դ+�ⲿ������Դ�ĳ�ʹ��


	// ʹ��GPIOA1 ģ������˯��ʱ��
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);


Button_PinMuxConfig();  // ��������ӳ��(����SW2(Pin15--GPIO22)��SW3(Pin4--GPIO13)���������ź�ʱ��)

unsigned int uiGPIOPort;
unsigned char pucGPIOPin;
unsigned char ucPinValue;
    //Read GPIO
GPIO_IF_GetPortNPin(PIN_SW2,&uiGPIOPort,&pucGPIOPin);
ucPinValue = GPIO_IF_Get(PIN_SW2,uiGPIOPort,pucGPIOPin);
        
    //�������ʱ��SW2�����£���������SSID������
if(ucPinValue == 1)
{
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)

	


}
GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ϩ��LED1(��ɫ)	




//Button_IF_Init_sw2(SW2_Int_Handler);      //SW2������ʼ��:IO�ж�

//care: BMA222���ж��������ӵ�SW3(��BMA222��IO�ж���أ�ֱ����SW3������Button_IF_Init)
Button_IF_Init_sw3(SW3_BMA222_Int_Handler);//SW3������ʼ��:IO�ж�
   

/*****************/
//#ifdef UART_DEBUG_ENABLE
	Uart_PinMuxConfig();   // ��������ӳ��(����UART �����ź�ʱ��)
	InitTerm();  // ����UART ��ز�����������Ϊ115200��8λ���ݣ�1λֹͣλ������żУ�顣
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
    //��ʼ��CC3200�����
    //��SimpleLink�⺯��"sl_Start"�������"ʹ�õ��Ŀ⺯��˵��"�½�
    //���� SimpleLink �豸��
    //��ʼ��Wi-FIͨ�Žӿڣ�����Wi-FIʹ�����ţ���������״̬���Ƿ������ɹ���
    iRetVal = sl_Start(NULL, NULL, NULL);
    if(iRetVal < 0)
    {
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    //
    // reset all network policies
    //��SimpleLink�⺯��"sl_WlanPolicySet"�������"ʹ�õ��Ŀ⺯��˵��"�½�
    //����WLAN����ز���
    //�������Ӳ��ԣ�NULL    
    sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                    SL_CONNECTION_POLICY(0,0,0,0,0),
                    &policyVal,
                    1 ); //PolicyValLen
      //��SimpleLink�⺯��"SL_CONNECTION_POLICY"�������"ʹ�õ��Ŀ⺯��˵��"�½�
    //�������Ӳ��ԣ�Auto��SmartConfig ��CC3200Ĭ���µ����Ӳ��ԣ�    



    //  create a user file
    //��SimpleLink�⺯��"sl_FsOpen"�������"ʹ�õ��Ŀ⺯��˵��"�½�
    //��Ҫд����û��ļ�(�ⲿFlash��)
    //��SimpleLink�⺯��"FS_MODE_OPEN_CREATE"�������"ʹ�õ��Ŀ⺯��˵��"�½�
    //�½�һ���û��ļ�(�ⲿFlash��)//��СΪ65536
    iRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
    			FS_MODE_OPEN_CREATE(65536, \
                          _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                        &ulToken,
                        &lFileHandle);
    if(iRetVal < 0)
    {
        //
        // File may already be created
	//��SimpleLink�⺯��"sl_FsClose"�������"ʹ�õ��Ŀ⺯��˵��"�½�
	//�ر��û��ļ�(�ⲿFlash��)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    }
    else
    {
        //
        // close the user file
        //�ر��½����û��ļ�(�ⲿFlash��)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        if (SL_RET_CODE_OK != iRetVal)
        {
        	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -1;
        }
    }
    
    //
    //  open a user file for writing
    //���½����û��ļ�(�ⲿFlash��)����ʹ��д����
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

   	 //��SimpleLink�⺯��"sl_FsWrite"�������"ʹ�õ��Ŀ⺯��˵��"�½�
   	 //���û��ļ��е�ָ����ַ��д��ָ����С���ļ�
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
    //�ر��û��ļ�(�ⲿFlash��)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    // open a user file for reading
    //���½����û��ļ�(�ⲿFlash��)����ʹ�ܶ�����
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

	//�Ƚ����������Ƿ���ͬ
        iRetVal = memcmp(AlarmFile, 
                         TempFile, 
                         sizeof(AlarmFile));
        if (iRetVal != 0)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -2;
        }


    // close the user file
    //�ر��û��ļ�(�ⲿFlash��)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }
    
    //
    // turn ON the green LED indicating success
    //��ɫLED��:��ʾ��д�½����û��ļ������ɹ�
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
	IIC_PinMuxConfig();  // ��������ӳ��(����IIC �����ź�ʱ��)
	I2C_IF_Open(I2C_MASTER_MODE_FST);  //��ʼ�� I2C ģ�� ������ģʽ��400kbps��


	bma222_init();   //��ʼ��bma222 (bma222������ٶȴ�����) 
	
	//care: BMA222���ж��������ӵ�SW3(��BMA222��IO�ж���أ�ֱ����SW3������Button_IF_Init)
	//ʹ��INT1,����"���⶯��"���ж�
	bma222_set_RegVaule(BMA222_19_INTR_MAP,BMA222_INT1_SLOPE ); 


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

	AppVariables_Initial();  //��ʼ����ر���(������صı���:IP���˿ںš�SSID��)

	WiFi_Combined_Data_Packet();  // ��װ���ݰ�(TCP+Http)

/************/

WiFi_Initial();  //��ʼ��CC3200 Wi-Fiģ��(����CC3200�ָ�����Ĭ�ϵ�״̬)

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
WiFi_Start();  //���� CC3200 Wi-Fiģ��

WiFi_Connect_To_AP();  //��CC3200���������ӵ�ָ����AP��WLAN��,����ȡ����Ӧ��IP��ַ��


	//��ɫLED ��˸����(ÿ����˸һ��)����ʾCC3200�����ӵ�ָ��AP(·����)
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)
	MAP_UtilsDelay(20000000); //Լ2s//��ʱ�ӳ���(������ʱ)
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ϩ��LED1(��ɫ)	




/************

	System_DeepSleep();  //MCU ���뵽"���˯��"ģʽ
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)

System_Initial();  //  ϵͳ��ʹ�� :��Ƭ���ڲ���Դ+�ⲿ������Դ�ĳ�ʹ��

// ʹ��GPIOA1 ģ������˯��ʱ��
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // ��������ӳ��(����SW2(Pin15--GPIO22)��SW3(Pin4--GPIO13)���������ź�ʱ��)
//care: BMA222���ж��������ӵ�SW3(��BMA222��IO�ж���أ�ֱ����SW3������Button_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2��SW3������ʼ��:IO�ж�




	System_DeepSleep();  //MCU ���뵽"���˯��"ģʽ
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)


System_Initial();  //  ϵͳ��ʹ�� :��Ƭ���ڲ���Դ+�ⲿ������Դ�ĳ�ʹ��

// ʹ��GPIOA1 ģ������˯��ʱ��
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // ��������ӳ��(����SW2(Pin15--GPIO22)��SW3(Pin4--GPIO13)���������ź�ʱ��)
//care: BMA222���ж��������ӵ�SW3(��BMA222��IO�ж���أ�ֱ����SW3������Button_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2��SW3������ʼ��:IO�ж�



	System_DeepSleep();  //MCU ���뵽"���˯��"ģʽ
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)

	
	while (1)
	{
	
	}
	*************/
	//��ʼ��"����"״ֵ̬
//WorkState = WORK_ReadAllVaule_BMA222;  //Work����ȡBMA222���м��ٶ�����(����ʱ)

	while (1)
	{

	////////////////////////////////////////////////////////////////////////////
	//==**"����"������Work��"״̬"�µ�����**===============//
		Deal_WorkState(); // "����"������Work��"״̬"�µ����� (״̬��)


		//MAP_UtilsDelay(10000000); //��ʱ�ӳ���(������ʱ)
		//��û��ʹ��OS������ϵͳ��������£��ú���������ѭ���б����á�
		//����0����ʾ����ɲ���������1����ʾ�������ڽ����С�
		_SlNonOsMainLoopTask();
	}
}


	//MAP_UtilsDelay(100000); //��ʱ�ӳ���(������ʱ)
//bma222_data->scaled = 1;  //Ϊ1ʱ���õ�����ʵ��������
//bma222_read(bma222_hal, SENSOR_READ_ACCELERATION, bma222_data);


//#ifdef UART_DEBUG_ENABLE  
	//UART_PRINT("x:  %d    y:  %d    z:  %d\n\r",bma222_data->axis.x, bma222_data->axis.y, bma222_data->axis.z);
//#endif	



