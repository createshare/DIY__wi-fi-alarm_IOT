/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
¡ôÏîÄ¿-Project:	      		Wi-Fi ±¨¾¯Æ÷
¡ôÉè¼ÆÕß-Author:		BlueS ÁÖ
¡ô´¦ÀíÆ÷-Processor: 		CC3200   
¡ô±àÒëÆ÷-Complier: 		IAR Embedded Workbench for ARM 7.20.1.7307  or higher
¡ô·ÂÕæÆ÷-IDE: 			IAR Embedded Workbench for ARM 7.20.1.7307  or higher
¡ô°æ±¾-Version: 			V1.0
¡ôÈÕÆÚ-Date: 			2014-08-01
¢Ù¢Ú¢Û¢Ü¢İ¢Ş¢ß¢à¢á¢â
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ÎÄ¼şÃû-FileName:			 Main.c
* ¸½ÊôÎÄ¼ş-Dependencies:  	  
* ÎÄ¼şÃèÊö-File Description:	 ( Ô´³ÌĞò-Source File)
	¡ö01)   CC3200  ÏµÍ³Ö÷ÆµÂÊ: 80 MHz
			Íâ½Ó¾§Õñ: 32.768 kHz ¾§ÌåÕñµ´Æ÷£¨£©--Î´Ê¹ÓÃ
			
	¡ö02) ÏµÍ³¶¨Ê±ÖĞ¶Ï£º 

	¡ö03)  UARTÄ£¿é(´®¿ÚÊä³öWIFIµÄµ÷ÊÔĞÅÏ¢)£º
	           ×¢£ºÊ¹ÓÃµÄÊÇ"CC3200°åÔØ·ÂÕæÆ÷"×Ô´øµÄĞéÄâ´®¿Ú 
	            ²¨ÌØÂÊ£º115200 £» ÂÖÑ¯£¨polled£©Ä£Ê½£»  ´ÓÊı¾İµÄµÍÎ»¿ªÊ¼´«Êä£» 
                   ÎŞÆæÅ¼Ğ£Ñé£»        8Î»Êı¾İÎ»£»           1Î»Í£Ö¹Î»£» 

	¡ö04) IICÄ£¿é£¨ÓëBMA222ÈıÖá¼ÓËÙ¶È´«¸ĞÆ÷Í¨ĞÅ£©£ºËÙ¶ÈÎª£º400 KHz(¸ßËÙ)

	¡ö05)  IO¿ÚÍâ²¿ÖĞ¶ÏÄ£¿é£º

	¡ö06)   	

	¡ö07)  ÏµÍ³×ÜÖĞ¶Ï£¬ÔÚÄ¬ÈÏÇé¿öÏÂ£¬ÊÇÊ¹ÄÜµÄ¡£¹ÊÔÚ±à³ÌÊ±£¬
	             ×îºÃÏÈ°Ñ×ÜÖĞ¶Ï½ûÓÃºó£¬ÔÙ³õÊ¼»¯³ÌĞò£¬ ³õÊ¼»¯Íêºó£¬ÔÙÖØĞÂ´ò¿ª×ÜÖĞ¶Ï¡£
	
	¡ö08)  WEB·şÎñÆ÷Ïà¹ØĞÅÏ¢:	ÍøÖ·£º  http://emlab.sinaapp.com
	                                                                                      http://emlab.sinaapp.com/alarm.php
	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ĞŞ¸Ä¼ÇÂ¼-Change History:   
	×÷Õß      Ê±¼ä            °æ±¾    ÄÚÈİÃèÊö
	Author   	   Date		      Rev          Comment
	-------------------------------------------------------------------------------
	BlueS ÁÖ  2014-08-01	      1.0	   
			   xxxx-xx-xx	      x.x	   
			   xxxx-xx-xx	      x.x				
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* ¹«Ë¾-Company: 			CS-EMLAB  Co. , Ltd.
* Èí¼şĞí¿ÉĞ­Òé-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include <stdio.h>    //sprintfÊÇCÓïÑÔ±ê×¼¿âÌá¹©µÄº¯Êı, °üº¬ÔÚstdio.hÖĞ
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

#include "bma222.h"    //"bma222ÈıÖá¼ÓËÙ¶È´«¸ĞÆ÷" -Çı¶¯³ÌĞò-Í·ÎÄ¼ş(Íâ²¿×ÊÔ´)
#include "MyProject.h"	//¾ßÌåÏîÄ¿: ×Ü¹¦ÄÜ³ÌĞò-Í·ÎÄ¼ş



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
//#define SSID_NAME           "STJUSE"   //AP SSIDÃû³Æ£¨¼´ÎŞÏßÂ·ÓÉÆ÷µÄWi-FIÈÈµã£© /* AP SSID */
//#define SECURITY_TYPE       SL_SEC_TYPE_OPEN //¼ÓÃÜ·½Ê½£ºOPEN¡¢WEP¡¢WPA /* Security type (OPEN or WEP or WPA)*/
//#define SECURITY_KEY        ""         //ÎŞÏßÂ·ÓÉÆ÷ÈÈµãµÄÃÜÂë     /* Password of the secured AP */


#define SSID_NAME           "cc3200blues"   //AP SSIDÃû³Æ£¨¼´ÎŞÏßÂ·ÓÉÆ÷µÄWi-FIÈÈµã£© /* AP SSID */
#define SECURITY_TYPE       SL_SEC_TYPE_WPA //¼ÓÃÜ·½Ê½£ºOPEN¡¢WEP¡¢WPA /* Security type (OPEN or WEP or WPA)*/
#define SECURITY_KEY        "1234567890"         //ÎŞÏßÂ·ÓÉÆ÷ÈÈµãµÄÃÜÂë     /* Password of the secured AP */
#define SSID_LEN_MAX        (32)
#define BSSID_LEN_MAX       (6)   



//°¢ÀïÔÆ·şÎñÆ÷¹Ì¶¨IP (ÊµÑéÊÒµÄ)
#define IP_ADDR             0x72D7ABDA   //IPµØÖ·(ÔÆÖĞµÄWEB·şÎñÆ÷)=114.215.171.218 

//ĞÂÀËÔÆ·şÎñÆ÷¹Ì¶¨IP (¾§¾§µÄ)
//#define IP_ADDR             0xdcb58819   //IPµØÖ·(ÔÆÖĞµÄWEB·şÎñÆ÷)= 220.181.136.25

#define PORT_NUM            80       //¶Ë¿ÚºÅ
//#define IP_ADDR             0xc0a80164 //IPµØÖ·(·şÎñÆ÷)= 192.168.1.100
//#define PORT_NUM            5001       //¶Ë¿ÚºÅ
#define SL_STOP_TIMEOUT     30
#define BUF_SIZE            1400
#define TCP_PACKET_COUNT    10

//Ê¹ÓÃUART£¬ÏÔÊ¾µ÷ÊÔĞÅÏ¢
//µ±Ê¹ÓÃÏÂÃæºê¶¨ÒåÊ±£¬´®¿ÚÊä³öÏà¹Øµ÷ÊÔĞÅÏ¢(ÓÃÓÚÏ¸½Úµ÷ÊÔ)
//µ±ÆÁ±ÎÏÂÃæºê¶¨ÒåÊ±£¬²»Ê¹ÓÃ´®¿ÚÄ£¿é(ÓÃÔÚ×îºóµÄ²úÆ·)
//#define UART_DEBUG_ENABLE   1  



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//BlueS/////////////////////////////////////////////////////////////////////
#define DELAY_COUNT_SOCKECT    (800000) //Ô¼5ms//Socket·¢ËÍÍêÊı¾İºó£¬ÒªÓĞ¶ÌÔİµÄÑÓÊ±(Êµ²â,ÕâÑù²ÅÄÜ±£Ö¤Êı¾İ·¢³ö³É¹¦)


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

static void AppVariables_Initial();  // ³õÊ¼»¯Ïà¹Ø±äÁ¿(ÍøÂçÏà¹ØµÄ±äÁ¿:IP¡¢¶Ë¿ÚºÅ¡¢SSIDµÈ)

void WiFi_Combined_Data_Packet(void);  // ×é×°Êı¾İ°ü(TCP+Http)


void WiFi_Initial(void);   //³õÊ¼»¯CC3200 Wi-FiÄ£¿é(ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬)

void WiFi_Start(void);   //Æô¶¯ CC3200 Wi-FiÄ£¿é

void WiFi_Connect_To_AP(void) ;  //½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡

int WiFi_Send_Data_TcpClient(void);  //½«²É¼¯µ½µÄÊı¾İ£¬Í¨¹ıWIFIÄ£¿é·¢ËÍ³öÈ¥

void WiFi_Stop(void);   //Í£Ö¹CC3200 Wi-FiÄ£¿é

void System_DeepSleep(void);   //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê







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
static void AppVariables_Initial()   //³õÊ¼»¯Ïà¹Ø±äÁ¿(ÍøÂçÏà¹ØµÄ±äÁ¿:IP¡¢¶Ë¿ÚºÅ¡¢SSIDµÈ)
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    
    //void *memset(void *s, int ch, size_t n);  
    //º¯Êı½âÊÍ£º½«sÖĞÇ°n¸ö×Ö½Ú £¨typedef unsigned int size_t£©ÓÃ ch Ìæ»»²¢·µ»Ø s ¡£
    //memset£º×÷ÓÃÊÇÔÚÒ»¶ÎÄÚ´æ¿éÖĞÌî³äÄ³¸ö¸ø¶¨µÄÖµ£¬ËüÊÇ¶Ô½Ï´óµÄ½á¹¹Ìå»òÊı×é½øĞĞÇåÁã²Ù×÷µÄÒ»ÖÖ×î¿ì·½·¨¡£
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
//ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬£ºStation¹¤×÷Ä£Ê½¡¢Á¬½Ó²ßÂÔ£ºAutoºÍSmartConfig¡¢
//É¾³ıËùÓĞ´æ´¢µÄWLANÅäÖÃÎÄ¼ş¡¢ÆôÓÃDHCP¡¢½ûÓÃÉ¨Ãè²ßÂÔ ¡¢ÉèÖÃTx¹¦ÂÊ×î´ó ¡¢
//ÉèÖÃµçÔ´²ßÂÔÎª±ê×¼Ä£Ê½¡¢£¨´ı¶¨£©×¢ÏúmDNS·şÎñ 
 //×¢:01)Ö®Ç°µÄWLANÅäÖÃÎÄ¼ş½«È«²¿±»É¾³ı  
//02)Èç¹ûCC3200´¦ÓÚÄ¬ÈÏ×´Ì¬£¬Ôò¿ÉÒÔÌø¹ıÏÂÃæ»Ö¸´Ä¬ÈÏ³ÌĞò
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    //¡÷SimpleLink¿âº¯Êı¡°sl_Start¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //Æô¶¯ SimpleLink Éè±¸£º
    //³õÊ¼»¯Wi-FIÍ¨ĞÅ½Ó¿Ú£¬ÉèÖÃWi-FIÊ¹ÄÜÒı½Å£¬·µ»ØÉèÖÃ×´Ì¬£¨ÊÇ·ñÆô¶¯³É¹¦£©
    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(__LINE__, lMode);

    // Get the device's version-information
    // »ñÈ¡Éè±¸µÄ°æ±¾ĞÅÏ¢
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    
    //¡÷SimpleLink¿âº¯Êı¡°sl_DevGet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÄÚ²¿º¯Êı£¬ÓÃÓÚ»ñÈ¡Éè±¸ÅäÖÃĞÅÏ¢
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
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanPolicySet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÉèÖÃWLANµÄÏà¹Ø²ßÂÔ
    //ÉèÖÃÁ¬½Ó²ßÂÔ£ºAutoºÍSmartConfig £¨CC3200Ä¬ÈÏÏÂµÄÁ¬½Ó²ßÂÔ£©    
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);
  //¡÷SimpleLink¿âº¯Êı¡°SL_CONNECTION_POLICY¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÉèÖÃÁ¬½Ó²ßÂÔ£ºAutoºÍSmartConfig £¨CC3200Ä¬ÈÏÏÂµÄÁ¬½Ó²ßÂÔ£©   

    
    // Remove all profiles
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanProfileDel¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // É¾³ıËùÓĞ´æ´¢µÄWLANÅäÖÃÎÄ¼ş
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // If the device is not in station-mode, try putting it in staion-mode
    //ÏÈÅĞ¶ÏCC3200µ±Ç°µÄ¹¤×÷Ä£Ê½ÊÇ·ñÎªStationÄ£Ê½£¬Èç¹û²»ÊÇ£¬ÔòÇĞ»»µ½stationÄ£Ê½
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event before doing anything
            //
            // Èç¹ûCC3200µ±Ç°µÄ¹¤×÷Ä£Ê½ÎªAPÄ£Ê½Ê±£¬Ñ­»·µÈ´ı²¢²éÑ¯Ïà¹Ø×´Ì¬
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              //¡÷SimpleLink¿âº¯Êı¡°_SlNonOsMainLoopTask¡±£¬
              //Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
              //ÔÚÃ»ÓĞÊ¹ÓÃOS£¨²Ù×÷ÏµÍ³£©µÄÇé¿öÏÂ£¬¸Ãº¯Êı±ØĞëÔÚÑ­»·ÖĞ±»µ÷ÓÃ¡£
              //·µ»Ø0£º±íÊ¾ÒÑÍê³É²Ù×÷£»·µ»Ø1£º±íÊ¾²Ù×÷ÈÔÔÚ½øĞĞÖĞ¡£
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        //¡÷SimpleLink¿âº¯Êı¡°sl_WlanSetMode¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
        //½«CC3200µÄWlan¹¤×÷Ä£Ê½£¬ÇĞ»»µ½stationÄ£Ê½
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        //¡÷SimpleLink¿âº¯Êı¡°sl_Stop¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
        // Í£Ö¹SimpleLink¹¦ÄÜ£º
        //Çå³ıÏà¹ØÊ¹ÄÜµÄÒı½Å¡¢¹Ø±ÕÍ¨ĞÅ½Ó¿Ú¡¢µ÷ÓÃÍê³ÉÍ£Ö¹µÄ»Øµ÷º¯Êı¡£
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // reset status bits
        // Çå³ı£¨¸´Î»£©ËùÓĞ×´Ì¬Î»
        CLR_STATUS_BIT_ALL(g_ulStatus);

        //¡÷SimpleLink¿âº¯Êı¡°sl_Start¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
        //Æô¶¯ SimpleLink Éè±¸£º
        //³õÊ¼»¯Wi-FIÍ¨ĞÅ½Ó¿Ú£¬ÉèÖÃWi-FIÊ¹ÄÜÒı½Å£¬·µ»ØÉèÖÃ×´Ì¬£¨ÊÇ·ñÆô¶¯³É¹¦£©
        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(__LINE__, lRetVal);

        // Check if the device is in station again
        // ÔÙ´ÎÅĞ¶ÏCC3200µÄWLAN¹¤×÷Ä£Ê½ÎªstationÄ£Ê½
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not up in STA-mode
            // Èç¹û¹¤×÷Ä£Ê½²»ÊÇstationÄ£Ê½Ê±£¬·µ»Ø DEVICE_NOT_IN_STATION_MODE
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanDisconnect¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // ¶Ï¿ªWLANÁ¬½Ó
    //·µ»Ø0£º±íÊ¾ÕıÔÚ¶Ï¿ªÖĞ£»·µ»ØÆäËûÖµ£º±íÊ¾ÒÑ¾­¶Ï¿ªÁ¬½Ó
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait// µÈ´ı¶Ï¿ªWLANÁ¬½Ó
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    //¡÷SimpleLink¿âº¯Êı¡°sl_NetCfgSet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÄÚ²¿º¯Êı£¬ÓÃÓÚÉèÖÃÍøÂçÅäÖÃ
    //ÆôÓÃDHCP
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Disable scan
    //¡÷SimpleLink¿âº¯Êı¡°SL_SCAN_POLICY¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //½ûÓÃÉ¨Ãè²ßÂÔ
    ucConfigOpt = SL_SCAN_POLICY(0);
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanPolicySet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÉèÖÃWLANµÄÏà¹Ø²ßÂÔ
    //ÉèÖÃWLANµÄÉ¨Ãè²ßÂÔ£º²»É¨Ãè
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    //ÉèÖÃStationÄ£Ê½ÏÂµÄTx¹¦ÂÊ: ×î´ó(0±íÊ¾¹¦ÂÊ×î´ó)
    ucPower = 0;
    
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanSet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // ÄÚ²¿º¯Êı£¬ÓÃÓÚÉèÖÃWLANµÄÅäÖÃ
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Set PM policy to normal
    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanPolicySet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //ÉèÖÃWLANµÄÏà¹Ø²ßÂÔ
    //ÉèÖÃµçÔ´²ßÂÔÎª: ±ê×¼Ä£Ê½
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    // Unregister mDNS services
    //¡÷SimpleLink¿âº¯Êı¡°sl_NetAppMDNSUnRegisterService¡±£¬
    //Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // ×¢ÏúmDNS·şÎñ (É¾³ıµÄmDNSÈí¼ş°üºÍÊı¾İ¿âµÄmDNS·şÎñ)
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    //¡÷SimpleLink¿âº¯Êı¡°sl_Stop¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // Í£Ö¹SimpleLink¹¦ÄÜ£º
    //Çå³ıÏà¹ØÊ¹ÄÜµÄÒı½Å¡¢¹Ø±ÕÍ¨ĞÅ½Ó¿Ú¡¢µ÷ÓÃÍê³ÉÍ£Ö¹µÄ»Øµ÷º¯Êı¡£
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(__LINE__, lRetVal);

    AppVariables_Initial();  //³õÊ¼»¯Ïà¹Ø±äÁ¿(ÍøÂçÏà¹ØµÄ±äÁ¿:IP¡¢¶Ë¿ÚºÅ¡¢SSIDµÈ)

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
 //½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©£¨Àı:ÎŞÏßÂ·ÓÉÆ÷£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£
//Èç¹ûIPµØÖ·Ã»ÓĞ»ñÈ¡³É¹¦£¬³ÌĞò½«Í£ÁôÔÚÕâ¸öº¯ÊıµÄÑ­»·Àï¡£
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    INT32 retVal = 0;

     // ÉèÖÃÒªÁ¬½ÓAPµÄÅäÖÃ°²È«²ÎÊı
    secParams.Key = SECURITY_KEY;   //ÎŞÏßÂ·ÓÉÆ÷ÈÈµãµÄÃÜÂë
    secParams.KeyLen = strlen(SECURITY_KEY);  //ÃÜÂë ³¤¶È
    secParams.Type = SECURITY_TYPE;  //¼ÓÃÜ·½Ê½£ºOPEN or WEP or WPA

    //¡÷SimpleLink¿âº¯Êı¡°sl_WlanConnect¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //Ğ¾Æ¬ÒÔStationÄ£Ê½Á¬½Óµ½ WLAN ÍøÂç¡£Á¬½Ó³É¹¦£¬·µ»Ø0£»Á¬½ÓÊ§°Ü£¬·µ»Ø¸ºÊı¡£
    retVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(__LINE__, retVal);

    /* Wait */ //Ñ­»·µÈ´ı£¬Ö±µ½Á¬½Ó³É¹¦£¬²¢»ñÈ¡µ½ÏàÓ¦µÄIPÖ·  
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
           //¡÷SimpleLink¿âº¯Êı¡°_SlNonOsMainLoopTask¡±£¬
            //Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
           //ÔÚÃ»ÓĞÊ¹ÓÃOS£¨²Ù×÷ÏµÍ³£©µÄÇé¿öÏÂ£¬¸Ãº¯Êı±ØĞëÔÚÑ­»·ÖĞ±»µ÷ÓÃ¡£
           //·µ»Ø0£º±íÊ¾ÒÑÍê³É²Ù×÷£»·µ»Ø1£º±íÊ¾²Ù×÷ÈÔÔÚ½øĞĞÖĞ¡£
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
//Èç¹ûÊ¹ÓÃTI-RTOS,ÖĞ¶ÏÏòÁ¿±íÓÉ²Ù×÷ÏµÍ³×Ô¼º³õÊ¼»¯
//Èç¹û²»Ê¹ÓÃTI-RTOS,ÖĞ¶ÏÏòÁ¿±íÓÉÏÂÃæ³ÌĞò½øĞĞ³õÊ¼»¯
#ifndef USE_TIRTOS
  //
  // Set vector table base
  // ¸ù¾İËùÊ¹ÓÃµÄ¿ª·¢»·¾³£¨CCS»òIAR£©À´ÉèÖÃÏàÓ¦µÄÖĞ¶ÏÏòÁ¿±í
#if defined(ccs)
        //¡÷¿âº¯Êı¡°IntVTableBaseSet¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
        //ÉèÖÃNVICµÄÖĞ¶ÏÏòÁ¿±í£¨VTable£©»ùµØÖ·¡£
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
  //
  // Enable Processor
    //¡÷¿âº¯Êı¡°IntMasterEnable¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //Ê¹ÄÜ´¦ÀíÆ÷×ÜÖĞ¶Ï
  MAP_IntMasterEnable();
  
    //¡÷¿âº¯Êı¡°IntEnable¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    //Ê¹ÄÜSystickÊ±ÖÓ´íÎóÖĞ¶Ï
  MAP_IntEnable(FAULT_SYSTICK);

    //¡÷¿âº¯Êı¡°PRCMCC3200MCUInit¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
    // MCU³õÊ¼»¯: Ê±ÖÓ³õÊ¼»¯¡¢Õñµ´Æ÷ÅäÖÃ¡¢µ÷ÊÔÄ£Ê½ÅäÖÃµÈ
  PRCMCC3200MCUInit();
}



/****************************************************************************
*º¯ÊıÃû-Function:		void leds_init(void) 
*ÃèÊö- Description:	       LED µÆIO¿Ú³õÊ¼»¯
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01) 	¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void leds_init(void)     //LED µÆIO¿Ú³õÊ¼»¯
{
	Led_PinMuxConfig(); // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃLED  GPIOµÄÒı½ÅºÍÊ±ÖÓ)

    	//¸ù¾İÒı½Å±àºÅÖµ£¬·µ»Ø±àºÅÖ¸¶¨µÄIO¿ÚµÄ¶Ë¿ÚºÅºÍÒı½ÅºÅ
   	 GPIO_IF_LedConfigure(LED1|LED2|LED3);

	// ÏÈÏ¨ÃğLED1(ºìÉ«)¡¢LED2(»ÆÉ«)¡¢LED3(ÂÌÉ«)
	GPIO_IF_LedOff(MCU_ALL_LED_IND);
}


/****************************************************************************
*º¯ÊıÃû-Function:		void SW2_Int_Handler(void)
*ÃèÊö- Description:	       SW2(Pin15--GPIO22)°´¼üÖĞ¶Ï×Ó³ÌĞò
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void SW2_Int_Handler(void )   //
{
	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //·­×ªLED1(ºìÉ«)	
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ï¨ÃğLED1(ºìÉ«)	


	//System_DeepSleep();  //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
	
}


/****************************************************************************
*º¯ÊıÃû-Function:		void SW3_BMA222_Int_Handler(void)
*ÃèÊö- Description:	       SW3(Pin4--GPIO13)°´¼ü¡¢BMA222¼ÓËÙ¶È´«¸ĞÆ÷ÖĞ¶Ï×Ó³ÌĞò
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void SW3_BMA222_Int_Handler(void )   //
{
	WorkState = WORK_Alarm;  //Work¨ˆ±¨¾¯(¹¤×÷Ê±)


//GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)

	//GPIO_IF_LedToggle(MCU_RED_LED_GPIO);    //·­×ªLED1(ºìÉ«)	
}




/****************************************************************************
*º¯ÊıÃû-Function:		void System_Initial(void)
*ÃèÊö- Description:		ÏµÍ³³õÊ¹»¯ :µ¥Æ¬»úÄÚ²¿×ÊÔ´+Íâ²¿»ù±¾×ÊÔ´µÄ³õÊ¹»¯
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01) °Ñ²»Ê¹ÓÃµÄÄÚ²¿×ÊÔ´ºÍÍâ²¿×ÊÔ´ÓÃ"//"ÆÁ±Îµô£¬ÒÔÃâÎó¶¯×÷   
	¡ø02) ³õÊ¹»¯Ë³ĞòÒªÇó:
	¢ÅÄÚ²¿×ÊÔ´:	ÏÈ¹ØËùÓĞÖĞ¶Ï¡ú¾§Õñ¡úAD¿ÚÉèÖÃ¡úIO¿Ú¡úTimer0
		¡úUART´®¿ÚÍ¨ĞÅ ¡úÆäËû¹¦ÄÜÄ£¿é¡ú¸´Î»¼Ä´æÆ÷¡ú¿´ÃÅ¹·
	¢ÆÄÚ²¿×ÊÔ´: ¢ÙLCD Òº¾§Ä£¿é ¢Ú  ¢Û ¢Ü¢İ¢Ş	
	¢ÇÖĞ¶Ï³õÊ¹»¯:Ê¹ÄÜÒªÊ¹ÓÃµÄÖĞ¶Ï
	¡ø03)    ¡ø04)  
*****************************************************************************/
void System_Initial(void)  //  ÏµÍ³³õÊ¹»¯ :µ¥Æ¬»úÄÚ²¿×ÊÔ´+Íâ²¿»ù±¾×ÊÔ´µÄ³õÊ¹»¯
{
	BoardInit(); // ¿ª·¢°å³õÊ¼»¯ÅäÖÃ(ÖĞ¶ÏÅäÖÃ¡¢MCU³õÊ¼»¯µÈ)

//	UDMAInit(); // ³õÊ¼»¯ uDMA ¹¦ÄÜÄ£¿é
	
	leds_init() ;    //LED µÆIO¿Ú³õÊ¼»¯	
}

/****************************************************************************
*º¯ÊıÃû-Function:		void WiFi_Combined_Data_Packet(void)
*ÃèÊö- Description:	       ×é×°Êı¾İ°ü(TCP+Http)
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01) ARM-GCC µÄsprintfº¯Êı£¬Ã»ÓĞ°Ñ"¸¡µãÊı"×ª»»Îª"×Ö·û´®"µÄ¹¦ÄÜ£¬µ«¿ÉÓÃÏÂÃæ³ÌĞòÊµÏÖ
	¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void WiFi_Combined_Data_Packet(void)  // ×é×°Êı¾İ°ü(TCP+Http)
{
	// ÏÂÃæÊı×é£¬ÒòÎªÓëWEB ·şÎñÆ÷Í¨ĞÅ£¬¹ÊµÃÔÚTCP/IPÊı¾İÇ°£¬Ìí¼ÓHTTPµÄÊı¾İÍ·(ÓÃPOST  ·½Ê½)
	//sprintfÊÇCÓïÑÔ±ê×¼¿âÌá¹©µÄº¯Êı, °üº¬ÔÚstdio.hÖĞ, Ö»ÒªÔÚÎÄ¼şÍ·#include <stdio.h>¼´¿É.
	//Ô­ĞÍÎªint sprintf ( char * str, const char * format, ... ); 
	//ÓÃÓÚ°´¸ñÊ½»¯·½Ê½(%d %f %c %sµÈ)½«Êı¾İĞ´Èë×Ö·û´®.

	//°¢ÀïÔÆ·şÎñÆ÷  // http://cloud.emlab.net/iot/alarm/   
		//01ºÅ±¨¾¯Æ÷
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02ºÅ±¨¾¯Æ÷
	//sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//03ºÅ±¨¾¯Æ÷
	sprintf(TxBuffer,  "POST /iot/alarm/index.php HTTP/1.1\r\nHost: cloud.emlab.net\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 



	//ĞÂÀËÔÆ·şÎñÆ÷//http://emlab.sinaapp.com/alarm.php
		//01ºÅ±¨¾¯Æ÷
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState01=02\r\n"); 
		//02ºÅ±¨¾¯Æ÷
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState02=02\r\n"); 
		//02ºÅ±¨¾¯Æ÷
	//sprintf(TxBuffer,  "POST /alarm.php HTTP/1.1\r\nHost: emlab.sinaapp.com\r\nConnection: keep-alive\r\nContent-Length: 15\r\nAccept: */*\r\nCache-Control: no-cache\r\nX-Requested-With: : XMLHttpRequest\r\nAccept-Language: en-US,en;q=0.8\r\nUser-Agent: Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/27.0.1453.110 Safari/537.36 CoolNovo/2.0.9.20\r\nContent-Type: application/x-www-form-urlencoded, application/x-www-form-urlencoded\r\n\r\nalarmState03=02\r\n"); 
     

	TxCounter=0;
	while(TxCounter < BUF_SIZE && TxBuffer[TxCounter]!='\0')
	{ 
		TxCounter++;
	}

}



/****************************************************************************
*º¯ÊıÃû-Function:		void WiFi_Initial(void)
*ÃèÊö- Description:	       ³õÊ¼»¯CC3200 Wi-FiÄ£¿é(ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬)
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void WiFi_Initial(void)   //³õÊ¼»¯CC3200 Wi-FiÄ£¿é(ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬)
{
	long retVal = -1;
	
	//ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬£ºStation¹¤×÷Ä£Ê½¡¢Á¬½Ó²ßÂÔ£ºAutoºÍSmartConfig¡¢
	//É¾³ıËùÓĞ´æ´¢µÄWLANÅäÖÃÎÄ¼ş¡¢ÆôÓÃDHCP¡¢½ûÓÃÉ¨Ãè²ßÂÔ ¡¢ÉèÖÃTx¹¦ÂÊ×î´ó ¡¢
	//ÉèÖÃµçÔ´²ßÂÔÎª±ê×¼Ä£Ê½¡¢£¨´ı¶¨£©×¢ÏúmDNS·şÎñ 
	//×¢:01)Ö®Ç°µÄWLANÅäÖÃÎÄ¼ş½«È«²¿±»É¾³ı  
	//02)Èç¹ûCC3200´¦ÓÚÄ¬ÈÏ×´Ì¬£¬Ôò¿ÉÒÔÌø¹ıÏÂÃæ»Ö¸´Ä¬ÈÏ³ÌĞò
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
*º¯ÊıÃû-Function:		void WiFi_Start(void)
*ÃèÊö- Description:	       Æô¶¯ CC3200 Wi-FiÄ£¿é
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void WiFi_Start(void)   //Æô¶¯ CC3200 Wi-FiÄ£¿é
{
	long retVal = -1;
	
	//¡÷SimpleLink¿âº¯Êı¡°sl_Start¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//Æô¶¯ SimpleLink Éè±¸£º
	//³õÊ¼»¯Wi-FIÍ¨ĞÅ½Ó¿Ú£¬ÉèÖÃWi-FIÊ¹ÄÜÒı½Å£¬·µ»ØÉèÖÃ×´Ì¬£¨ÊÇ·ñÆô¶¯³É¹¦£©
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
*º¯ÊıÃû-Function:		void WiFi_Connect_To_AP(void)
*ÃèÊö- Description:	       ½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void WiFi_Connect_To_AP(void)   //½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£
{
	//½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£
	//Èç¹ûIPµØÖ·Ã»ÓĞ»ñÈ¡³É¹¦£¬³ÌĞò½«Í£ÁôÔÚÕâ¸öº¯ÊıµÄÑ­»·Àï¡£
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
*º¯ÊıÃû-Function:		int WiFi_Send_Data_TcpClient(void); 
*ÃèÊö- Description:	      ½«²É¼¯µ½µÄÊı¾İ£¬Í¨¹ıWIFIÄ£¿é·¢ËÍ³öÈ¥
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01) 	¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
//´ò¿ªÒ»¸öTCP¿Í»§¶ËÌ×½Ó×Ö£¬Á¬½Óµ½Ö¸¶¨µÄ·şÎñÆ÷£¬·¢ËÍÊı¾İ¡£
//·şÎñÆ÷IPÖ·Îª£ºÓÉºê¶¨ÒåIP_ADDR¾ö¶¨£¬¼àÌı¶Ë¿ÚºÅ£ºÓÉºê¶¨ÒåPORT_NUM¾ö¶¨
//¡°±¾µØ¡±TCP¿Í»§¶ËÁ¬½ÓÉÏ·şÎñÆ÷ºó£¬
//±¾µØ¿Í»§¶ËÏò·şÎñÆ÷·¢ËÍTCPÊı¾İ°ü
//·µ»Ø¡°0¡±£º±íÊ¾·¢ËÍ³É¹¦£» ·µ»Ø¡°-1¡±£º±íÊ¾·¢ËÍÊ§°Ü¡£
//×¢£ºÔÚ½øĞĞTCP¿Í»§¶Ëµ÷ÊÔÖ®Ç°£¬ÇëÔÚPC»úÉÏ½¨ºÃÒ»¸öTCP·şÎñÆ÷£¬¼àÌıPORT_NUM¶Ë¿Ú¡£
int WiFi_Send_Data_TcpClient(void)  //½«²É¼¯µ½µÄÊı¾İ£¬Í¨¹ıWIFIÄ£¿é·¢ËÍ³öÈ¥
{
	SlSockAddrIn_t  sAddr;
	int             iAddrSize;
	int             iSockID;
	int             iStatus;
	


	//filling the TCP server socket address
	//³õÊ¼»¯¡°Ô¶¶Ë»ú¡± TCP·şÎñÆ÷Ì×½Ó×ÖµÄÏà¹Ø²ÎÊı£¨½á¹¹Ìå£©
	sAddr.sin_family = SL_AF_INET;  //IPv4 Ì×½Ó×Ö (UDP, TCP, etc) 
	//ÖØĞÂÅÅÁĞ16Î»ÎŞ·ûºÅÊıµÄ×Ö½ÚË³Ğò (Êı¾İ´óĞ¡¶ËÎÊÌâ) //ÉèÖÃ16Î»µÄ¶Ë¿ÚºÅ
	sAddr.sin_port = sl_Htons((unsigned short)g_uiPortNum);
	//ÖØĞÂÅÅÁĞ32Î»ÎŞ·ûºÅÊıµÄ×Ö½ÚË³Ğò (Êı¾İ´óĞ¡¶ËÎÊÌâ) //ÉèÖÃ32Î»µÄIPµØÖ·
	sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

	iAddrSize = sizeof(SlSockAddrIn_t);


	//¡÷SimpleLink¿âº¯Êı¡°sl_Socket¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//´´½¨Ò»¸öÌ×½Ó×Ö(´´½¨Ò»¸öÍ¨ĞÅ¶Ëµã),²¢·ÖÅäÏàÓ¦µÄÏµÍ³×ÊÔ´¡£
	//ÓÉÓ¦ÓÃµ÷ÓÃ¸Ãº¯Êı£¬´Ó¶ø»ñµÃÒ»¸öÌ×½Ó×Ö¾ä±ú¡£
	//´Ëº¯Êı¹¦ÄÜ£º´´½¨Ò»¸öTCPÌ×½Ó×Ö£¨Ê¹ÓÃ IPv4 Ğ­Òé£©
	//·µ»Ø¡°ÕıÊı¡±£º±íÊ¾´´½¨³É¹¦£¬´ËÕıÊıÎªÌ×½Ó×Ö¾ä±ú¡£
	//·µ»Ø¡°¸ºÊı¡±£º±íÊ¾´´½¨Ê§°Ü£¬²»Í¬¸ºÊıÖµ£¬±íÊ¾²»Í¬µÄ´íÎóÀàĞÍ¡£
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
	if( iSockID < 0 ) //·µ»Ø¡°¸ºÊı¡±Ê±£º±íÊ¾´´½¨Ê§°Ü¡£
	{
		return -1; //³ö´í£¬·µ»Ø¸ºÖµ¡°-1¡±¡£
	}

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)  

	//¡÷SimpleLink¿âº¯Êı¡°sl_Connect¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//´´½¨ÓëÖ¸¶¨Íâ²¿¶Ë¿Ú£¨ÍøÂçµØÖ·£©µÄÁ¬½Ó¡£
	//·µ»Ø¡°0¡±£º±íÊ¾Á¬½Ó³É¹¦£»·µ»Ø¡°·ÇÁã¡±£º±íÊ¾Á¬½ÓÊ§°Ü¡£ 
	iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
	if( iStatus < 0 ) //·µ»Ø¡°¸ºÊı¡±Ê±£º±íÊ¾Á¬½ÓÊ§°Ü¡£
	{
		// error
		//¡÷SimpleLink¿âº¯Êı¡°sl_Close¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
		//¹Ø±Õ´ËÌ×½Ó×Ö£¬²¢ÊÍ·Å¸ÃÌ×½Ó×ÖµÄÏà¹Ø×ÊÔ´¡£
		//·µ»Ø¡°0¡±£ºÌ×½Ó×Ö¹Ø±Õ³É¹¦¡£
		//·µ»Ø¡°¸ºÊı¡±£ºÌ×½Ó×Ö¹Ø±ÕÊ§°Ü¡£
		sl_Close(iSockID);
		return -1;  //³ö´í£¬·µ»Ø¸ºÖµ¡°-1¡±¡£
	}

        

//MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)  
	
	//¡÷SimpleLink¿âº¯Êı¡°sl_Send¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//ÓÃÓÚÏòÒ»¸öÒÑ¾­Á¬½ÓµÄsocket·¢ËÍÊı¾İ¡£
	//·µ»Ø¡°ÕıÕûÊı¡±£º·¢ËÍ³É¹¦£¬Îª·¢ËÍµÄ×Ö½Ú¸öÊı£»·µ»Ø¡°-1¡±£º·¢ËÍÊ§°Ü¡£
	iStatus = sl_Send(iSockID, TxBuffer, TxCounter, 0 );
	if( iStatus <= 0 )
	{
		// error
		//¡÷SimpleLink¿âº¯Êı¡°sl_Close¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
		//¹Ø±Õ´ËÌ×½Ó×Ö£¬²¢ÊÍ·Å¸ÃÌ×½Ó×ÖµÄÏà¹Ø×ÊÔ´¡£
		//·µ»Ø¡°0¡±£ºÌ×½Ó×Ö¹Ø±Õ³É¹¦¡£
		//·µ»Ø¡°¸ºÊı¡±£ºÌ×½Ó×Ö¹Ø±ÕÊ§°Ü¡£ 
		sl_Close(iSockID);
		return -1;  //³ö´í£¬·µ»Ø¸ºÖµ¡°-1¡±¡£
	}
MAP_UtilsDelay(DELAY_COUNT_SOCKECT); //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)  
	
#ifdef UART_DEBUG_ENABLE
	Report("Sent packet successfully\n\r"); //´®¿ÚÌáÊ¾TCP Êı¾İ°ü·¢ËÍ³É¹¦
#endif
	
	//¡÷SimpleLink¿âº¯Êı¡°sl_Close¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//·¢ËÍÍêTCPÊı¾İ°üÖ®ºó£¬
	//¹Ø±Õ´ËÌ×½Ó×Ö£¬²¢ÊÍ·Å¸ÃÌ×½Ó×ÖµÄÏà¹Ø×ÊÔ´¡£
	//·µ»Ø¡°0¡±£ºÌ×½Ó×Ö¹Ø±Õ³É¹¦¡£
	//·µ»Ø¡°¸ºÊı¡±£ºÌ×½Ó×Ö¹Ø±ÕÊ§°Ü¡£
	sl_Close(iSockID);

	return 0;
}



/****************************************************************************
*º¯ÊıÃû-Function:		void WiFi_Stop(void)  
*ÃèÊö- Description:	       Í£Ö¹CC3200 Wi-FiÄ£¿é
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void WiFi_Stop(void)   //Í£Ö¹CC3200 Wi-FiÄ£¿é
{
	//¡÷SimpleLink¿âº¯Êı¡°sl_Stop¡±£¬Çë²éÔÄ¡°Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷¡±ÕÂ½Ú
	//Í£Ö¹SimpleLink¹¦ÄÜ£º
	//Çå³ıÏà¹ØÊ¹ÄÜµÄÒı½Å¡¢¹Ø±ÕÍ¨ĞÅ½Ó¿Ú¡¢µ÷ÓÃÍê³ÉÍ£Ö¹µÄ»Øµ÷º¯Êı¡£
	sl_Stop(SL_STOP_TIMEOUT);
}



/****************************************************************************
*º¯ÊıÃû-Function:		void Send_Alarm_To_Cloud(void)
*ÃèÊö- Description:	       CC3200·¢ËÍ±¨¾¯ĞÅÏ¢¸øÔÆ·şÎñÆ÷
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void Send_Alarm_To_Cloud(void)   //CC3200·¢ËÍ±¨¾¯ĞÅÏ¢¸øÔÆ·şÎñÆ÷
{
//ÍêÕûµÄWi-Fi ¹¤×÷Á÷³Ì/////////////////////////////////////////////////
//	WiFi_Start();  //Æô¶¯ CC3200 Wi-FiÄ£¿é

//  	WiFi_Connect_To_AP();  //½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£


	WiFi_Send_Data_TcpClient();  //½«²É¼¯µ½µÄÊı¾İ£¬Í¨¹ıWIFIÄ£¿é·¢ËÍ³öÈ¥

//	WiFi_Stop();  //Í£Ö¹CC3200 Wi-FiÄ£¿é
/////////////////////////////////////////////////////////////////////////////////////
  
}



/****************************************************************************
*º¯ÊıÃû-Function:		void System_DeepSleep(void)
*ÃèÊö- Description:	       MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*****************************************************************************/
void System_DeepSleep(void)   //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
{

	//½øÈëĞİÃß×´Ì¬Ê±£¬¹Øµô²»ÓÃµÄÊ±ÖÓ:UART¡¢GPIOA2¡¢IIC(BMA222)
	//Ê¹ÏµÍ³½øÈëÉî¶ÈË¯Ãß×´Ì¬¡£
	MAP_PRCMDeepSleepEnter();
}






/****************************************************************************
*º¯ÊıÃû-Function:		void WiFi_XXX(void)
*ÃèÊö- Description:	       
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)   ¡ø02)    ¡ø03)    ¡ø04)  
*******************************************
void WiFi_XXX(void)   //
{
	long retVal = -1;

}
**********************************/




/****************************************************************************
*º¯ÊıÃû-Function:		int main(void)
*ÃèÊö- Description:		Ö÷º¯Êı
*ÊäÈë²ÎÊı-Input:	None
*Êä³ö²ÎÊı-output:	None
*×¢ÒâÊÂÏî-Note£º	
	¡ø01)     MAP_UtilsDelay(8000000); //ÑÓÊ±Ô¼Îª500ms //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)
	¡ø02)    GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)
        		GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO); //µãÁÁLED2(»ÆÉ«)
        		GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  //µãÁÁLED3(ÂÌÉ«)
	¡ø03)     ¡ø04)   ¡ø05)    ¡ø06)  
*****************************************************************************/

#define PIN_SW2  22
int main()
{
	System_Initial();  //  ÏµÍ³³õÊ¹»¯ :µ¥Æ¬»úÄÚ²¿×ÊÔ´+Íâ²¿»ù±¾×ÊÔ´µÄ³õÊ¹»¯


	// Ê¹ÄÜGPIOA1 Ä£¿éµÄÉî¶ÈË¯ÃßÊ±ÖÓ
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);


Button_PinMuxConfig();  // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃSW2(Pin15--GPIO22)¡¢SW3(Pin4--GPIO13)°´¼üµÄÒı½ÅºÍÊ±ÖÓ)

unsigned int uiGPIOPort;
unsigned char pucGPIOPin;
unsigned char ucPinValue;
    //Read GPIO
GPIO_IF_GetPortNPin(PIN_SW2,&uiGPIOPort,&pucGPIOPin);
ucPinValue = GPIO_IF_Get(PIN_SW2,uiGPIOPort,pucGPIOPin);
        
    //Èç¹û¿ª»úÊ±£¬SW2±»°´ÏÂ£¬½øÈëÅäÖÃSSIDºÍÃÜÂë
if(ucPinValue == 1)
{
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)

	


}
GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ï¨ÃğLED1(ºìÉ«)	




//Button_IF_Init_sw2(SW2_Int_Handler);      //SW2°´¼ü³õÊ¼»¯:IOÖĞ¶Ï

//care: BMA222µÄÖĞ¶ÏÒı½ÅÁ¬½Óµ½SW3(¹ÊBMA222µÄIOÖĞ¶ÏÏà¹Ø£¬Ö±½ÓÔÚSW3ÖĞÉèÖÃButton_IF_Init)
Button_IF_Init_sw3(SW3_BMA222_Int_Handler);//SW3°´¼ü³õÊ¼»¯:IOÖĞ¶Ï
   

/*****************/
//#ifdef UART_DEBUG_ENABLE
	Uart_PinMuxConfig();   // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃUART µÄÒı½ÅºÍÊ±ÖÓ)
	InitTerm();  // ÅäÖÃUART Ïà¹Ø²ÎÊı£º²¨ÌØÂÊÎª115200£¬8Î»Êı¾İ£¬1Î»Í£Ö¹Î»£¬ÎŞÆæÅ¼Ğ£Ñé¡£
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
    //³õÊ¼»¯CC3200ÍøÂç²ã
    //¡÷SimpleLink¿âº¯Êı"sl_Start"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
    //Æô¶¯ SimpleLink Éè±¸£º
    //³õÊ¼»¯Wi-FIÍ¨ĞÅ½Ó¿Ú£¬ÉèÖÃWi-FIÊ¹ÄÜÒı½Å£¬·µ»ØÉèÖÃ×´Ì¬£¨ÊÇ·ñÆô¶¯³É¹¦£©
    iRetVal = sl_Start(NULL, NULL, NULL);
    if(iRetVal < 0)
    {
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    //
    // reset all network policies
    //¡÷SimpleLink¿âº¯Êı"sl_WlanPolicySet"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
    //ÉèÖÃWLANµÄÏà¹Ø²ßÂÔ
    //ÉèÖÃÁ¬½Ó²ßÂÔ£ºNULL    
    sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                    SL_CONNECTION_POLICY(0,0,0,0,0),
                    &policyVal,
                    1 ); //PolicyValLen
      //¡÷SimpleLink¿âº¯Êı"SL_CONNECTION_POLICY"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
    //ÉèÖÃÁ¬½Ó²ßÂÔ£ºAutoºÍSmartConfig £¨CC3200Ä¬ÈÏÏÂµÄÁ¬½Ó²ßÂÔ£©    



    //  create a user file
    //¡÷SimpleLink¿âº¯Êı"sl_FsOpen"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
    //´ò¿ªÒªĞ´ÈëµÄÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)
    //¡÷SimpleLink¿âº¯Êı"FS_MODE_OPEN_CREATE"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
    //ĞÂ½¨Ò»¸öÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)//´óĞ¡Îª65536
    iRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
    			FS_MODE_OPEN_CREATE(65536, \
                          _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                        &ulToken,
                        &lFileHandle);
    if(iRetVal < 0)
    {
        //
        // File may already be created
	//¡÷SimpleLink¿âº¯Êı"sl_FsClose"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
	//¹Ø±ÕÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    }
    else
    {
        //
        // close the user file
        //¹Ø±ÕĞÂ½¨µÄÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)
        iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        if (SL_RET_CODE_OK != iRetVal)
        {
        	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -1;
        }
    }
    
    //
    //  open a user file for writing
    //´ò¿ªĞÂ½¨µÄÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)£¬²¢Ê¹ÄÜĞ´²Ù×÷
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

   	 //¡÷SimpleLink¿âº¯Êı"sl_FsWrite"£¬Çë²éÔÄ"Ê¹ÓÃµ½µÄ¿âº¯ÊıËµÃ÷"ÕÂ½Ú
   	 //ÏòÓÃ»§ÎÄ¼şÖĞµÄÖ¸¶¨µØÖ·£¬Ğ´ÈëÖ¸¶¨´óĞ¡µÄÎÄ¼ş
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
    //¹Ø±ÕÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }

    // open a user file for reading
    //´ò¿ªĞÂ½¨µÄÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)£¬²¢Ê¹ÄÜ¶Á²Ù×÷
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

	//±È½ÏÁ½¸öÊı×éÊÇ·ñÏàÍ¬
        iRetVal = memcmp(AlarmFile, 
                         TempFile, 
                         sizeof(AlarmFile));
        if (iRetVal != 0)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            return -2;
        }


    // close the user file
    //¹Ø±ÕÓÃ»§ÎÄ¼ş(Íâ²¿FlashÖĞ)
    iRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != iRetVal)
    {
    	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return -1;
    }
    
    //
    // turn ON the green LED indicating success
    //ÂÌÉ«LEDÁÁ:±íÊ¾¶ÁĞ´ĞÂ½¨µÄÓÃ»§ÎÄ¼ş²Ù×÷³É¹¦
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
	IIC_PinMuxConfig();  // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃIIC µÄÒı½ÅºÍÊ±ÖÓ)
	I2C_IF_Open(I2C_MASTER_MODE_FST);  //³õÊ¼»¯ I2C Ä£¿é £¨¿ìËÙÄ£Ê½£º400kbps£©


	bma222_init();   //³õÊ¼»¯bma222 (bma222ÈıÖá¼ÓËÙ¶È´«¸ĞÆ÷) 
	
	//care: BMA222µÄÖĞ¶ÏÒı½ÅÁ¬½Óµ½SW3(¹ÊBMA222µÄIOÖĞ¶ÏÏà¹Ø£¬Ö±½ÓÔÚSW3ÖĞÉèÖÃButton_IF_Init)
	//Ê¹ÄÜINT1,ÓÃÓÚ"ÈÎÒâ¶¯×÷"µÄÖĞ¶Ï
	bma222_set_RegVaule(BMA222_19_INTR_MAP,BMA222_INT1_SLOPE ); 


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

	AppVariables_Initial();  //³õÊ¼»¯Ïà¹Ø±äÁ¿(ÍøÂçÏà¹ØµÄ±äÁ¿:IP¡¢¶Ë¿ÚºÅ¡¢SSIDµÈ)

	WiFi_Combined_Data_Packet();  // ×é×°Êı¾İ°ü(TCP+Http)

/************/

WiFi_Initial();  //³õÊ¼»¯CC3200 Wi-FiÄ£¿é(ÅäÖÃCC3200»Ö¸´µ½ÆäÄ¬ÈÏµÄ×´Ì¬)

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
WiFi_Start();  //Æô¶¯ CC3200 Wi-FiÄ£¿é

WiFi_Connect_To_AP();  //½«CC3200¿ª·¢°åÁ¬½Óµ½Ö¸¶¨µÄAP£¨WLAN£©,²¢»ñÈ¡µ½ÏàÓ¦µÄIPµØÖ·¡£


	//ºìÉ«LED ÉÁË¸Á½´Î(Ã¿ÃëÉÁË¸Ò»´Î)£¬±íÊ¾CC3200ÒÑÁ¬½Óµ½Ö¸¶¨AP(Â·ÓÉÆ÷)
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)
	MAP_UtilsDelay(20000000); //Ô¼2s//ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ï¨ÃğLED1(ºìÉ«)	




/************

	System_DeepSleep();  //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)

System_Initial();  //  ÏµÍ³³õÊ¹»¯ :µ¥Æ¬»úÄÚ²¿×ÊÔ´+Íâ²¿»ù±¾×ÊÔ´µÄ³õÊ¹»¯

// Ê¹ÄÜGPIOA1 Ä£¿éµÄÉî¶ÈË¯ÃßÊ±ÖÓ
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃSW2(Pin15--GPIO22)¡¢SW3(Pin4--GPIO13)°´¼üµÄÒı½ÅºÍÊ±ÖÓ)
//care: BMA222µÄÖĞ¶ÏÒı½ÅÁ¬½Óµ½SW3(¹ÊBMA222µÄIOÖĞ¶ÏÏà¹Ø£¬Ö±½ÓÔÚSW3ÖĞÉèÖÃButton_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2¡¢SW3°´¼ü³õÊ¼»¯:IOÖĞ¶Ï




	System_DeepSleep();  //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)


System_Initial();  //  ÏµÍ³³õÊ¹»¯ :µ¥Æ¬»úÄÚ²¿×ÊÔ´+Íâ²¿»ù±¾×ÊÔ´µÄ³õÊ¹»¯

// Ê¹ÄÜGPIOA1 Ä£¿éµÄÉî¶ÈË¯ÃßÊ±ÖÓ
MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_DSLP_MODE_CLK);

MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_DSLP_MODE_CLK);

Button_PinMuxConfig();  // ÅäÖÃÒı½ÅÓ³Éä(ÅäÖÃSW2(Pin15--GPIO22)¡¢SW3(Pin4--GPIO13)°´¼üµÄÒı½ÅºÍÊ±ÖÓ)
//care: BMA222µÄÖĞ¶ÏÒı½ÅÁ¬½Óµ½SW3(¹ÊBMA222µÄIOÖĞ¶ÏÏà¹Ø£¬Ö±½ÓÔÚSW3ÖĞÉèÖÃButton_IF_Init)
Button_IF_Init(SW2_Int_Handler, SW3_BMA222_Int_Handler );  //SW2¡¢SW3°´¼ü³õÊ¼»¯:IOÖĞ¶Ï



	System_DeepSleep();  //MCU ½øÈëµ½"Éî¶ÈË¯Ãß"Ä£Ê½
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //µãÁÁLED1(ºìÉ«)

	
	while (1)
	{
	
	}
	*************/
	//³õÊ¼»¯"¹¤×÷"×´Ì¬Öµ
//WorkState = WORK_ReadAllVaule_BMA222;  //Work¨ˆ¶ÁÈ¡BMA222ËùÓĞ¼ÓËÙ¶ÈÊı¾İ(¹¤×÷Ê±)

	while (1)
	{

	////////////////////////////////////////////////////////////////////////////
	//==**"´¦Àí"¸÷¹¤×÷Work¨ˆ"×´Ì¬"ÏÂµÄÊÂÎñ**===============//
		Deal_WorkState(); // "´¦Àí"¸÷¹¤×÷Work¨ˆ"×´Ì¬"ÏÂµÄÊÂÎñ (×´Ì¬»ú)


		//MAP_UtilsDelay(10000000); //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)
		//ÔÚÃ»ÓĞÊ¹ÓÃOS£¨²Ù×÷ÏµÍ³£©µÄÇé¿öÏÂ£¬¸Ãº¯Êı±ØĞëÔÚÑ­»·ÖĞ±»µ÷ÓÃ¡£
		//·µ»Ø0£º±íÊ¾ÒÑÍê³É²Ù×÷£»·µ»Ø1£º±íÊ¾²Ù×÷ÈÔÔÚ½øĞĞÖĞ¡£
		_SlNonOsMainLoopTask();
	}
}


	//MAP_UtilsDelay(100000); //ÑÓÊ±×Ó³ÌĞò(¶ÌÔİÑÓÊ±)
//bma222_data->scaled = 1;  //Îª1Ê±£¬µÃµ½×îºóµÄÊµ¼ÊÎïÀíÁ¿
//bma222_read(bma222_hal, SENSOR_READ_ACCELERATION, bma222_data);


//#ifdef UART_DEBUG_ENABLE  
	//UART_PRINT("x:  %d    y:  %d    z:  %d\n\r",bma222_data->axis.x, bma222_data->axis.y, bma222_data->axis.z);
//#endif	



