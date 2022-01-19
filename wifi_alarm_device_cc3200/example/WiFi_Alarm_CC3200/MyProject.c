/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* �ļ���-FileName:			 
* �����ļ�-Dependencies:  	 MyProject.h; System_HeadFile.h;	
* �ļ�����-File Description:	 ( Դ����-Source File)
	�� ������Ŀ: �ܹ��ܳ���
	01) ϵͳ״ֵ̬����
	02) EEPROM ��ַ����
	03) Flash ��ַ����   
	04) ������Ŀ�ı�������    
	05)    06)	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ע������-Attention : 	
	��01)     ��02)     ��03)    ��04)    ��05)    ��06)      
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
* ��������Э��-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


#include "MyProject.h"	//������Ŀ: �ܹ��ܳ���-ͷ�ļ�


#include "gpio_if.h"


#include "bma222.h"    //"bma222������ٶȴ�����" -��������-ͷ�ļ�(�ⲿ��Դ)


////////////////////////////////////////////////////////////////////////////
//==**"��ģ��ר��"�궨��**Module-specific macro**==============//
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//==**��Ŀ"ר��"�궨��**Project-specific macro**================//
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//==**ȫ�ֱ�������**Global variables**========================//
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//==**ϵͳ״̬������**=================================//
unsigned int MainState = 0;      //Mainϵͳ״̬���� 
unsigned int Last_MainState = 0; //������һ��ϵͳ״̬���� 

//unsigned char G_UsersType = 0;     //�û�����:�߼�����Ա����ͨ����Ա



////////////////////////////////////////////////////////////////////////////
//==**��������ʱ,�õ���״̬����**====================//
unsigned int WorkState = 0;	      //ϵͳ����״̬���� 
unsigned int Last_WorkState = 0;  //������һ�ι���״̬���� 




////////////////////////////////////////////////////////////////////////////
//==**�ֲ���������**Local variables**========================//
////////////////////////////////////////////////////////////////////////////








/****************************************************************************
*������-Function:		void Deal_WorkState(void)
*����- Description:	 	"����"������Work��"״̬"�µ����� (״̬��)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    ��02)    ��03)    ��04)  
*****************************************************************************/
void Deal_WorkState(void)
{

////////////////////////////////////////////////////////////////////////////
//==**��������: **0x8000--0x8FFF** ===========================//
////////////////////////////////////////////////////////////////////////////

	switch(WorkState) 
	{
////////////////////////////////////////////////////////////////////////////
//==**"����ʱ"�Ŀ���״̬** ============================//

case WORK_Test_IdleState: //Work��"����ʱ"�Ŀ���״̬(����ʱ)

	break;

case WORK_ReadAllVaule_BMA222: //Work����ȡBMA222���м��ٶ�����(����ʱ)
	ReadAllEventVaule_BMA222(); //��ȡBMA222���м��ٶ�����(bma222������ٶȴ�����) 
	
	WorkState = WORK_Test_IdleState;  //���ص�״̬: 
	break;

case WORK_Alarm: //Work������ (����ʱ)
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);    //����LED1(��ɫ)
		Send_Alarm_To_Cloud();  //CC3200���ͱ�����Ϣ���Ʒ�����
	GPIO_IF_LedOff(MCU_RED_LED_GPIO);    //Ϩ��LED1(��ɫ)

	WorkState = WORK_LowPower;  //���ص�״̬:  //Work���������ߵ͹���״̬(����ʱ)
	break;

case WORK_LowPower: //Work���������ߵ͹���״̬(����ʱ)


	WorkState = WORK_Test_IdleState;  //���ص�״̬: 
	
	break;
	
                          
//case : //Work��

	//break;

 

////////////////////////////////////////////////////////////////////////////
//==**"��ز���" ( �����˵�)** ===========================//


////////////////////////////////////////////////////////////////////////////
//==**"��ز���" ( �����˵�)** ===========================//





////////////////////////////////////////////////////////////////////////////
default:
	break;
	}
}

