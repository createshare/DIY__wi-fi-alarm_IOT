/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* �ļ���-FileName:			 bma222.c
* �����ļ�-Dependencies:  	 bma222.h;
* �ļ�����-File Description:	 ( Դ����-Source File) 
	��  "bma222������ٶȴ�����" -��������(�ⲿ��Դ)
	        Bosch BMA222 3-axis accelerometer driver.
	01)     02)     03)    04)    05)    06)	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ע������-Attention : 	
	��01)   	��02)     ��03)    ��04)    ��05)    ��06)     
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
* �޸ļ�¼-Change History:   
	����     ʱ��          �汾    ��������
	Author 	  Date		   Rev          Comment
	--------------------------------------------------------
	BlueS ��2014-07-01	   1.0	   
			 xxxx-xx-xx	   x.x	   
			 xxxx-xx-xx	   x.x			
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* ��˾-Company: 			CS-EMLAB  Co. , Ltd.
* ������Э��-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "utils.h"
#include "rom_map.h"


#include "i2c_if.h"


#include "bma222.h"    //"bma222������ٶȴ�����" -��������-ͷ�ļ�(�ⲿ��Դ)


////////////////////////////////////////////////////////////////////////////
//==**ȫ�ֱ�������**Global variables**========================//
////////////////////////////////////////////////////////////////////////////

sensor_hal_t   bma222_hal_s;
sensor_hal_t *bma222_hal = &bma222_hal_s;       //Sensor Hardware Abstraction Descriptor

sensor_data_t  bma222_data_s;  
sensor_data_t *bma222_data = &bma222_data_s;   //Sensor Data Descriptor


////////////////////////////////////////////////////////////////////////////
//==**�ֲ���������**Local variables**========================//
////////////////////////////////////////////////////////////////////////////

//brief Sensor Event Registers 
static struct {
	bma_axis_t acc[3];                 //Acceleration data 
	int8_t temp;                       //< Temperature data 

	union {
		uint8_t status_byte[4];        // Status bytes 
		struct {                       //Status fields 
			uint8_t low_int       : 1; // Low-g criteria triggered 
			uint8_t high_int      : 1; // High-g criteria triggered
			uint8_t slope_int     : 1; //Slope criteria triggered
			uint8_t reserved_09   : 1;
			uint8_t d_tap_int     : 1; // double-tap interrupt triggered /
			uint8_t s_tap_int     : 1; //Single-tap interrupt triggered /
			uint8_t orient_int    : 1; // Orientation interrupt triggered /					
			uint8_t flat_int      : 1; //Flat interrupt triggered /

						
			uint8_t reserved_0a   : 7;
			uint8_t data_int      : 1; //New data interrupt triggered 
			

			uint8_t slope_first_x : 1; //x-axis any-motion interrupt
			uint8_t slope_first_y : 1; //y-axis any-motion interrupt 
			uint8_t slope_first_z : 1; //z-axis any-motion interrupt 
			uint8_t slope_sign    : 1; //Axis motion direction
			uint8_t tap_first_x   : 1; //x-axis tap interrupt
			uint8_t tap_first_y   : 1; //y-axis tap interrupt
			uint8_t tap_first_z   : 1; //z-axis tap interrupt 
			uint8_t tap_sign      : 1; //Tap axis motion direction 
			
			 
			uint8_t high_first_x  : 1; // x-axis high-g interrupt 
			uint8_t high_first_y  : 1; // y-axis high-g interrupt  
			uint8_t high_first_z  : 1; // z-axis high-g interrupt 
			uint8_t high_sign     : 1;//High-g interrupt sign 
			uint8_t orient        : 3; //< orientation with respect  to gravity
			uint8_t flat          : 1; //Orientation with respect to gravity 
					
		} status_field;
	};
} event_regs;

/***************************************************************
//brief Sensor Event Registers 
static struct {
	bma_axis_t acc[3];                 //Acceleration data 
	int8_t temp;                       //< Temperature data 

	union {
		uint8_t status_byte[4];        // Status bytes 
		struct {                       //Status fields 
			uint8_t flat_int      : 1; //Flat interrupt triggered /
			uint8_t orient_int    : 1; // Orientation interrupt triggered /
			uint8_t s_tap_int     : 1; //Single-tap interrupt triggered /
			uint8_t d_tap_int     : 1; // double-tap interrupt triggered /
			uint8_t reserved_09   : 1;
			uint8_t slope_int     : 1; //Slope criteria triggered
			uint8_t high_int      : 1; // High-g criteria triggered
			uint8_t low_int       : 1; // Low-g criteria triggered 

			uint8_t data_int      : 1; //New data interrupt triggered 
			uint8_t reserved_0a   : 7;

			uint8_t tap_sign      : 1; //Tap axis motion direction 
			uint8_t tap_first_z   : 1; //z-axis tap interrupt 
			uint8_t tap_first_y   : 1; //y-axis tap interrupt 
			uint8_t tap_first_x   : 1; //x-axis tap interrupt
			uint8_t slope_sign    : 1; //Axis motion direction 
			uint8_t slope_first_z : 1; //z-axis any-motion interrupt 
			uint8_t slope_first_y : 1; //y-axis any-motion interrupt 
			uint8_t slope_first_x : 1; //x-axis any-motion interrupt

			uint8_t flat          : 1; //Orientation with respect to gravity 
			uint8_t orient        : 3; //< orientation with respect  to gravity
			                          
			uint8_t high_sign     : 1;//High-g interrupt sign 
			uint8_t high_first_z  : 1; // z-axis high-g interrupt 
			uint8_t high_first_y  : 1; // y-axis high-g interrupt 
			uint8_t high_first_x  : 1; // x-axis high-g interrupt 
		} status_field;
	};
} event_regs;

***************************************************************/







/****************************************************************************
*������-Function:		static void format_axis_data(const sensor_hal_t *hal, const bma_axis_t acc[], sensor_data_t *data)
*����- Description:		ת�����ٶ����ݸ�ʽ (bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)   �ȰѼ��ٶ����ݵķ���λ����ֵ��ˣ��õ������ŵ�ԭʼ���� 
	              �ٸ���data->scaledλ���Դ����ŵ�ԭʼ���ݣ��ٽ�һ��ת�����ݸ�ʽ
	              ��data->scaled = 1ʱ��������ת��Ϊ����ʵ������ֵ
	��02)    	��03)    ��04)  
*****************************************************************************/
static void format_axis_data(const sensor_hal_t *hal, const bma_axis_t acc[], sensor_data_t *data)
{
	/* Get axis values based based on device orientation configuration. */
	int32_t const acc_x = hal->orientation.x.sign *
			bma_axis_val(acc[hal->orientation.x.axis]);

	int32_t const acc_y = hal->orientation.y.sign *
			bma_axis_val(acc[hal->orientation.y.axis]);

	int32_t const acc_z = hal->orientation.z.sign *
			bma_axis_val(acc[hal->orientation.z.axis]);


	/* Convert raw sensor sample to engineering units if requested. */
	if (data->scaled) {
		data->axis.x = raw_to_scaled(hal, acc_x);
		data->axis.y = raw_to_scaled(hal, acc_y);
		data->axis.z = raw_to_scaled(hal, acc_z);
	} else {
		data->axis.x = acc_x;
		data->axis.y = acc_y;
		data->axis.z = acc_z;
	}
}




        

/****************************************************************************
*������-Function:		void bma222_set_range(unsigned char range)
*����- Description:		����: ���Է�Χ��������ѡ�� (bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01)// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_set_range(unsigned char range)
{
	unsigned char array[2];

	
	//����:���̡���BMA222��G_RANGE�Ĵ�����д��1���ֽ����ݡ�
	array[0] = BMA222_G_RANGE;
	array[1] = range;
	
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//��������£����Ҫ���ͽ���λ(��ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_G_RANGE, &range, 1);

	//������ĺ������ֵ����λΪmg
	switch (range) 
	{
		case BMA222_RANGE_2G:
			bma222_hal->range = 2000; //������Χ: +/-2g = +/-2000mg
			break;
			
		case BMA222_RANGE_4G:
			bma222_hal->range = 4000; //������Χ: +/-2g = +/-4000mg
			break;
			
		case BMA222_RANGE_8G:
			bma222_hal->range = 8000; //������Χ: +/-2g = +/-8000mg
			break;

		case BMA222_RANGE_16G:
			bma222_hal->range = 16000; //������Χ: +/-2g = +/-16000mg
			break;
			
		default:
			break;
	}

}


/****************************************************************************
*������-Function:		void bma222_set_bandwidth(unsigned char bandwidth)
*����- Description:		����: ���� (bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_set_bandwidth(unsigned char bandwidth)
{
	unsigned char array[2];


	array[0] = BMA222_BANDWIDTH;
	array[1] = bandwidth;

	//����:������BMA222��BANDWIDTH�Ĵ�����д��1���ֽ����ݡ�
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//��������£����Ҫ���ͽ���λ(��ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_BANDWIDTH, &bandwidth, 1);

	
	//������ĺ�Ĵ�����λΪHz
	switch (bandwidth) 
	{
		case BMA222_BANDWIDTH_8Hz:  //7.81 Hz    filtered data bandwidth
			bma222_hal->bandwidth = 8; 
			break;

		case BMA222_BANDWIDTH_16Hz:  // 15.63 Hz
			bma222_hal->bandwidth = 16; 
			break;

		case BMA222_BANDWIDTH_31Hz:  //31.25 Hz
			bma222_hal->bandwidth = 31; 
			break;

		case BMA222_BANDWIDTH_63Hz: //62.50 Hz
			bma222_hal->bandwidth = 63; 
			break;			
			
		case BMA222_BANDWIDTH_125Hz:  //125.00 Hz
			bma222_hal->bandwidth = 125;   
			break;
			
		case BMA222_BANDWIDTH_250Hz:  //250.00 Hz
			bma222_hal->bandwidth = 250;   
			break;
			
		case BMA222_BANDWIDTH_500Hz: //500.00 Hz
			bma222_hal->bandwidth = 500;
			break;

		case BMA222_BANDWIDTH_1000Hz:  //1000.00 Hz
			bma222_hal->bandwidth = 1000; 
			break;
			
		default:
			break;
	}
	
}



/****************************************************************************
*������-Function:		void bma222_set_RegVaule(unsigned char reg, unsigned char value)
*����- Description:		����: ָ���Ĵ�����ֵ (bma222������ٶȴ�����)
*�������-Input:	reg:  �Ĵ�����ַ,  value:Ҫ���õļĴ���ֵ
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_set_RegVaule(unsigned char reg, unsigned char value)
{
	unsigned char array[2];


	array[0] = reg;
	array[1] = value;

	//����:������BMA222��BANDWIDTH�Ĵ�����д��1���ֽ����ݡ�
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//��������£����Ҫ���ͽ���λ(��ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_BANDWIDTH, &bandwidth, 1);

}









/****************************************************************************
*������-Function:		unsigned char bma222_get_device_id(void)
*����- Description:		��ȡ: оƬ��ID��(bma222������ٶȴ�����) //Read BMA222 device ID and revision numbers.
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) BMA222��ID��Ĭ��Ϊ0x03
	��02)    	��03)    ��04)  
*****************************************************************************/
unsigned char bma222_get_RegVaule(unsigned char reg)
{
//	unsigned char idNum;
	unsigned char array[2];
	unsigned char cc[2];


	array[0] = reg;
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// ��������ӻ�дҪ��ȡ�ļĴ�����ַ��
	I2C_IF_Write(BMA222_TWI_ADDR,array,1,0);  //��������£�д��Ĵ�����ַ�󣬲�Ҫ���ͽ���λ(��ucStop=0)

	// Read the specified length of data
	//����ָ���Ĵ�����ֵ
	I2C_IF_Read(BMA222_TWI_ADDR,cc, 1);

	return cc[0];
}


/****************************************************************************
*������-Function:		unsigned char bma222_get_device_id(void)
*����- Description:		��ȡ: оƬ��ID��(bma222������ٶȴ�����) //Read BMA222 device ID and revision numbers.
*�������-Input:	None
*�������-output:	None
*ע������-Note��	
	��01) BMA222��ID��Ĭ��Ϊ0x03
	��02)    	��03)    ��04)  
*****************************************************************************/
unsigned char bma222_get_device_id(void)
{
//	unsigned char idNum;
	unsigned char array[2];
	unsigned char cc[2];

	array[0] = BMA222_CHIP_ID;
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// ��������ӻ�дҪ��ȡ�ļĴ�����ַ��
	I2C_IF_Write(BMA222_TWI_ADDR,array,1,0);  //��������£�д��Ĵ�����ַ�󣬲�Ҫ���ͽ���λ(��ucStop=0)

	// Read the specified length of data
	//����ָ���Ĵ�����ֵ
	I2C_IF_Read(BMA222_TWI_ADDR,cc, 1);

	return cc[0];
}

/****************************************************************************
*������-Function:		void bma222_bus_read(unsigned char addr, void *data, unsigned char count)
*����- Description:		ͨ��IIC��ָ����ַ��������ȡ�������(bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_bus_read(unsigned char addr, void *data, unsigned char count)
{
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// ��������ӻ�дҪ��ȡ�ļĴ�����ַ��
	I2C_IF_Write(BMA222_TWI_ADDR,&addr,1,0);  //��������£�д��Ĵ�����ַ�󣬲�Ҫ���ͽ���λ(��ucStop=0)

	// Read the specified length of data
	//����ָ���Ĵ�����ֵ
	I2C_IF_Read(BMA222_TWI_ADDR, data, count);
}



/****************************************************************************
*������-Function:		static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data)
*����- Description:		�Ӵ�������ȡ:���ٶ�����(bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data)
{

	hal->burst_addr = BMA222_NEW_DATA_X;  //������ȡ���ݵ���ʼ��ַ(�Ĵ���)

	bma222_bus_read(hal->burst_addr, event_regs.acc, sizeof(event_regs.acc));


	format_axis_data(hal, event_regs.acc, data);
}








/****************************************************************************
*������-Function:	void bma222_init(void)
*����- Description:		��ʼ��bma222 (bma222������ٶȴ�����)
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_init(void)
{

//==BMA222���������λ========================//
	
	//the default range and bandwidth after device reset are +/- 2g and 1kHz respectively.(bma222)

	//bma222_set_state(sensor, SENSOR_STATE_RESET);

	bma222_hal->range      = 2000;   //������Χ: +/-2g= +/-2000mg (��λ���Ĭ��ֵ)
	bma222_hal->bandwidth  = 1000; //����: 1000 Kz   (��λ���Ĭ��ֵ) // Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	bma222_hal->resolution = BMA222_DATA_RESOLUTION;  //���ٶ�����λ��: 8λ
	bma222_hal->burst_addr = BMA222_NEW_DATA_X;  //������ȡ���ݵ���ʼ��ַ(�Ĵ���)

	bma222_hal->orientation.x.axis =  SENSOR_AXIS_X;
	bma222_hal->orientation.x.sign=  SENSOR_SIGN_POS;

	bma222_hal->orientation.y.axis =  SENSOR_AXIS_Y;
	bma222_hal->orientation.y.sign=  SENSOR_SIGN_POS;

	bma222_hal->orientation.z.axis =  SENSOR_AXIS_Z;
	bma222_hal->orientation.z.sign=  SENSOR_SIGN_POS;




//==BMA222��λ�����������������========================//
	
	bma222_set_RegVaule(BMA222_SOFTRESET,BMA222_RESET);    //�����λbma222 (bma222������ٶȴ�����) 
	MAP_UtilsDelay(800000); //Լ50ms//��ʱ�ӳ���(������ʱ)
	bma222_set_RegVaule(BMA222_SOFTRESET,BMA222_RESET);    //�����λbma222 (bma222������ٶȴ�����) 
	MAP_UtilsDelay(800000); //Լ50ms//��ʱ�ӳ���(������ʱ)


	//���PS����Ϊ�͵�ƽ��ߵ�ƽʱ����BMA222ѡ��Ĺ���ģʽΪͨ��ģʽ��
       //PS����Ϊ�ߵ�ƽ,��BMA222ѡ��I2C��Ϊͨ�Žӿ�
       
	//��λ�󣬼Ĵ���(0x13)�ļĴ���λdata_high_bw=0//���þ����˲���������
	//��λ�󣬼Ĵ���(0x19~0x1B)��Ϊ0//�ж�����INT1��INT2���жϹ���ӳ��: ��������
	//��λ�󣬼Ĵ���(0x20)=0x05
		//�Ĵ���λint2_lvl=1,int1_lvl=1//���ж�ʱ��INT1 ��INT2 ����ߵ�ƽ
		//�Ĵ���λint2_od=0,int1_od=0//INT1 ��INT2����ģʽ:�������(�ߵ͵�ƽ��IC�ĵ�Դ����)
		
	//��λ�󣬼Ĵ���(0x27)=0x00,slope_dur=0//  �ʣ������õ��ض���ֵN=0+1=1
	//��λ�󣬼Ĵ���(0x28)=slope_th=0x14//���ڲ�����ΧΪ��2gʱ��slope_thÿ����1���൱��gֵ����15.6mg
		//��slope_th=0x14����Ӧ��ֵ15.6mg*0x14=312mg(���ٶ�ֵ�仯����312mgʱ�������ж�)

	//��λ�󣬼Ĵ���(0x17)=0x00��(0x16)=0x00//���м�ⱻ����
		//�Ĵ���(0x17): reserved  reserved  reserved   data_en  low_en   high_en_z  high_en_y  high_en_x
		//�Ĵ���(0x16): flat_en   orient_en    s_tap_en   d_tap_en  reserved   slope_en_z   slope_en_y   slope_en_x
		//data_en=0//�������ݸ����ж�(��Z��ļ��ٶ����ݼĴ������ݸ���ʱ�������������͵��жϡ������ݲɼ�����һ�����ڵĿ�ʼʱ���жϻ��Զ������)
	//��λ�󣬼Ĵ���(0x13)�ļĴ���λdata_high_bw=0//
	//��λ�󣬼Ĵ���(0x13)�ļĴ���λdata_high_bw=0//
	//��λ�󣬼Ĵ���(0x13)�ļĴ���λdata_high_bw=0//




	//bma222_set_range(BMA222_RANGE_2G); //������Χ: +/-2g= +/-2000mg

	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	//bma222_set_bandwidth(BMA222_BANDWIDTH_1000Hz);  //����: 1000 Hz


//==========================//
	bma222_set_range(BMA222_RANGE_2G); //������Χ: +/-2g= +/-2000mg



/**1�ź�2�ż���ƶ����������**************************************************
	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	//bma222_set_bandwidth(BMA222_BANDWIDTH_16Hz);  //����: 16 Hz
bma222_set_bandwidth(BMA222_BANDWIDTH_8Hz);  //����: 8 Hz
	//bma222_set_RegVaule(BMA222_17_INTR_EN, BMA222_DATA_EN); //ʹ�����ݸ����ж�(��Z��ļ��ٶ����ݼĴ������ݸ���ʱ�������������͵��жϡ������ݲɼ�����һ�����ڵĿ�ʼʱ���жϻ��Զ������)


	//ʹ��"���⶯��"��"x��"��"y��"��"z��"�������
	bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z+BMA222_SLOPE_EN_Y+BMA222_SLOPE_EN_X); //
	//bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z);
	
	//��������ʱ�õĲ���//bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x0B);  //����"���⶯��"��б����ֵ
	bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x06);  //����"���⶯��"��б����ֵ
	bma222_set_RegVaule(BMA222_SLOPE_DURATION,0x03);    //����"���⶯��"���˲�����
**************************************************/
        
        

/**3�ż������������ƶ�***********************/
	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	bma222_set_bandwidth(BMA222_BANDWIDTH_1000Hz);  //����: 1000 Hz

	//bma222_set_RegVaule(BMA222_17_INTR_EN, BMA222_DATA_EN); //ʹ�����ݸ����ж�(��Z��ļ��ٶ����ݼĴ������ݸ���ʱ�������������͵��жϡ������ݲɼ�����һ�����ڵĿ�ʼʱ���жϻ��Զ������)


	//ʹ��"���⶯��"��"x��"��"y��"��"z��"�������
	bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z+BMA222_SLOPE_EN_Y+BMA222_SLOPE_EN_X); //
	//bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z);

	//��������ʱ�õĲ���//bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x20);  //����"���⶯��"��б����ֵ
	bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x0A);  //����"���⶯��"��б����ֵ
	bma222_set_RegVaule(BMA222_SLOPE_DURATION,0x03);    //����"���⶯��"���˲�����
/***************************************/





//care: BMA222���ж��������ӵ�SW3(��BMA222��IO�ж���أ�ֱ����SW3������Button_IF_Init)


}



/****************************************************************************
*������-Function:		void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
*����- Description:		���ݶ�ȡ�����ͣ���ȡbma222 ���� (bma222������ٶȴ�����)  //brief Read sensor data
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
{


bma222_bus_read(bma222_hal->burst_addr,  &event_regs, sizeof(event_regs));
format_axis_data(bma222_hal, event_regs.acc, bma222_data);






/*************
	switch (type) 
	{
		case SENSOR_READ_ACCELERATION:  //��ȡ���ٶ����� (bma222������ٶȴ�����) 
			bma222_get_accel(hal, data);
			break;

//		case SENSOR_READ_TEMPERATURE: //��ȡ�ڲ��¶����� (bma222������ٶȴ�����) 
//			return bma222_get_temperature(hal, data);

//		case SENSOR_READ_ID:
//			return bma222_device_id(hal, data);//��ȡbma222 ���豸ID�� (bma222������ٶȴ�����) 

		default:
			break;
	}
********/

}

/****************************************************************************
*������-Function:		void ReadAllEventVaule_BMA222(void)
*����- Description:		��ȡBMA222���м��ٶ�����(bma222������ٶȴ�����) 
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
*****************************************************************************/
unsigned char a;

void ReadAllEventVaule_BMA222(void) //��ȡBMA222���м��ٶ�����(bma222������ٶȴ�����) 
{
a= bma222_get_RegVaule(0x09);
	bma222_bus_read(bma222_hal->burst_addr,  &event_regs, sizeof(event_regs));
//bma222_bus_read(bma222_hal->burst_addr,  &event_regs, sizeof(event_regs));

if(a  != 0)
{
a=0;
}
}



/****************************************************************************
*������-Function:		void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
*����- Description:		bma222��ʹ���ж����Ŷ�ӦIO���жϳ�ʼ��(bma222������ٶȴ�����) 
*�������-Input:	None
*�������-output:	None
*ע������-Note��	��01)    	��02)    	��03)    ��04)  
********************************************
void bma222_GPIO_Interrupt_Initial(P_INT_HANDLER bma222InterruptHdl )
{
	// Set Interrupt Type for GPIO
	//
	MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
	MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);

	g_S3InterruptHdl = S3InterruptHdl;
	g_S2InterruptHdl = S2InterruptHdl;


	// ����ʹ��RTOSʱ�������溯�����ж��ӳ̽���ע��
	GPIOIntRegister(GPIOA1_BASE, GPIOs3IntHandler);
	GPIOIntRegister(GPIOA2_BASE, GPIOs2IntHandler);

	// ��ʹ��RTOSʱ�������溯�����ж��ӳ̽���ע��
	//osi_InterruptRegister(INT_GPIOA1,(P_OSI_INTR_ENTRY)GPIOs3IntHandler, INT_PRIORITY_LVL_1);
	//osi_InterruptRegister(INT_GPIOA2,(P_OSI_INTR_ENTRY)GPIOs2IntHandler, INT_PRIORITY_LVL_1);

	// Enable Interrupt
	//
	MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_5);      //����жϱ�־λ
	MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_5);  //ʹ��ָ������Pin���ж�
	MAP_IntEnable(INT_GPIOA1); //ʹ��ָ�����Ŷ�Ӧ��GPIOxx���ж�


	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6); //����жϱ�־λ
	MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_6); //ʹ��ָ������Pin���ж�
	MAP_IntEnable(INT_GPIOA2); //ʹ��ָ�����Ŷ�Ӧ��GPIOxx���ж�
}
*********************************/


