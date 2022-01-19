/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* 文件名-FileName:			 bma222.c
* 附属文件-Dependencies:  	 bma222.h;
* 文件描述-File Description:	 ( 源程序-Source File) 
	■  "bma222三轴加速度传感器" -驱动程序(外部资源)
	        Bosch BMA222 3-axis accelerometer driver.
	01)     02)     03)    04)    05)    06)	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 注意事项-Attention : 	
	▲01)   	▲02)     ▲03)    ▲04)    ▲05)    ▲06)     
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
* 修改记录-Change History:   
	作者     时间          版本    内容描述
	Author 	  Date		   Rev          Comment
	--------------------------------------------------------
	BlueS 林2014-07-01	   1.0	   
			 xxxx-xx-xx	   x.x	   
			 xxxx-xx-xx	   x.x			
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* 公司-Company: 			CS-EMLAB  Co. , Ltd.
* 软件许可协议-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "utils.h"
#include "rom_map.h"


#include "i2c_if.h"


#include "bma222.h"    //"bma222三轴加速度传感器" -驱动程序-头文件(外部资源)


////////////////////////////////////////////////////////////////////////////
//==**全局变量定义**Global variables**========================//
////////////////////////////////////////////////////////////////////////////

sensor_hal_t   bma222_hal_s;
sensor_hal_t *bma222_hal = &bma222_hal_s;       //Sensor Hardware Abstraction Descriptor

sensor_data_t  bma222_data_s;  
sensor_data_t *bma222_data = &bma222_data_s;   //Sensor Data Descriptor


////////////////////////////////////////////////////////////////////////////
//==**局部变量定义**Local variables**========================//
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
*函数名-Function:		static void format_axis_data(const sensor_hal_t *hal, const bma_axis_t acc[], sensor_data_t *data)
*描述- Description:		转换加速度数据格式 (bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)   先把加速度数据的符号位和数值相乘，得到带符号的原始数据 
	              再根据data->scaled位，对带符号的原始数据，再进一步转换数据格式
	              当data->scaled = 1时，当数据转换为最后的实际物理值
	▲02)    	▲03)    ▲04)  
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
*函数名-Function:		void bma222_set_range(unsigned char range)
*描述- Description:		配置: 测试范围，即量程选择 (bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_set_range(unsigned char range)
{
	unsigned char array[2];

	
	//配置:量程。向BMA222的G_RANGE寄存器，写入1个字节数据。
	array[0] = BMA222_G_RANGE;
	array[1] = range;
	
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//这种情况下，最后要发送结束位(即ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_G_RANGE, &range, 1);

	//保存更改后的量程值，单位为mg
	switch (range) 
	{
		case BMA222_RANGE_2G:
			bma222_hal->range = 2000; //测量范围: +/-2g = +/-2000mg
			break;
			
		case BMA222_RANGE_4G:
			bma222_hal->range = 4000; //测量范围: +/-2g = +/-4000mg
			break;
			
		case BMA222_RANGE_8G:
			bma222_hal->range = 8000; //测量范围: +/-2g = +/-8000mg
			break;

		case BMA222_RANGE_16G:
			bma222_hal->range = 16000; //测量范围: +/-2g = +/-16000mg
			break;
			
		default:
			break;
	}

}


/****************************************************************************
*函数名-Function:		void bma222_set_bandwidth(unsigned char bandwidth)
*描述- Description:		配置: 带宽 (bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_set_bandwidth(unsigned char bandwidth)
{
	unsigned char array[2];


	array[0] = BMA222_BANDWIDTH;
	array[1] = bandwidth;

	//配置:带宽。向BMA222的BANDWIDTH寄存器，写入1个字节数据。
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//这种情况下，最后要发送结束位(即ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_BANDWIDTH, &bandwidth, 1);

	
	//保存更改后的带宽，单位为Hz
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
*函数名-Function:		void bma222_set_RegVaule(unsigned char reg, unsigned char value)
*描述- Description:		配置: 指定寄存器的值 (bma222三轴加速度传感器)
*输入参数-Input:	reg:  寄存器地址,  value:要设置的寄存器值
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_set_RegVaule(unsigned char reg, unsigned char value)
{
	unsigned char array[2];


	array[0] = reg;
	array[1] = value;

	//配置:带宽。向BMA222的BANDWIDTH寄存器，写入1个字节数据。
	I2C_IF_Write(BMA222_TWI_ADDR, array, 2, 1);//这种情况下，最后要发送结束位(即ucStop=1)
	//I2C_IF_WriteReg(BMA222_TWI_ADDR, BMA222_BANDWIDTH, &bandwidth, 1);

}









/****************************************************************************
*函数名-Function:		unsigned char bma222_get_device_id(void)
*描述- Description:		读取: 芯片的ID号(bma222三轴加速度传感器) //Read BMA222 device ID and revision numbers.
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) BMA222的ID号默认为0x03
	▲02)    	▲03)    ▲04)  
*****************************************************************************/
unsigned char bma222_get_RegVaule(unsigned char reg)
{
//	unsigned char idNum;
	unsigned char array[2];
	unsigned char cc[2];


	array[0] = reg;
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// 主机先向从机写要读取的寄存器地址。
	I2C_IF_Write(BMA222_TWI_ADDR,array,1,0);  //这种情况下，写完寄存器地址后，不要发送结束位(即ucStop=0)

	// Read the specified length of data
	//读出指定寄存器的值
	I2C_IF_Read(BMA222_TWI_ADDR,cc, 1);

	return cc[0];
}


/****************************************************************************
*函数名-Function:		unsigned char bma222_get_device_id(void)
*描述- Description:		读取: 芯片的ID号(bma222三轴加速度传感器) //Read BMA222 device ID and revision numbers.
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) BMA222的ID号默认为0x03
	▲02)    	▲03)    ▲04)  
*****************************************************************************/
unsigned char bma222_get_device_id(void)
{
//	unsigned char idNum;
	unsigned char array[2];
	unsigned char cc[2];

	array[0] = BMA222_CHIP_ID;
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// 主机先向从机写要读取的寄存器地址。
	I2C_IF_Write(BMA222_TWI_ADDR,array,1,0);  //这种情况下，写完寄存器地址后，不要发送结束位(即ucStop=0)

	// Read the specified length of data
	//读出指定寄存器的值
	I2C_IF_Read(BMA222_TWI_ADDR,cc, 1);

	return cc[0];
}

/****************************************************************************
*函数名-Function:		void bma222_bus_read(unsigned char addr, void *data, unsigned char count)
*描述- Description:		通过IIC从指定地址，连续读取多个数据(bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_bus_read(unsigned char addr, void *data, unsigned char count)
{
	// Write the register address to be read from.  Stop bit implicitly assumed to be 0.
	// 主机先向从机写要读取的寄存器地址。
	I2C_IF_Write(BMA222_TWI_ADDR,&addr,1,0);  //这种情况下，写完寄存器地址后，不要发送结束位(即ucStop=0)

	// Read the specified length of data
	//读出指定寄存器的值
	I2C_IF_Read(BMA222_TWI_ADDR, data, count);
}



/****************************************************************************
*函数名-Function:		static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data)
*描述- Description:		从传感器读取:加速度数据(bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data)
{

	hal->burst_addr = BMA222_NEW_DATA_X;  //连续读取数据的起始地址(寄存器)

	bma222_bus_read(hal->burst_addr, event_regs.acc, sizeof(event_regs.acc));


	format_axis_data(hal, event_regs.acc, data);
}








/****************************************************************************
*函数名-Function:	void bma222_init(void)
*描述- Description:		初始化bma222 (bma222三轴加速度传感器)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_init(void)
{

//==BMA222进行软件复位========================//
	
	//the default range and bandwidth after device reset are +/- 2g and 1kHz respectively.(bma222)

	//bma222_set_state(sensor, SENSOR_STATE_RESET);

	bma222_hal->range      = 2000;   //测量范围: +/-2g= +/-2000mg (复位后的默认值)
	bma222_hal->bandwidth  = 1000; //带宽: 1000 Kz   (复位后的默认值) // Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	bma222_hal->resolution = BMA222_DATA_RESOLUTION;  //加速度数据位数: 8位
	bma222_hal->burst_addr = BMA222_NEW_DATA_X;  //连续读取数据的起始地址(寄存器)

	bma222_hal->orientation.x.axis =  SENSOR_AXIS_X;
	bma222_hal->orientation.x.sign=  SENSOR_SIGN_POS;

	bma222_hal->orientation.y.axis =  SENSOR_AXIS_Y;
	bma222_hal->orientation.y.sign=  SENSOR_SIGN_POS;

	bma222_hal->orientation.z.axis =  SENSOR_AXIS_Z;
	bma222_hal->orientation.z.sign=  SENSOR_SIGN_POS;




//==BMA222复位后，重新配置下面参数========================//
	
	bma222_set_RegVaule(BMA222_SOFTRESET,BMA222_RESET);    //软件复位bma222 (bma222三轴加速度传感器) 
	MAP_UtilsDelay(800000); //约50ms//延时子程序(短暂延时)
	bma222_set_RegVaule(BMA222_SOFTRESET,BMA222_RESET);    //软件复位bma222 (bma222三轴加速度传感器) 
	MAP_UtilsDelay(800000); //约50ms//延时子程序(短暂延时)


	//如果PS引脚为低电平或高电平时，故BMA222选择的工作模式为通用模式。
       //PS引脚为高电平,故BMA222选择I2C作为通信接口
       
	//复位后，寄存器(0x13)的寄存器位data_high_bw=0//采用经过滤波的数据流
	//复位后，寄存器(0x19~0x1B)都为0//中断引脚INT1和INT2的中断功能映射: 都被禁用
	//复位后，寄存器(0x20)=0x05
		//寄存器位int2_lvl=1,int1_lvl=1//有中断时，INT1 和INT2 输出高电平
		//寄存器位int2_od=0,int1_od=0//INT1 和INT2引脚模式:推挽输出(高低电平由IC的电源决定)
		
	//复位后，寄存器(0x27)=0x00,slope_dur=0//  故，消抖用的特定的值N=0+1=1
	//复位后，寄存器(0x28)=slope_th=0x14//对于测量范围为±2g时，slope_th每增加1，相当于g值增加15.6mg
		//故slope_th=0x14，对应阈值15.6mg*0x14=312mg(加速度值变化超过312mg时，产生中断)

	//复位后，寄存器(0x17)=0x00、(0x16)=0x00//所有检测被禁用
		//寄存器(0x17): reserved  reserved  reserved   data_en  low_en   high_en_z  high_en_y  high_en_x
		//寄存器(0x16): flat_en   orient_en    s_tap_en   d_tap_en  reserved   slope_en_z   slope_en_y   slope_en_x
		//data_en=0//禁用数据更新中断(当Z轴的加速度数据寄存器数据更新时，将产生此类型的中断。在数据采集的下一个周期的开始时，中断会自动清除。)
	//复位后，寄存器(0x13)的寄存器位data_high_bw=0//
	//复位后，寄存器(0x13)的寄存器位data_high_bw=0//
	//复位后，寄存器(0x13)的寄存器位data_high_bw=0//




	//bma222_set_range(BMA222_RANGE_2G); //测量范围: +/-2g= +/-2000mg

	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	//bma222_set_bandwidth(BMA222_BANDWIDTH_1000Hz);  //带宽: 1000 Hz


//==========================//
	bma222_set_range(BMA222_RANGE_2G); //测量范围: +/-2g= +/-2000mg



/**1号和2号检测移动，不检测冲击**************************************************
	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	//bma222_set_bandwidth(BMA222_BANDWIDTH_16Hz);  //带宽: 16 Hz
bma222_set_bandwidth(BMA222_BANDWIDTH_8Hz);  //带宽: 8 Hz
	//bma222_set_RegVaule(BMA222_17_INTR_EN, BMA222_DATA_EN); //使能数据更新中断(当Z轴的加速度数据寄存器数据更新时，将产生此类型的中断。在数据采集的下一个周期的开始时，中断会自动清除。)


	//使能"任意动作"的"x轴"、"y轴"、"z轴"动作检测
	bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z+BMA222_SLOPE_EN_Y+BMA222_SLOPE_EN_X); //
	//bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z);
	
	//华东决赛时用的参数//bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x0B);  //设置"任意动作"的斜率阈值
	bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x06);  //设置"任意动作"的斜率阈值
	bma222_set_RegVaule(BMA222_SLOPE_DURATION,0x03);    //设置"任意动作"的滤波次数
**************************************************/
        
        

/**3号检测冲击，不检测移动***********************/
	// Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
	bma222_set_bandwidth(BMA222_BANDWIDTH_1000Hz);  //带宽: 1000 Hz

	//bma222_set_RegVaule(BMA222_17_INTR_EN, BMA222_DATA_EN); //使能数据更新中断(当Z轴的加速度数据寄存器数据更新时，将产生此类型的中断。在数据采集的下一个周期的开始时，中断会自动清除。)


	//使能"任意动作"的"x轴"、"y轴"、"z轴"动作检测
	bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z+BMA222_SLOPE_EN_Y+BMA222_SLOPE_EN_X); //
	//bma222_set_RegVaule(BMA222_16_INTR_EN, BMA222_SLOPE_EN_Z);

	//华东决赛时用的参数//bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x20);  //设置"任意动作"的斜率阈值
	bma222_set_RegVaule(BMA222_SLOPE_THRESHOLD,0x0A);  //设置"任意动作"的斜率阈值
	bma222_set_RegVaule(BMA222_SLOPE_DURATION,0x03);    //设置"任意动作"的滤波次数
/***************************************/





//care: BMA222的中断引脚连接到SW3(故BMA222的IO中断相关，直接在SW3中设置Button_IF_Init)


}



/****************************************************************************
*函数名-Function:		void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
*描述- Description:		根据读取的类型，读取bma222 数据 (bma222三轴加速度传感器)  //brief Read sensor data
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
{


bma222_bus_read(bma222_hal->burst_addr,  &event_regs, sizeof(event_regs));
format_axis_data(bma222_hal, event_regs.acc, bma222_data);






/*************
	switch (type) 
	{
		case SENSOR_READ_ACCELERATION:  //读取加速度数据 (bma222三轴加速度传感器) 
			bma222_get_accel(hal, data);
			break;

//		case SENSOR_READ_TEMPERATURE: //读取内部温度数据 (bma222三轴加速度传感器) 
//			return bma222_get_temperature(hal, data);

//		case SENSOR_READ_ID:
//			return bma222_device_id(hal, data);//读取bma222 的设备ID号 (bma222三轴加速度传感器) 

		default:
			break;
	}
********/

}

/****************************************************************************
*函数名-Function:		void ReadAllEventVaule_BMA222(void)
*描述- Description:		读取BMA222所有加速度数据(bma222三轴加速度传感器) 
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
unsigned char a;

void ReadAllEventVaule_BMA222(void) //读取BMA222所有加速度数据(bma222三轴加速度传感器) 
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
*函数名-Function:		void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data)
*描述- Description:		bma222所使用中断引脚对应IO口中断初始化(bma222三轴加速度传感器) 
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
********************************************
void bma222_GPIO_Interrupt_Initial(P_INT_HANDLER bma222InterruptHdl )
{
	// Set Interrupt Type for GPIO
	//
	MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
	MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);

	g_S3InterruptHdl = S3InterruptHdl;
	g_S2InterruptHdl = S2InterruptHdl;


	// 当不使用RTOS时，用下面函数对中断子程进行注册
	GPIOIntRegister(GPIOA1_BASE, GPIOs3IntHandler);
	GPIOIntRegister(GPIOA2_BASE, GPIOs2IntHandler);

	// 当使用RTOS时，用下面函数对中断子程进行注册
	//osi_InterruptRegister(INT_GPIOA1,(P_OSI_INTR_ENTRY)GPIOs3IntHandler, INT_PRIORITY_LVL_1);
	//osi_InterruptRegister(INT_GPIOA2,(P_OSI_INTR_ENTRY)GPIOs2IntHandler, INT_PRIORITY_LVL_1);

	// Enable Interrupt
	//
	MAP_GPIOIntClear(GPIOA1_BASE,GPIO_PIN_5);      //清除中断标志位
	MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_5);  //使能指定引脚Pin的中断
	MAP_IntEnable(INT_GPIOA1); //使能指定引脚对应的GPIOxx的中断


	MAP_GPIOIntClear(GPIOA2_BASE,GPIO_PIN_6); //清除中断标志位
	MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_6); //使能指定引脚Pin的中断
	MAP_IntEnable(INT_GPIOA2); //使能指定引脚对应的GPIOxx的中断
}
*********************************/


