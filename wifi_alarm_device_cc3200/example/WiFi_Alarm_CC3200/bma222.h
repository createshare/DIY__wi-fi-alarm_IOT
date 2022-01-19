/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* 文件名-FileName:			 bma222.h
* 附属文件-Dependencies:  	 None	
* 文件描述-File Description:	 ( 头文件-Header File )
	■  "bma222三轴加速度传感器" -驱动程序-头文件(外部资源)
	        Bosch BMA222 3-axis accelerometer driver.
	01)     02)     03)    04)    05)    06)	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 注意事项-Attention : 	
	▲01)     ▲02)     ▲03)    ▲04)    ▲05)    ▲06)     
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

#ifndef _BMA222_H_
#define _BMA222_H_


////////////////////////////////////////////////////////////////////////////
//==**"BlueS Add"宏定义**================================//
////////////////////////////////////////////////////////////////////////////
#include <stdint.h>      //j头文件内:定义了"uint8_t"类型
#include<stdbool.h>    //j头文件内:定义了"bool"类型
#include<stddef.h>     //j头文件内:定义了"size_t"类型

////////////////////////////////////////////////////////////////////////////
//==**"此模块专用"宏定义**Module-specific macro**==============//
////////////////////////////////////////////////////////////////////////////

/* TWI/I2C address (write @ 0x16 on bus, read @ 0x17 on bus) */
#define BMA222_TWI_ADDR         (0x18)   //因为芯片"引脚1"接地，故设备IIC的地址为0x08
//#define BMA222_SPI_MODE         (3)

/* Sensor Data Resolution and Offsets */
#define BMA222_DATA_RESOLUTION  (8)     /* signed axis data size (bits) */
#define BMA222_TEMP_OFFSET      (24)    /* temperature center (Celsius) */


/*
 * Standard Register Addresses (TWI & SPI)
 *
 * w/r=write/read, wo=write only, ro=read only
 */
typedef enum {
	BMA222_CHIP_ID = 0,     /* 00 (ro) chip ID (always 0x03) */
	BMA222_01_RSVD,         /* 01 reserved */
	BMA222_NEW_DATA_X,      /* 02 (ro) X-axis new data flag */
	BMA222_ACC_X,           /* 03 (ro) X-axis acceleration */
	BMA222_NEW_DATA_Y,      /* 04 (ro) Y-axis new data flag */
	BMA222_ACC_Y,           /* 05 (ro) Y-axis acceleration */
	BMA222_NEW_DATA_Z,      /* 06 (ro) Z-axis new data flag */
	BMA222_ACC_Z,           /* 07 (ro) Z-axis acceleration */
	BMA222_TEMP,            /* 08 (ro) temperature */
	BMA222_INTR_STATUS,     /* 09 (ro) misc. interrupt status */
	BMA222_NEW_DATA_STATUS, /* 0a (ro) new data interrupt status */
	BMA222_TAP_SLOPE_STATUS, /* 0b (ro) tap and slope interrupt status */
	BMA222_ORIENTATION_STATUS, /* 0c (ro) flat and orientation status */
	BMA222_0D_RSVD,         /* 0d reserved */
	BMA222_0E_RSVD,         /* 0e reserved */
	BMA222_G_RANGE,         /* 0f (w/r) G-range selection */
	BMA222_BANDWIDTH,       /* 10 (w/r) bandwidth selection */
	BMA222_POWER_MODES,     /* 11 (w/r) power mode configuration */
	BMA222_12_RSVD,         /* 12 reserved */
	BMA222_DATA_HIGH_BW,    /* 13 (w/r) acceleration data filter */
	BMA222_SOFTRESET,       /* 14 (wo) user-triggered software reset */
	BMA222_15_RSVD,         /* 15 reserved */
	BMA222_16_INTR_EN,      /* 16 (w/r) interrupt enable bits */
	BMA222_17_INTR_EN,      /* 17 (w/r) interrupt enable bits */
	BMA222_18_RSVD,         /* 18 reserved */
	BMA222_19_INTR_MAP,     /* 19 (w/r) interrupt pin mapping */
	BMA222_1A_INTR_MAP,     /* 1a (w/r) interrupt pin mapping */
	BMA222_1B_INTR_MAP,     /* 1b (w/r) interrupt pin mapping */
	BMA222_1C_RSVD,         /* 1c reserved */
	BMA222_1D_RSVD,         /* 1d reserved */
	BMA222_INTR_DATA_SRC,   /* 1e (w/r) filtered/unfiltered data */
	BMA222_1F_RSVD,         /* 1f reserved */
	BMA222_INTR_PIN_CONFIG, /* 20 (w/r) interrupt pin configuration */
	BMA222_INTR_PIN_MODE,   /* 21 (w/r) interrupt pin mode & reset */
	BMA222_LOW_G_DURATION,  /* 22 (w/r) low-g interrupt delay time */
	BMA222_LOW_G_THRESHOLD, /* 23 (w/r) low-g interrupt threshold */
	BMA222_EVENT_HYSTERESIS, /* 24 (w/r) low-/high-g event hysteresis */
	BMA222_HIGH_G_DURATION, /* 25 (w/r) high-g interrupt delay time */
	BMA222_HIGH_G_THRESHOLD, /* 26 (w/r) high-g interrupt threshold */
	BMA222_SLOPE_DURATION,  /* 27 (w/r) no. samples for slope event */
	BMA222_SLOPE_THRESHOLD, /* 28 (w/r) slope event threshold */
	BMA222_29_RSVD,         /* 29 reserved */
	BMA222_TAP_TIMING,      /* 2a (w/r) single/double tap event timing */
	BMA222_TAP_CONFIG,      /* 2b (w/r) wake samples & thresholds */
	BMA222_ORIENTATION_CONFIG, /* 2c (w/r) hysteresis, blocking, & mode */
	BMA222_ORIENTATION_THETA, /* 2d (w/r) theta blocking angle */
	BMA222_FLAT_THETA,      /* 2e (w/r) flat threshold angle */
	BMA222_FLAT_HOLD_TIME,  /* 2f (w/r) flat hold time */
	BMA222_30_RSVD,         /* 30 reserved */
	BMA222_31_RSVD,         /* 31 reserved */
	BMA222_SENSOR_SELF_TEST, /* 32 (w/r) self-test settings/activation */
	BMA222_EEPROM_CONTROL,  /* 33 (w/r) non-volatile memory control */
	BMA222_DIGITAL_IO_CONTROL, /* 34 (w/r) I2C & SPI interface settings */
	BMA222_35_RSVD,         /* 35 reserved */
	BMA222_FAST_OFFSET_COMP, /* 36 (w/r) fast offset compensation settings */
	BMA222_SLOW_OFFSET_COMP, /* 37 (w/r) slow offset compensation settings */
	BMA222_OFFSET_FILT_X,   /* 38 (w/r) filtered data compensation x-axis */
	BMA222_OFFSET_FILT_Y,   /* 39 (w/r) filtered data compensation y-axis */
	BMA222_OFFSET_FILT_Z,   /* 3a (w/r) filtered data compensation z-axis */
	BMA222_OFFSET_UNFILT_X, /* 3b (w/r) unfiltered data compensation x-axis */
	BMA222_OFFSET_UNFILT_Y, /* 3c (w/r) unfiltered data compensation y-axis */
	BMA222_OFFSET_UNFILT_Z  /* 3d (w/r) unfiltered data compensation z-axis */
} bma222_register_t;

/** \brief BMA222 Register Bit Definitions */
/** @{ */

/* BMA222_CHIP_ID (0x00) */

#define BMA222_ID_VAL           (0x03)

/* BMA222_INTR_STATUS (0x09) */

#define BMA222_FLAT_INT         (1 << 7)    /* flat interrupt status */
#define BMA222_ORIENT_INT       (1 << 6)    /* orientation interrupt status */
#define BMA222_S_TAP_INT        (1 << 5)    /* single tap interrupt status */
#define BMA222_D_TAP_INT        (1 << 4)    /* double tap interrupt status */
#define BMA222_SLOPE_INT        (1 << 2)    /* slope interrupt status */
#define BMA222_HIGH_INT         (1 << 1)    /* high-g interrupt status */
#define BMA222_LOW_INT          (1 << 0)    /* low-g interrupt status */

/* BMA222_NEW_DATA_STATUS (0x0a) */

#define BMA222_DATA_INT         (1 << 7)    /* new data interrupt status */

/* BMA222_TAP_SLOPE_STATUS (0x0b) */

#define BMA222_TAP_SIGN_POS     (1 << 7)    /* tap interrupt sign (0=negative) */
#define BMA222_TAP_FIRST_Z      (1 << 6)    /* z-axis triggered tap interrupt */
#define BMA222_TAP_FIRST_Y      (1 << 5)    /* y-axis triggered tap interrupt */
#define BMA222_TAP_FIRST_X      (1 << 4)    /* x-axis triggered tap interrupt */
#define BMA222_SLOPE_SIGN_POS   (1 << 3)    /* slope interrupt sign (0=negative) */
#define BMA222_SLOPE_FIRST_Z    (1 << 2)    /* z-axis triggered slope interrupt */
#define BMA222_SLOPE_FIRST_Y    (1 << 1)    /* y-axis triggered slope interrupt */
#define BMA222_SLOPE_FIRST_X    (1 << 0)    /* x-axis triggered slope interrupt */

/* BMA222_ORIENTATION_STATUS (0x0c) */

#define BMA222_FLAT             (1 << 7)    /* flat condition is fulfilled */
#define BMA222_Z_DOWN           (1 << 6)    /* z-axis orientation (0=upward) */
#define BMA222_XY_PORTRAIT_UP   (0 << 4)    /* x-y plane portrait upright */
#define BMA222_XY_PORTRAIT_DOWN (1 << 4)    /* x-y plane portrait upside-down */
#define BMA222_XY_LANDSCAPE_L   (2 << 4)    /* x-y plane landscape left */
#define BMA222_XY_LANDSCAPE_R   (3 << 4)    /* x-y plane landscape right */
#define BMA222_HIGH_SIGN_NEG    (1 << 3)    /* high-g interrupt sign
	                                     * (0=positive) */
#define BMA222_HIGH_FIRST_Z     (1 << 2)    /* z-axis triggered high-g interrupt */
#define BMA222_HIGH_FIRST_Y     (1 << 1)    /* y-axis triggered high-g interrupt */
#define BMA222_HIGH_FIRST_X     (1 << 0)    /* x-axis triggered high-g interrupt */

/* BMA222_G_RANGE (0x0f) */

#define BMA222_RANGE_2G         (0x03)      /* +/- 2g range (default) */
#define BMA222_RANGE_4G         (0x05)      /* +/- 4g range */
#define BMA222_RANGE_8G         (0x08)      /* +/- 8g range */
#define BMA222_RANGE_16G        (0x0c)      /* +/- 16g range */



/* BMA222_BANDWIDTH (0x10) */

#define BMA222_BANDWIDTH_8Hz    (0x08)      /*   7.81 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_16Hz   (0x09)      /*  15.63 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_31Hz   (0x0a)      /*  31.25 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_63Hz   (0x0b)      /*  62.5 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_125Hz  (0x0c)      /* 125 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_250Hz  (0x0d)      /* 250 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_500Hz  (0x0e)      /* 500 Hz filtered data bandwidth */
#define BMA222_BANDWIDTH_1000Hz (0x1f)      /* 1000 Hz filtered data bandwidth */

/* BMA222_POWER_MODES (0x11) */

#define BMA222_SUSPEND          (1  << 7)   /* set suspend mode (0=reset mode) */
#define BMA222_LOWPOWER_EN      (1  << 6)   /* set low-power mode (0=reset mode) */
#define BMA222_SLEEP_DUR_0_5ms  (5  << 1)   /* 0.5 ms sleep phase duration */
#define BMA222_SLEEP_DUR_1ms    (6  << 1)   /*   1 ms sleep phase duration */
#define BMA222_SLEEP_DUR_2ms    (7  << 1)   /*   2 ms sleep phase duration */
#define BMA222_SLEEP_DUR_4ms    (8  << 1)   /*   4 ms sleep phase duration */
#define BMA222_SLEEP_DUR_6ms    (9  << 1)   /*   6 ms sleep phase duration */
#define BMA222_SLEEP_DUR_10ms   (10 << 1)   /*   6 ms sleep phase duration */
#define BMA222_SLEEP_DUR_25ms   (11 << 1)   /*  25 ms sleep phase duration */
#define BMA222_SLEEP_DUR_50ms   (12 << 1)   /*  50 ms sleep phase duration */
#define BMA222_SLEEP_DUR_100ms  (13 << 1)   /* 100 ms sleep phase duration */
#define BMA222_SLEEP_DUR_500ms  (14 << 1)   /* 500 ms sleep phase duration */
#define BMA222_SLEEP_DUR_1000ms (15 << 1)   /*   1 s sleep phase duration */

/* BMA222_DATA_HIGH_BW (0x13) */

#define BMA222_DATA_UNFILTERED  (1 << 7)    /* unfiltered data (0=filtered) */

/* BMA222_SOFTRESET (0x14) */

#define BMA222_RESET            (0xb6)      /* user-triggered reset write value */


/* BMA222_16_INTR_EN (0x16) */

#define BMA222_FLAT_EN          (1 << 7)    /* flat interrupt enable */
#define BMA222_ORIENT_EN        (1 << 6)    /* orientation interrupt enable */
#define BMA222_S_TAP_EN         (1 << 5)    /* single tap interrupt enable */
#define BMA222_D_TAP_EN         (1 << 4)    /* double tap interrupt enable */
#define BMA222_SLOPE_EN_Z       (1 << 2)    /* z-axis slope interrupt enable */
#define BMA222_SLOPE_EN_Y       (1 << 1)    /* y-axis slope interrupt enable */
#define BMA222_SLOPE_EN_X       (1 << 0)    /* x-axis slope interrupt enable */

/* BMA222_17_INTR_EN (0x17) */

#define BMA222_DATA_EN          (1 << 4)    /* new data interrupt enable */
#define BMA222_LOW_EN           (1 << 3)    /* low-g interrupt enable */
#define BMA222_HIGH_EN_Z        (1 << 2)    /* z-axis high-g interrupt enable */
#define BMA222_HIGH_EN_Y        (1 << 1)    /* y-axis high-g interrupt enable */
#define BMA222_HIGH_EN_X        (1 << 0)    /* x-axis high-g interrupt enable */


/* BMA222_19_INTR_MAP (0x19) */

#define BMA222_INT1_FLAT        (1 << 7)    /* map flat interrupt to INT1 */
#define BMA222_INT1_ORIENT      (1 << 6)    /* map orientation interrupt to INT1 */
#define BMA222_INT1_S_TAP       (1 << 5)    /* map single tap interrupt to INT1 */
#define BMA222_INT1_D_TAP       (1 << 4)    /* map double tap interrupt to INT1 */
#define BMA222_INT1_SLOPE       (1 << 2)    /* map slope interrupt to INT1 */
#define BMA222_INT1_HIGH        (1 << 1)    /* map high-g interrupt to INT1 */
#define BMA222_INT1_LOW         (1 << 0)    /* map low-g interrupt to INT1 */

/* BMA222_1A_INTR_MAP (0x1a) */

#define BMA222_INT2_DATA        (1 << 7)    /* map new data interrupt to INT2 */
#define BMA222_INT1_DATA        (1 << 0)    /* map new data interrupt to INT1 */

/* BMA222_1B_INTR_MAP (0x1b) */

#define BMA222_INT2_FLAT        (1 << 7)    /* map flat interrupt to INT2 */
#define BMA222_INT2_ORIENT      (1 << 6)    /* map orientation interrupt to INT2 */
#define BMA222_INT2_S_TAP       (1 << 5)    /* map single tap interrupt to INT2 */
#define BMA222_INT2_D_TAP       (1 << 4)    /* map double tap interrupt to INT2 */
#define BMA222_INT2_SLOPE       (1 << 2)    /* map slope interrupt to INT2 */
#define BMA222_INT2_HIGH        (1 << 1)    /* map high-g interrupt to INT2 */
#define BMA222_INT2_LOW         (1 << 0)    /* map low-g interrupt to INT2 */

/* BMA222_INTR_DATA_SRC (0x1e) */

#define BMA222_INT_SRC_DATA     (1 << 5)    /* unfiltered new data interrupt */
#define BMA222_INT_SRC_TAP      (1 << 4)    /* unfiltered s/d tap interrupt data */
#define BMA222_INT_SRC_SLOPE    (1 << 2)    /* unfiltered slope interrupt data */
#define BMA222_INT_SRC_HIGH     (1 << 1)    /* unfiltered high-g interrupt data */
#define BMA222_INT_SRC_LOW      (1 << 0)    /* unfiltered low-g interrupt data */

/* BMA222_INTR_PIN_CONFIG (0x20) */

#define BMA222_INT2_OD          (1 << 3)    /* open drive for INT2 pin */
#define BMA222_INT2_LVL_1       (1 << 2)    /* active level 1 for INT2 (default) */
#define BMA222_INT1_OD          (1 << 1)    /* open drive for INT1 pin */
#define BMA222_INT1_LVL_1       (1 << 0)    /* active level 1 for INT1 (default) */

/* BMA222_INTR_PIN_MODE (0x21) */

#define BMA222_RESET_INT        (0x80)      /* reset any latch interrupt */
#define BMA222_INT_NON_LATCHED  (0x00)      /* non-latch interrupt (default) */
#define BMA222_INT_TMP_250ms    (0x01)      /* 250ms temporary latch interrupt */
#define BMA222_INT_TMP_500ms    (0x02)      /* 500ms temporary latch interrupt */
#define BMA222_INT_TMP_1sec     (0x03)      /* 1000ms temporary latch interrupt */
#define BMA222_INT_TMP_2sec     (0x04)      /* 2000ms temporary latch interrupt */
#define BMA222_INT_TMP_4sec     (0x05)      /* 4000ms temporary latch interrupt */
#define BMA222_INT_TMP_8sec     (0x06)      /* 8000ms temporary latch interrupt */
#define BMA222_INT_TMP_500us    (0x0a)      /* 500us temporary latch interrupt */
#define BMA222_INT_TMP_1ms      (0x0b)      /* 1ms temporary latch interrupt */
#define BMA222_INT_TMP_12_5ms   (0x0c)      /* 12.5ms temporary latch interrupt */
#define BMA222_INT_TMP_25ms     (0x0d)      /* 25ms temporary latch interrupt */
#define BMA222_INT_TMP_50ms     (0x0e)      /* 50ms temporary latch interrupt */
#define BMA222_INT_LATCHED      (0x0f)      /* latch interrupt mode */


/**
 * BMA222_LOW_G_DURATION (0x22)
 *
 * Low-G interrupt delay time constants where the physical delay time is
 * computed as: delay[ms] = [low_dur + 1] * 2ms. The default value is 0x09
 * corresponding to a delay of 20ms.
 */

/**
 * BMA222_LOW_G_THRESHOLD (0x23)
 *
 * The log-g interrupt threshold value LSB corresponds to an acceleration
 * of 7.81mg with range 0 to 1.992g. The default value is 0x30 corresponding
 * to 375mg.
 */


/* BMA222_TAP_CONFIG (0x2b) */

#define BMA222_TAP_TH_FIELD     (0x1f)      /* tap interrupt threshold field */
#define BMA222_TAP_SAMP_FIELD   (0xc0)      /* tap wake-up samples count field */

/* BMA222_SENSOR_SELF_TEST (0x32) */

#define BMA222_SELF_TEST_NONE   (0x00)      /* no self-test (default) */
#define BMA222_SELF_TEST_AXIS_X (0x01)      /* self-test positive x-axis */
#define BMA222_SELF_TEST_AXIS_Y (0x02)      /* self-test positive y-axis */
#define BMA222_SELF_TEST_AXIS_Z (0x03)      /* self-test positive z-axis */

/** @} */

/**
 * convert real G values into register values
 *  \note no range checking is included
 */
#define threshold_in_g(threshold, range)    (((threshold) * 16) / (range))

/**
 * convert real G values into hysteresis register values
 *  \note no range checking is included
 */
#define hysteresis_in_g(hysteresis, range)  (((threshold) * 4) / (range))

/* Function Prototypes */


////////////////////////////////////////////////////////////////////////////
//==**"bma_axis.inc.h"宏定义**================================//
////////////////////////////////////////////////////////////////////////////


/**
 * @brief Bosch 8-bit Axis Data Format
 *
 * The BMA222 stores 2's-complement 8-bit axis data samples split across
 * two contiguous 8-bit device locations where the lower address in the
 * device register space stores a 1-bit "new data" flag and the next
 * higher address stores the 8 most significant bits of the axis data.
 */
typedef union {
	int16_t word;

	struct {
		uint16_t new_data : 1;
		uint16_t unused   : 7;
		
		int16_t acc_msb   : 8;
	} field;
} bma_axis8_t;


/**
 * @brief Construct a signed value from a raw axis sample.
 *
 * The BMA222 returns little-endian 2's-complement 8-bit axis values
 * stored within a 16-bit word.
 *
 * @param   axis    An bma_axis8_t type storing a raw sensor axis sample.
 * @return          The aligned and sign-extended axis data value.
 */
static inline int16_t bma_axis8_val(const bma_axis8_t axis)
{
	return axis.field.acc_msb;

	//return (axis.word >> 8);
}

#if defined(_BMA222_H_)
#   define bma_axis_t       bma_axis8_t
#   define bma_axis_val     bma_axis8_val
#endif


////////////////////////////////////////////////////////////////////////////
//==**项目"专用"宏定义**Project-specific macro**================//
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//==**全局变量定义**Global variables**========================//
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//==**sensors.h**========================//

/** \name Sensor Physical Axis Definitions */
/** @{ */

/** \brief Sensor Axis Name Constants */
typedef enum {
	SENSOR_AXIS_X               =  0,      /* Valid indices are 0, 1, 2 */
	SENSOR_AXIS_Y               =  1,
	SENSOR_AXIS_Z               =  2,
	SENSOR_AXIS_NONE            = -1
} sensor_axis_t;

/** \brief Sensor Axis Sign Constants */
typedef enum {
	SENSOR_SIGN_POS             =  1,
	SENSOR_SIGN_NEG             = -1,
	SENSOR_SIGN_NONE            =  0
} sensor_sign_t;

/** \brief Sensor Axis Descriptor */
typedef struct {
	sensor_axis_t axis;
	sensor_sign_t sign;
} sensor_axis_vec_t;

/** \brief Sensor Orientation Descriptor */
typedef struct {
	sensor_axis_vec_t x;
	sensor_axis_vec_t y;
	sensor_axis_vec_t z;
} sensor_orient_t;


/** \brief Sensor Platform Hardware Abstraction Descriptor */
struct sensor_hal {
//	bus_desc_t bus;                  /**< Platform Bus Descriptor */
	uint8_t burst_addr;              /**< Sensor Burst Read Address */
	uint32_t mcu_sigint;             /**< I/O input to MCU from sensor */
	uint32_t mcu_sigout;             /**< I/O output from MCU to sensor */
//	sensor_type_t dev_type;          /**< Sensor Device Type */



	/* Sensor Physical/Logical Orientation */
	sensor_orient_t orientation;     /**< Sensor axis/sign used as X,Y,Z */

	/* Sensor Operational Parameters */
	int16_t range;                   /**< Sensor range (engineering units) */
	int16_t bandwidth;               /**< Sensor bandwidth (Hz) */
	int16_t sample_rate;             /**< Sensor sample rate (Hz) */
	int16_t resolution;              /**< Sensor sample resolution (bits) */

};

/** \brief Sensor Hardware Abstraction Descriptor */
typedef struct sensor_hal sensor_hal_t;


#define AXIS_X_POS  {SENSOR_AXIS_X, SENSOR_SIGN_POS}
#define AXIS_X_NEG  {SENSOR_AXIS_X, SENSOR_SIGN_NEG}
#define AXIS_Y_POS  {SENSOR_AXIS_Y, SENSOR_SIGN_POS}
#define AXIS_Y_NEG  {SENSOR_AXIS_Y, SENSOR_SIGN_NEG}
#define AXIS_Z_POS  {SENSOR_AXIS_Z, SENSOR_SIGN_POS}
#define AXIS_Z_NEG  {SENSOR_AXIS_Z, SENSOR_SIGN_NEG}
#define AXIS_NONE   {SENSOR_AXIS_NONE, SENSOR_SIGN_NONE}
/** @} */




/** \brief Sensor Data Descriptor */
typedef struct {
	union {
		int32_t value [3];          /**< Generic data name */

		struct {                    /* Linear axis data */
			int32_t x;              /**< x-axis value */
			int32_t y;              /**< y-axis value */
			int32_t z;              /**< z-axis value */
		} axis;

		struct {                    /* Rotational axis data */
			int32_t pitch;          /**< Pitch-axis value */
			int32_t roll;           /**< Roll-axis value */
			int32_t yaw;            /**< Yaw-axis value */
		} angle;

		struct {                    /* Magnetic heading data */
			int32_t direction;      /**< Heading relative to magnetic north */
			int32_t inclination;    /**< Inclination above/below horizontal */
			int32_t strength;       /**< Net magnetic field intensity */
		} heading;

		struct {                    /* Tap detection data */
			int32_t count;          /**< Tap count */
			int32_t axis;           /**< Tap detection axis (X,Y,Z) */
			int32_t direction;      /**< Tap direction */
		} tap;

		struct {                    /* Pressure data */
			int32_t value;          /**< Pressure value */
			int32_t unused[2];      /**< Unused (spare) fields */
		} pressure;

		struct {                    /* Temperature data */
			int32_t value;          /**< Temperature value */
			int32_t unused[2];      /**< Unused (spare) fields */
		} temperature;

		struct {                    /* Light data */
			int32_t value;          /**< Light value */
			int32_t unused[2];      /**< Unused (spare) fields */
		} light;

		struct {                    /* Proximity data */
			int32_t value[3];       /**< Proximity values (3) */
		} proximity;

		struct {
			uint32_t id;            /**< Device ID */
			uint32_t version;       /**< Version */
			uint32_t unused;        /**< Unused (spare) field */
		} device;
	};

	uint32_t timestamp;             /**< Data sample timestamp */
	bool scaled;                    /**< Data sample format (true => scaled) */
} sensor_data_t;


/** \brief Sensor Data Read Operations */
typedef enum {
	SENSOR_READ_ACCELERATION,       /**< Read acceleration data */
	SENSOR_READ_FIELD,              /**< Read field strength data */
	SENSOR_READ_HEADING,            /**< Read heading data */
	SENSOR_READ_ID,                 /**< Read sensor device ID */
	SENSOR_READ_INTENSITY,          /**< Read intensity data */
	SENSOR_READ_LIGHT,              /**< Read light level data */
	SENSOR_READ_PRESSURE,           /**< Read pressure data */
	SENSOR_READ_PROXIMITY,          /**< Read proximity data */
	SENSOR_READ_ROTATION,           /**< Read rotation data */
	SENSOR_READ_TEMPERATURE,        /**< Read temperature data */
} sensor_read_t;




/** \brief Convert raw sensor data to scaled engineering units.
 *
 * This routine converts a sensor sample, \c counts, to a linearly scaled
 * integer value in engineering units using device-specific range in
 * standard engineering units and full scale resolution parameters in
 * \c hal.
 *
 * \param hal       An initialized hardware interface descriptor.
 * \param counts    An unscaled raw sensor sample value.
 *
 * \return Scaled signed sensor data in standard engineering units.
 */
static inline int32_t raw_to_scaled(const sensor_hal_t *hal, int32_t counts)
{
	/* The unit increment per count is peak-to-peak range divided
	 * by full-scale resolution.
	 */
	return (counts * (2 * (int32_t)(hal->range))) >> hal->resolution;
}

/** \brief Convert scaled sensor data to raw counts
 *
 * This routine converts a linearly scaled integer value in engineering
 * units to the corresponding "raw" reading value for the device,
 * using the device-specific range in standard engineering units and full
 * scale resolution parameters in "opts".
 *
 * \param hal       An initialized hardware interface descriptor.
 * \param value     A scaled sensor sample value.
 *
 * \return Scaled signed sensor data in standard engineering units.
 */
static inline int32_t scaled_to_raw(const sensor_hal_t *hal, int32_t value)
{
	/* The unit increment per count is peak-to-peak range divided
	 * by full-scale resolution.
	 */
	return (value << hal->resolution) / (2 * (int32_t)(hal->range));
}


//sensors.h/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//==**"函数"宏定义**Functions macro**=========================//
////////////////////////////////////////////////////////////////////////////

extern sensor_data_t *bma222_data;   //Sensor Data Descriptor
extern sensor_hal_t *bma222_hal;       //Sensor Hardware Abstraction Descriptor





/* Application configuration constants */

#define SCALED_DATA     (true) /* If true, convert sensor data to std. units */




////////////////////////////////////////////////////////////////////////////
//==**"此模块专用"函数声明**Exported Module-specific funcitions**====//
////////////////////////////////////////////////////////////////////////////

//配置: 测试范围，即量程选择 (bma222三轴加速度传感器)
void bma222_set_range(unsigned char range);

//配置: 带宽 (bma222三轴加速度传感器)
void bma222_set_bandwidth(unsigned char bandwidth);

//配置: 指定寄存器的值 (bma222三轴加速度传感器)
void bma222_set_RegVaule(unsigned char reg, unsigned char value);



//读取: 芯片的ID号(bma222三轴加速度传感器) 
unsigned char bma222_get_device_id(void);


unsigned char bma222_get_RegVaule(unsigned char reg);






//从传感器读取:加速度数据(bma222三轴加速度传感器)
static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data);

////////////////////////////////////////////////////////////////////////
//==**"外部" API 函数声明**Exported  API funcitions**===============//
////////////////////////////////////////////////////////////////////////////

//初始化bma222 (bma222三轴加速度传感器)
extern void bma222_init(void);   

//根据读取的类型，读取bma222 数据 (bma222三轴加速度传感器)
extern void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data);

//通过IIC从指定地址，连续读取多个数据(bma222三轴加速度传感器)
extern void bma222_bus_read(unsigned char addr, void *data, unsigned char count);

extern void ReadAllEventVaule_BMA222(void); //读取BMA222所有加速度数据(bma222三轴加速度传感器) 








/* Application configuration constants */

#define SCALED_DATA     (true) /* If true, convert sensor data to std. units */




////////////////////////////////////////////////////////////////////////////
//==**"此模块专用"函数声明**Exported Module-specific funcitions**====//
////////////////////////////////////////////////////////////////////////////

//配置: 测试范围，即量程选择 (bma222三轴加速度传感器)
void bma222_set_range(unsigned char range);

//配置: 带宽 (bma222三轴加速度传感器)
void bma222_set_bandwidth(unsigned char bandwidth);

//配置: 指定寄存器的值 (bma222三轴加速度传感器)
void bma222_set_RegVaule(unsigned char reg, unsigned char value);



//读取: 芯片的ID号(bma222三轴加速度传感器) 
unsigned char bma222_get_device_id(void);


unsigned char bma222_get_RegVaule(unsigned char reg);






//从传感器读取:加速度数据(bma222三轴加速度传感器)
static void bma222_get_accel(sensor_hal_t *hal, sensor_data_t *data);

////////////////////////////////////////////////////////////////////////
//==**"外部" API 函数声明**Exported  API funcitions**===============//
////////////////////////////////////////////////////////////////////////////

//初始化bma222 (bma222三轴加速度传感器)
extern void bma222_init(void);   

//根据读取的类型，读取bma222 数据 (bma222三轴加速度传感器)
extern void bma222_read(sensor_hal_t  *hal, sensor_read_t type, sensor_data_t *data);

//通过IIC从指定地址，连续读取多个数据(bma222三轴加速度传感器)
extern void bma222_bus_read(unsigned char addr, void *data, unsigned char count);

extern void ReadAllEventVaule_BMA222(void); //读取BMA222所有加速度数据(bma222三轴加速度传感器) 




#endif /* _bma222_h_ */









