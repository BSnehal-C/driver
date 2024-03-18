

#ifndef INC_ADXL_DRIVER_H_
#define INC_ADXL_DRIVER_H_

/**
 * @file adxl343.h.h
 * @brief adxl343 Driver Module Interface
 *
 * @defgroup hal Hardware Abstraction Layer (HAL)
 * @brief Abstracts hardware specifics through a unified API.
 *
 * This module provides an interface for ADXL343 accelerometer over I2C (Inter-Integrated Circuit) communication. 
 * It abstracts the lower-level details of hardware interaction, facilitating communication with ADXL343. 
 * The module is designed to be portable, efficient, and easy to use. 
 * It includes functions for initializing the ADXL343, reading from and writing to ADXL343 registers, 
 * with robust error management.
 *
 * @{
 */


// --------------------------------------------------------------------------------------------------------------------
// Include files 
// --------------------------------------------------------------------------------------------------------------------

// CompilerIncludes
//  All include files that are provided by the compiler directly
#include <stdbool.h>                            //!< Include to use standard boolean
#include <stdint.h>                             //!< Include to use integer types

// ProjectIncludes
// All include files that are provided by the project
#include "i2c_driver.h"
#include "FunctionStatus.h"                     //!< Include to use the generic function status enumeration type
// --------------------------------------------------------------------------------------------------------------------
// Constant and macro definitions 
// --------------------------------------------------------------------------------------------------------------------
/*7bit I2c slave Address */
#define I2C_ADDR1  0x1D
#define I2C_ADDR2  0x53
 
/*DevId - holds a fixed device ID code of 0xE5*/ 
#define DEVICE_ID   0xE5


/**************************************Start ADXL343 REGISTERS**********************************************/

#define ADXL343_REG_DEVID  			0x00
#define ADXL343_REG_BW_RATE  		0x2C
#define ADXL343_REG_POWER_CTL  		0x2D
#define ADXL343_REG_INT_ENABLE		0x2E
#define ADXL343_REG_DATA_FORMAT   	0x31
#define ADXL343_REG_DATAX0			0x32
#define ADXL343_REG_DATAY0			0x34
#define ADXL343_REG_DATAZ0			0x36

/***************************************End ADXL343 REGISTERS**********************************************/


#define ASSERT(exp) {	\
	if((exp)) 			\
	{					\
		return exp;		\
	}\
}
/*checks given Register address is valid*/
#define isRegInvalid(reg) !(((reg) == 0) || (((reg) >= 0x1D) && ((reg) <= 0x39)))

/*Converts 7bit address to 8 bit slave address - 7 bit address followed by W bit*/
#define I2C_ADDR_W(Add)  ((Add) << 1)

/*Converts 7bit address to 8 bit slave address - 7 bit address followed by R bit*/
#define I2C_ADDR_R(Add)  (((Add) << 1) | 1)
   
// --------------------------------------------------------------------------------------------------------------------
// Type definitions. 
// --------------------------------------------------------------------------------------------------------------------
/* @brief: enum to set data rate.
   @ref BW_RATE-0x2C register*/
typedef enum {
   ADXL343_DATARATE_0_10_HZ = 0,
   ADXL343_DATARATE_0_20_HZ = 1, 
   ADXL343_DATARATE_0_39_HZ = 2, 
   ADXL343_DATARATE_0_78_HZ = 3,
   ADXL343_DATARATE_1_56_HZ = 4, 
   ADXL343_DATARATE_3_13_HZ = 5,
   ADXL343_DATARATE_6_25HZ = 6,
   ADXL343_DATARATE_12_5_HZ = 7,
   ADXL343_DATARATE_25_HZ = 8,
   ADXL343_DATARATE_50_HZ = 9,
   ADXL343_DATARATE_100_HZ = 10, 
   ADXL343_DATARATE_200_HZ = 11, 
   ADXL343_DATARATE_400_HZ = 12,
   ADXL343_DATARATE_800_HZ = 13,
   ADXL343_DATARATE_1600_HZ = 14,     
   ADXL343_DATARATE_3200_HZ = 15    
     
 } adxl343_dataRate;

/* @brief: enum to set range g range.
   @ref DATA_FORMAT-0x31 register*/
typedef enum {
  ADXL343_RANGE_2_G ,  /* +/- 2g */
  ADXL343_RANGE_4_G ,  /* +/- 4g */
  ADXL343_RANGE_8_G ,  /* +/- 8g */
  ADXL343_RANGE_16_G , /* +/- 16g */
   
} adxl343_range;

typedef enum
{
    ADXL343_BYPASS_MODE  = 0x00,        /*bypass mode */
    ADXL343_FIFO_MODE    = 0x01,        /*fifo mode */
    ADXL343_STREAM_MODE = 0x02,        /*stream mode */
    ADXL343_TRIGGER_MODE = 0x03,        /*trigger mode */
} adxl343_mode;

/**
 * @brief adxl343 interrupts
 */
typedef enum
{
    ADXL343_INTERRUPT_DATA_READY = 0x07,        /* data ready */
    ADXL343_INTERRUPT_SINGLE_TAP = 0x06,        /* single tap */
    ADXL343_INTERRUPT_DOUBLE_TAP = 0x05,        /* double tap */
    ADXL343_INTERRUPT_ACTIVITY   = 0x04,        /* activity */
    ADXL343_INTERRUPT_INACTIVITY = 0x03,        /* inactivity */
    ADXL343_INTERRUPT_FREE_FALL  = 0x02,        /* free fall */
    ADXL343_INTERRUPT_WATERMARK  = 0x01,        /* watermark */
    ADXL343_INTERRUPT_OVERRUN    = 0x00,        /* overrun */
} adxl343_interrupt;

/* -------------------------------------------------------------------------------------------------------------------
* This struct must be declared somewhere to use the ADC.
* Only 2 instances per i2c bus are possible , as only 2 I2c addreses possible to configure
* -------------------------------------------------------------------------------------------------------------------*/
typedef struct adxl343_handle
{
	/*7bit Slave address used to communicate with adxl343
	it can be either I2C_ADDR1 or I2C_ADDR2*/	
	uint8_t i2c_address;
	
	/*This indicates - timeout in ms for clock stretching allowed by slave.*/
	uint8_t i2c_timeout;

	adxl343_mode operating_mode;

	/* enum to set g range.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: +/- 2g */
	adxl343_range  data_range;

 
	/*to set FULL_RES Bit.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: 10bit mode */
	bool full_res;

	/*This indicates - if handle is initialized.*/
	bool is_initialized;
	

}adxl343_handle;
	
/* -------------------------------------------------------------------------------------------------------------------
* This struct used to set configuration of adxl343 
* -------------------------------------------------------------------------------------------------------------------
*/
typedef struct adxl
{
	/*Slave address used to communicate with adxl343*/	
	/*It can be either I2C_ADDR1 or I2C_ADDR2*/
	uint8_t i2c_address;
	
	/*This indicates - timeout in ms for clock stretching allowed by slave.*/
	uint8_t i2c_timeout;

	/* enum to set g range.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: +/- 2g */
	adxl343_range  range;

	/* @brief: enum to set data rate.
    @ref BW_RATE-0x2C register
	@default value: ADXL343_DATARATE_100_HZ 
	@note: consider i2c buadrate while setting data rate of accelerometer, ref data sheet*/
    adxl343_dataRate data_rate;

  
	/*to set FULL_RES Bit.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: 10bit mode */
	bool full_res; 
	
 
	/*todo - below configuration not implemented in init function */
	/*to set Justify Bit Bit.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: right-justified mode with sign extension. */
	//bool justify; 	

	/*setting true enables link mode*/
	//bool link_enable;
	
	/* to set INT_INVERT Bit.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: active high */
	//bool interrupt_inv; 

	/*setting true enables auto sleep Mode*/
	/*note: enabling auto sleeps also enables link mode*/
	//bool auto_sleep_enable;
	
	
} adxl343_config ;


// --------------------------------------------------------------------------------------------------------------------
// Function declarations 
// --------------------------------------------------------------------------------------------------------------------


/******************************************************************************************************************** 
* @brief:	This Function initializes ADXL343 with adxl343_conf and sets I2C slave address and timeout
* @param:  	adxl343_conf[IN]: Pointer to adxl343_config
* @param	handle[OUT]: Pointer to handle
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_WRONG_DEVICE_ID if device returns device id other than 0xE5		
*******************************************************************************************************************/
extern FunctionStatus adxl343_init(const adxl343_config *adxl343_conf, adxl343_handle *handle );

/******************************************************************************************************************** 
* @brief:  This Function sets the data rate for the ADXL343 
* @param:  handle[IN]: pointer to adxl343_handle 
*		   DataRate[IN]: The data rate to set
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: consider i2c buadrate while setting data rate of accelerometer, ref data sheet
*******************************************************************************************************************/

extern FunctionStatus setDataRate(const adxl343_handle *handle, adxl343_dataRate DataRate);

/**********************************************************************************
* @brief: This Function sets the data rate for the ADXL343 
* @param[IN,OUT]:   handle: pointer to adxl343_handle 
* @param[IN]: 		DataRange: The data rate to set
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/

extern FunctionStatus setDataRange( adxl343_handle *handle, adxl343_range DataRange);

/**********************************************************************************
* @brief: This Function gets device id of adxl343
* @param:  handle[IN]: pointer to adxl343_handle 
*		   device_id[IN]  : pointer to variable where device id stored by function
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/

extern FunctionStatus getDeviceID(const adxl343_handle *adxl343, uint8_t* device_id);

/**********************************************************************************
* @brief:  This Function updates FULL_RES bit in DATA_FORMAT register
* @param: handle [IN][OUT] - Pointer to an adxl343_handle structure
* @param: is_enable [IN] - Represents FULL_RES bit value
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/

extern FunctionStatus update_full_res_bit(adxl343_handle *handle, bool is_enable);
	
/**********************************************************************************
* @brief:  This Function gets FULL_RES bit value in DATA_FORMAT register
* @param: *handle [IN]-  Pointer to an adxl343_handle structure
* @param: *is_enable [OUT] - Pointer to bool datatype, FULL_RES bit value read from DATA_FORMAT register
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/

extern FunctionStatus get_full_res_bit(const adxl343_handle *handle, bool *is_enable);	

/**********************************************************************************
* @brief:  This Function sets measurement bit in POWER_CTL register and puts adx343 in measurement mode
* @param:  *handle[IN] -  Pointer to an adxl343_handle structure
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
extern FunctionStatus start_measurment(const adxl343_handle *handle);

/************************************************************************************************
* @brief:  This Function clears measurement bit in POWER_CTL register and puts adx343 in standby mode
* @param:  *handle[IN] -  Pointer to an adxl343_handle structure
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/***************************************************************************************************/
FunctionStatus stop_measurment(const adxl343_handle *handle);

/**********************************************************************************
* @brief:  This Function enables interrupt of type mentioned by int_type
* @param:  *handle[IN] -  Pointer to an adxl343_handle structure
* @param:  int_type[IN] -  Define interrupt type to be enabled
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
FunctionStatus enable_interrupt(const adxl343_handle *handle, adxl343_interrupt int_type);

/**********************************************************************************
* @brief:  This Function gets scaled x-axis acceleration value in g unit
* @param: *handle[in] - Pointer to an adxl343_handle structure
* @param: *DataX[out] - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only								
/**********************************************************************************/
FunctionStatus get_DATAX(const adxl343_handle *handle, double *DataX);

/**********************************************************************************
* @brief:  This Function gets scaled y-axis acceleration value in g unit
* @param: *handle[in] - Pointer to an adxl343_handle structure
* @param: *DataY[out] - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only								
/**********************************************************************************/
extern FunctionStatus get_DATAY(const adxl343_handle *handle, double *DataY);

/**********************************************************************************
* @brief:  This Function gets scaled z-axis acceleration value in g unit
* @param: *handle[in] - Pointer to an adxl343_handle structure
* @param: *DataZ[out] - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only								
/**********************************************************************************/
extern FunctionStatus get_DATAZ(const adxl343_handle *handle, double *DataZ);

/**********************************************************************************
* 
* @brief:  This Function writes the 8bit data to Register 
* @param:  handle: pointer to adxl343_handle 
* @param:  Reg[IN]: The Register to be updated with value of Data parameter
* @param:  Data[IN]: The Data to be set in Reg
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: do not set operating mode, data range and full resolution using this API
/**********************************************************************************/
extern FunctionStatus setRegister(const adxl343_handle *handle, const uint8_t Reg, const uint8_t Data);

/**********************************************************************************
* 
* @brief:  This Function reads the 8bit data from Register 
* @param:  handle: pointer to adxl343_handle 
* @param:  Reg[IN]: The Register to be updated with value of Data parameter
* @param:  Data[IN]: The Data to be set in Reg
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
extern FunctionStatus getRegister(const adxl343_handle *handle, const uint8_t Reg, uint8_t *Data);



#endif