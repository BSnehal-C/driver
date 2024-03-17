
// --------------------------------------------------------------------------------------------------------------------
// Include files 
// --------------------------------------------------------------------------------------------------------------------

// CompilerIncludes
//  All include files that are provided by the compiler directly
#include <stdbool.h>                            //!< Include to use standard boolean
#include <stdint.h>                             //!< Include to use integer types

// ProjectIncludes
// All include files that are provided by the project
#include "FunctionStatus.h"                     //!< Include to use the generic function status enumeration type
// --------------------------------------------------------------------------------------------------------------------
// Constant and macro definitions 
// --------------------------------------------------------------------------------------------------------------------
/*I2c slave Address 7bit*/
#define I2C_ADDR1  0x1D
#define I2C_ADDR2  0x53
 
#define DEVICE_ID   0xE5



#define ADXL343_REG_DEVID  			0x00
#define ADXL343_REG_BW_RATE  		0x2C
#define ADXL343_REG_DATA_FORMAT   	0x31
#define ADXL343_REG_POWER_CTL  		0x2D






/**/

#define ADXL_FULL_RES		(1 << 3)
#define ADXL_RANGE_PM_2g	0
#define ADXL_RANGE_PM_4g	1
#define ADXL_RANGE_PM_8g	2
#define ADXL_RANGE_PM_16g	3



#define ASSERT(exp) {	\
	if((exp)) 			\
	{					\
		return exp;		\
	}\
}

#define isRegInvalid(reg) !(((reg) == 0) || (((reg) >= 0x1D) && ((reg) <= 0x39)))


#define I2C_ADDR_W(Add)  ((Add) << 1)
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
   
} adxl343_range_enum;
typedef struct adxl343_handle
{
	/*Slave address used to communicate with adxl343*/	
	uint8_t i2c_address;
	
	/*This indicates - timeout in ms for clock stretching allowed by slave.*/
	uint8_t i2c_timeout;

	bool is_initialized;
	
}	adxl343_handle;
	
/* -------------------------------------------------------------------------------------------------------------------
* This struct must be declared somewhere to use the ADC.
* Only 2 instances per i2c bus are possible , as only 2 I2c addreses possible to configure
* -------------------------------------------------------------------------------------------------------------------
*/
typedef struct adxl
{
	/*Slave address used to communicate with adxl343*/	
	uint8_t i2c_address;
	
	/*This indicates - timeout in ms for clock stretching allowed by slave.*/
	uint8_t i2c_timeout;

	/* enum to set g range.
	@ref ADXL3XX_REG_DATA_FORMAT register
	@default value: +/- 2g */
	adxl343_range_enum  range;

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
	
	 
  
	/*declared other defualt config as required*/
} adxl343_config;

// --------------------------------------------------------------------------------------------------------------------
// Function declarations 
// --------------------------------------------------------------------------------------------------------------------


/*******************************************************************************************************************
* 
* @brief:	This Function initializes ADXL343 with adxl343_conf and sets I2C slave address and timeout
* @param:  	adxl343_conf: Pointer to adxl343_config 
*		   	i2c_address: The 7-bit I2C address of device.
*			i2c_timeout: The I2C timeout in ms for clock stretching allowed by slave device.	
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_WRONG_DEVICE_ID if device returns device id other than 0xE5		
*******************************************************************************************************************/
extern FunctionStatus adxl343_init(const adxl343_config *adxl343_conf, adxl343_handle *handle );

/*******************************************************************************************************************
* 
* @brief:  This Function sets the data rate for the ADXL343 
* @param:  adxl343_i2c: pointer to adxl343_handle 
*		   DataRate: The data rate to set
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
* @note: consider i2c buadrate while setting data rate of accelerometer, ref data sheet
*******************************************************************************************************************/

extern FunctionStatus setDataRate(const adxl343_handle *adxl343_i2c, adxl343_dataRate DataRate);



/**********************************************************************************
* @brief: This Function sets the data rate for the ADXL343 
* @param:  adxl343_i2c: pointer to adxl343_handle 
*		   DataRange: The data rate to set
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/

extern FunctionStatus setDataRange(const adxl343_handle *adxl343_i2c, adxl343_range_enum DataRange);


/**********************************************************************************
* @brief: This Function gets device id of adxl343
* @param:  handle: pointer to adxl343_handle 
*		   device_id  : pointer to variable where device id stored by function
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/
extern FunctionStatus getDeviceID(const adxl343_handle *adxl343, uint8_t* device_id);


/**********************************************************************************
* @brief:  This Function updates FULL_RES bit in DATA_FORMAT register
* @param[in]: handle -  Pointer to an adxl343_handle structure
* @param[in]: is_enable - Represents FULL_RES bit value
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
extern FunctionStatus update_full_res_bit(const adxl343_handle *handle, bool is_enable);
	
/**********************************************************************************
* @brief:  This Function gets FULL_RES bit value in DATA_FORMAT register
* @param[in]: *handle -  Pointer to an adxl343_handle structure
* @param[out]: *is_enable - Pointer to bool datatype, FULL_RES bit value read from DATA_FORMAT register
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
extern FunctionStatus get_full_res_bit(const adxl343_handle *handle, bool *is_enable);	