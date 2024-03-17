// --------------------------------------------------------------------------------------------------------------------
// Include files 
// --------------------------------------------------------------------------------------------------------------------

// ProjectIncludes
// All include files that are provided by the project
#include "i2c_driver.h"
#include "adxl343.h"
 
static FunctionStatus read8(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, uint8_t* data);
static FunctionStatus read16(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, uint16_t* data);
static uint8_t write8(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, const uint8_t* data);
static uint8_t write16(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, const uint16_t* data);


/*Initialise adxl343*/
bool adxl343_init(const adxl343_config *adxl343, const adxl343_i2c_info *adxl343_i2c)
{
	if((adxl343 != NULL) && (adxl343_i2c != NULL))
	{
        setDataRate(adxl343_i2c, adxl343->data_rate);
		
		setDataRange(adxl343_i2c, adxl343->range);
	
		/*resolution*/
	}
	
	return false;
}
/**********************************************************************************
* 
* @brief:  This Function sets the data rate for the ADXL343 
* @param:  adxl343_i2c: pointer to adxl343_i2c_info 
*		   DataRate: The data rate to set
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
* @note: consider i2c buadrate while setting data rate of accelerometer, ref data sheet
/**********************************************************************************/
FunctionStatus setDataRate(const adxl343_i2c_info *adxl343_i2c, adxl343_dataRate DataRate)
{
	
	uint8_t Data;

	ASSERT(read8(adxl343_i2c, ADXL343_REG_BW_RATE, &Data));

	Data = (Data & 0xF0) | ((uint8_t)DataRate & 0x0F);
	ASSERT(write8(adxl343_i2c,  ADXL343_REG_BW_RATE, &Data));

	return FUNCTION_STATUS_OK;
}

/**********************************************************************************
* @brief: This Function sets the data rate for the ADXL343 
* @param:  adxl343_i2c: pointer to adxl343_i2c_info 
*		   DataRange: The data rate to set
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/
FunctionStatus setDataRange(const adxl343_i2c_info *adxl343_i2c, adxl343_range_enum DataRange)
{
	uint8_t Data;

	ASSERT(read8(adxl343_i2c, ADXL343_REG_DATA_FORMAT, &Data));

	Data = (Data & 0xFC) | ((uint8_t)DataRange & 0x03);
	ASSERT(write8(adxl343_i2c,  ADXL343_REG_DATA_FORMAT, &Data));

	return FUNCTION_STATUS_ERROR;
}

/**********************************************************************************
* @brief: This Function gets device id of adxl343
* @param:  adxl343_i2c: pointer to adxl343_i2c_info 
*		   device_id  : pointer to variable where device id stored by function
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/
FunctionStatus getDeviceID(const adxl343_i2c_info *adxl343_i2c, uint8_t* device_id)
{	
	
	FunctionStatus status = read8(adxl343_i2c, ADXL343_REG_DEVID, device_id);
	if((status == FUNCTION_STATUS_OK) && (*device_id != DEVICE_ID))
	{
		return FUNCTION_STATUS_ERROR;
	}	

	return status;
}



static FunctionStatus read8(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, uint8_t* data)
{
		
	uint8_t write_buff[2];	
	
	if((adxl343_i2c == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address
	*/
	
	write_buff[0] = I2C_ADDR_W(adxl343_i2c->i2c_address) ;
	write_buff[1] = RegAddr;
	ASSERT(i2c_write(write_buff, 2, adxl343_i2c->i2c_timeout));
	
	
	/* 3. write slave address with lsb bit 1*/	
	write_buff[0] = I2C_ADDR_R(adxl343_i2c->i2c_address) ;
	ASSERT(i2c_write(write_buff, 1, adxl343_i2c->i2c_timeout));
	
	/* 4. read byte in Data 	*/
	ASSERT(i2c_read(data, 1, adxl343_i2c->i2c_timeout));	
	
	return FUNCTION_STATUS_OK;
}

static FunctionStatus read16(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, uint16_t* data)
{
	if((adxl343_i2c == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;
	
	uint8_t write_buff[2];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address
	*/	
	write_buff[0] = I2C_ADDR_W(adxl343_i2c->i2c_address) ;
	write_buff[1] = RegAddr;
	ASSERT(i2c_write(write_buff, 2, adxl343_i2c->i2c_timeout));
	
	/* 3. write slave address with lsb bit 1*/	
	write_buff[0] = I2C_ADDR_R(adxl343_i2c->i2c_address) ;
	ASSERT(i2c_write(write_buff, 1, adxl343_i2c->i2c_timeout));
	
	/* 4. read byte in Data 	*/
	ASSERT(i2c_read((uint8_t*)data, 2, adxl343_i2c->i2c_timeout));

	return FUNCTION_STATUS_OK;	
}

static uint8_t write8(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, const uint8_t* data)
{
	if((adxl343_i2c == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;	
	uint8_t write_buff[3];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2.  write slave address with lsb bit 0	followed Register address and then 1 bytes of data
	*/
	
	write_buff[0] = I2C_ADDR_W(adxl343_i2c->i2c_address) ;
	write_buff[1] = RegAddr;
	write_buff[2] = *data;
	ASSERT(i2c_write(write_buff, 3, adxl343_i2c->i2c_timeout));
	
	
	return FUNCTION_STATUS_OK;	
}

static uint8_t write16(const adxl343_i2c_info *adxl343_i2c,  uint8_t RegAddr, const uint16_t* data)
{
	if((adxl343_i2c == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;	
	
	uint8_t write_buff[4];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address and then 2 bytes of data
	*/	
	write_buff[0] = I2C_ADDR_W(adxl343_i2c->i2c_address) ;
	write_buff[1] = RegAddr;
	write_buff[2] = (*data) & 0xFF;
	write_buff[3] = (*data) >> 8 & 0xFF;;
	ASSERT(i2c_write(write_buff, 4, adxl343_i2c->i2c_timeout));
	
	
	
	return FUNCTION_STATUS_OK;	
}