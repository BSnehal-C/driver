// --------------------------------------------------------------------------------------------------------------------
/// \file  adxl343.c
/// \brief Description
// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------
// Include files 
// --------------------------------------------------------------------------------------------------------------------

// ProjectIncludes
// All include files that are provided by the project
#include "i2c_driver.h"
#include "adxl343.h"
 
static FunctionStatus read8(const adxl343_handle *handle,  uint8_t RegAddr, uint8_t* data);
static FunctionStatus read16(const adxl343_handle *handle,  uint8_t RegAddr, uint8_t* data);
static uint8_t write8(const adxl343_handle *handle,  uint8_t RegAddr, const uint8_t* data);
static uint8_t write16(const adxl343_handle *handle,  uint8_t RegAddr, const uint8_t* data);



/*******************************************************************************************************************
* 
* @brief:	This Function initializes ADXL343 with adxl343_conf and sets I2C slave address and timeout
* @param:  	adxl343_conf[in]: Pointer to adxl343_config 
*           adxl343_conf[out]: Pointer to adxl343_handle	
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
*							Returns FUNCTION_WRONG_DEVICE_ID if device returns device id other than 0xE5		
*******************************************************************************************************************/
FunctionStatus adxl343_init(const adxl343_config *adxl343_conf, adxl343_handle *handle )
{
	uint8_t device_id;

	handle->i2c_address = adxl343_conf->i2c_address;
	handle->i2c_timeout = adxl343_conf->i2c_timeout;
	handle->is_initialized = true;

	/*check if adxl device returns correct device id to ensure successfull communication between adxl343 chip and uC.*/
	FunctionStatus Status = getDeviceID(handle, &device_id);	
	if((Status == FUNCTION_STATUS_OK) && (device_id  == DEVICE_ID))
	{
		/*adxl343 initialized successfully*/
		
	}	
	else
	{
		/*error in communication - ensure i2c driver initialized and correct slave address is set.*/
		handle->is_initialized = false;
		if(Status == FUNCTION_STATUS_OK)
			Status = FUNCTION_WRONG_DEVICE_ID;		
		
	}

	if(FUNCTION_STATUS_OK == Status)
	{
		/*set data rate*/
        setDataRate(handle, adxl343_conf->data_rate);
		
		/*set data range*/
		setDataRange(handle, adxl343_conf->range);
	
		/*set resolution*/
		update_full_res_bit(handle, adxl343_conf->full_res);

		handle->operating_mode = ADXL343_BYPASS_MODE;
	}
	
	
	return Status;
}
/**********************************************************************************
* 
* @brief:  This Function sets the data rate for the ADXL343 
* @param:  handle: pointer to adxl343_handle 
*		   DataRate: The data rate to set
* @return: FunctionStatus  	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                          	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
* @note: consider i2c buadrate while setting data rate of accelerometer, ref data sheet
/**********************************************************************************/
FunctionStatus setDataRate(const adxl343_handle *handle, adxl343_dataRate DataRate)
{
	
	uint8_t Data;
	
	ASSERT(read8(handle, ADXL343_REG_BW_RATE, &Data));

	Data = (Data & 0xF0) | ((uint8_t)DataRate & 0x0F);
	ASSERT(write8(handle,  ADXL343_REG_BW_RATE, &Data));

	return FUNCTION_STATUS_OK;
}

/**********************************************************************************
* @brief: This Function sets the data rate for the ADXL343 
* @param[IN,OUT]:   handle: pointer to adxl343_handle 
* @param[IN]: 		DataRange: The data rate to set
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/
FunctionStatus setDataRange(adxl343_handle *handle, adxl343_range DataRange)
{
	uint8_t Data;

	handle->data_range = DataRange;
	ASSERT(read8(handle, ADXL343_REG_DATA_FORMAT, &Data));

	Data = (Data & 0xFC) | ((uint8_t)DataRange & 0x03);
	ASSERT(write8(handle,  ADXL343_REG_DATA_FORMAT, &Data));

	return FUNCTION_STATUS_OK;
}

/**********************************************************************************
* @brief: This Function gets device id of adxl343
* @param:  handle: pointer to adxl343_handle 
*		   device_id  : pointer to variable where device id stored by function
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the transmission was successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
/**********************************************************************************/
FunctionStatus getDeviceID(const adxl343_handle *handle, uint8_t* device_id)
{	
	
	FunctionStatus status = read8(handle, ADXL343_REG_DEVID, device_id);
	if((status == FUNCTION_STATUS_OK) && (*device_id != DEVICE_ID))
	{
		return FUNCTION_STATUS_ERROR;
	}	

	return status;
}


/**********************************************************************************
* @brief:  This Function updates FULL_RES bit in DATA_FORMAT register
* @param: handle[IN], OUT] -  Pointer to an adxl343_handle structure
* @param[IN]: is_enable - Represents FULL_RES bit value
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
FunctionStatus update_full_res_bit(adxl343_handle *handle, bool is_enable)
{
    uint8_t data;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	handle->full_res = is_enable;
	/* read old value of DATA_FORMAT register */
    ASSERT(read8(handle, ADXL343_REG_DATA_FORMAT, &data));         			
    
	/* reset FULL_RES bit */       
    data &= ~(1 << 3);       

	/* update resolution with new value*/                                                                   
    data |= (is_enable << 3);                                                                      
    
    return write8(handle, ADXL343_REG_DATA_FORMAT, &data);       
}

/**********************************************************************************
* @brief:  This Function gets FULL_RES bit value in DATA_FORMAT register
* @param[IN]: *handle -  Pointer to an adxl343_handle structure
* @param[OUT]: *is_enable - Pointer to bool datatype, FULL_RES bit value read from DATA_FORMAT register
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
FunctionStatus get_full_res_bit(const adxl343_handle *handle, bool *is_enable)
{
    uint8_t data;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read old value of DATA_FORMAT register */
    ASSERT(read8(handle, ADXL343_REG_DATA_FORMAT, &data));             
                                                                  
    
	/* reset data value except FULL_RES bit */  
	data &= (1 << 3);    

	/* update resolution with read value*/                                                                      
    *is_enable = (bool)(data >> 3);   

    return FUNCTION_STATUS_OK;                                                                                  
}

/**********************************************************************************
* @brief:  This Function sets measurement bit in POWER_CTL register and puts adx343 in measurement mode
* @param:  *handle[IN] -  Pointer to an adxl343_handle structure
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/**********************************************************************************/
FunctionStatus start_measurment(const adxl343_handle *handle)
{
    uint8_t data;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read old value of DATA_FORMAT register */
    ASSERT(read8(handle, ADXL343_REG_POWER_CTL, &data));             
              
	/* set measurement bit*/                                                                      
    /* reset data value except Measure bit */  
	data |= (1 << 3);   

    return write8(handle, ADXL343_REG_POWER_CTL, &data);                                                                                  
}

/************************************************************************************************
* @brief:  This Function clears measurement bit in POWER_CTL register and puts adx343 in standby mode
* @param:  *handle[IN] -  Pointer to an adxl343_handle structure
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
/***************************************************************************************************/
FunctionStatus stop_measurment(const adxl343_handle *handle)
{
    uint8_t data;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read old value of DATA_FORMAT register */
    ASSERT(read8(handle, ADXL343_REG_POWER_CTL, &data));             
              
	/* clear measurement bit*/                                                                       
	data &= ~(1 << 3);   

    return write8(handle, ADXL343_REG_POWER_CTL, &data);                                                                                  
}

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
FunctionStatus enable_interrupt(const adxl343_handle *handle, adxl343_interrupt int_type)
{
    uint8_t data;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read old value of DATA_FORMAT register */
    ASSERT(read8(handle, ADXL343_REG_INT_ENABLE, &data));             
              
	/* set measurement bit*/
	data |= (1 << int_type);   

    return write8(handle, ADXL343_REG_POWER_CTL, &data);                                                                                  
}
/**********************************************************************************
* @brief:  This Function gets scaled x-axis acceleration value in g unit
* @param[in]: *handle - Pointer to an adxl343_handle structure
* @param[out]:*DataX - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only								
/**********************************************************************************/
FunctionStatus get_DATAX(const adxl343_handle *handle, double *DataX)
{
	/*
	read raw data from register DATAX
	raw data is 2's complement format - need to convert it in double	

	*/

	uint8_t buf[2];
	int16_t RawDataX;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read DataX */
    ASSERT(read16(handle, ADXL343_REG_DATAX0, buf));

	if(handle->operating_mode == ADXL343_BYPASS_MODE)
	{
		
		/*to understand calculation*//*ToDo: Remove comment*/
		/*ref:https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf*/

		RawDataX = (int16_t)(buf[1] << 8) | buf[0];

		/*for full resolution with range +- 16g data is 13 bit so resolution is 32/2^13 = 0.0039*/
		if(handle->full_res == true)
		{
			*DataX = (double)(RawDataX) * 0.0039;
		}	
		else
		{
			if(handle->data_range == ADXL343_RANGE_2_G)
				*DataX = (double)(RawDataX) * 0.0039;
			else if(handle->data_range == ADXL343_RANGE_4_G)
				*DataX = (double)(RawDataX) * 0.0078;
			else if(handle->data_range == ADXL343_RANGE_8_G)
				*DataX = (double)(RawDataX) * 0.0156;	
			else if(handle->data_range == ADXL343_RANGE_16_G)
				*DataX = (double)(RawDataX) * 0.0312;		
		}

	}
	else{

		/*Todo: implementation is pending for other modes*/
	}

	return FUNCTION_STATUS_OK;

}

/**********************************************************************************
* @brief:  This Function gets scaled y-axis acceleration value in g unit
* @param[in]: *handle - Pointer to an adxl343_handle structure
* @param[out]:*DataY - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only								
/**********************************************************************************/
FunctionStatus get_DATAY(const adxl343_handle *handle, double *DataY)
{
	uint8_t buf[2];
	int16_t RawDataY;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read DataY */
    ASSERT(read16(handle, ADXL343_REG_DATAY0, buf));

	if(handle->operating_mode == ADXL343_BYPASS_MODE)
	{
		
		/*to understand calculation*//*ToDo: Remove comment*/
		/*ref:https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf*/

		RawDataY = (int16_t)(buf[1] << 8) | buf[0];

		/*for full resolution with range +- 16g data is 13 bit so resolution is 32/2^13 = 0.0039*/
		if(handle->full_res == true)
		{
			*DataY = (double)(RawDataY) * 0.0039;
		}	
		else
		{
			if(handle->data_range == ADXL343_RANGE_2_G)
				*DataY = (double)(RawDataY) * 0.0039;
			else if(handle->data_range == ADXL343_RANGE_4_G)
				*DataY = (double)(RawDataY) * 0.0078;
			else if(handle->data_range == ADXL343_RANGE_8_G)
				*DataY = (double)(RawDataY) * 0.0156;	
			else if(handle->data_range == ADXL343_RANGE_16_G)
				*DataY = (double)(RawDataY) * 0.0312;		
		}

	}
	else{
		
		/*Todo: implementation is pending for other modes*/
	}

	return FUNCTION_STATUS_OK;

}

/**********************************************************************************
* @brief:  This Function gets scaled z-axis acceleration value in g unit
* @param[in]: *handle - Pointer to an adxl343_handle structure
* @param[out]:*DataZ - Pointer to double datatype
* @return: FunctionStatus 	Returns FUNCTION_STATUS_OK if the operation is successful.
*                         	Returns FUNCTION_STATUS_ERROR for non-specific errors.
*							Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
*                         	Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
							Returns FUNCTION_STATUS_NOT_INITIALIZED if handle is not initialized
* @note: data is returned considering bypass mode and right justified data only							
/**********************************************************************************/
FunctionStatus get_DATAZ(const adxl343_handle *handle, double *DataZ)
{
	/*
	read raw data from register DATAX
	raw data is 2's complement format - need to convert it in double	

	*/

	uint8_t buf[2];
	int16_t RawDataZ;
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	/* read DataZ */
    ASSERT(read16(handle, ADXL343_REG_DATAZ0, buf));

	if(handle->operating_mode == ADXL343_BYPASS_MODE)
	{
		
		/*to understand calculation*//*ToDo: Remove comment*/
		/*ref:https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf*/

		RawDataZ = (int16_t)(buf[1] << 8) | buf[0];

		/*for full resolution with range +- 16g data is 13 bit so resolution is 32/2^13 = 0.0039*/
		if(handle->full_res == true)
		{
			*DataZ = (double)(RawDataZ) * 0.0039;
		}	
		else
		{
			if(handle->data_range == ADXL343_RANGE_2_G)
				*DataZ = (double)(RawDataZ) * 0.0039;
			else if(handle->data_range == ADXL343_RANGE_4_G)
				*DataZ = (double)(RawDataZ) * 0.0078;
			else if(handle->data_range == ADXL343_RANGE_8_G)
				*DataZ = (double)(RawDataZ) * 0.0156;	
			else if(handle->data_range == ADXL343_RANGE_16_G)
				*DataZ = (double)(RawDataZ) * 0.0312;		
		}

	}
	else{
		
		/*Todo: implementation is pending for other modes*/
	}

	return FUNCTION_STATUS_OK;

}



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
FunctionStatus setRegister(const adxl343_handle *handle, const uint8_t Reg, const uint8_t Data)
{
	
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	ASSERT(write8(handle,  Reg, &Data));

    return FUNCTION_STATUS_OK;  
}

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
FunctionStatus getRegister(const adxl343_handle *handle, const uint8_t Reg, uint8_t *Data)
{
	
    /* check handle */
    if (handle == NULL)                                                                         
    {
        return FUNCTION_STATUS_ARGUMENT_ERROR;                                                                               
    }

	/* check if handle initialized */
    if (handle->is_initialized != true)                                                                    
    {
        return FUNCTION_STATUS_NOT_INITIALIZED;                                                                               
    }
    
	ASSERT(read8(handle,  Reg, Data));

    return FUNCTION_STATUS_OK;  
}


static FunctionStatus read8(const adxl343_handle *handle,  uint8_t RegAddr, uint8_t* data)
{
		
	uint8_t write_buff[2];	
	
	if((handle == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address
	*/
	
	write_buff[0] = I2C_ADDR_W(handle->i2c_address) ;
	write_buff[1] = RegAddr;
	ASSERT(i2c_write(write_buff, 2, handle->i2c_timeout));
	
	
	/* 3. write slave address with lsb bit 1*/	
	write_buff[0] = I2C_ADDR_R(handle->i2c_address) ;
	ASSERT(i2c_write(write_buff, 1, handle->i2c_timeout));
	
	/* 4. read byte in Data 	*/
	ASSERT(i2c_read(data, 1, handle->i2c_timeout));	
	
	return FUNCTION_STATUS_OK;
}

static FunctionStatus read16(const adxl343_handle *handle,  uint8_t RegAddr, uint8_t* data)
{
	if((handle == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;
	
	uint8_t write_buff[2];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address
	*/	
	write_buff[0] = I2C_ADDR_W(handle->i2c_address) ;
	write_buff[1] = RegAddr;
	ASSERT(i2c_write(write_buff, 2, handle->i2c_timeout));
	
	/* 3. write slave address with lsb bit 1*/	
	write_buff[0] = I2C_ADDR_R(handle->i2c_address) ;
	ASSERT(i2c_write(write_buff, 1, handle->i2c_timeout));
	
	/* 4. read bytes in Data 	*/
	ASSERT(i2c_read(data, 2, handle->i2c_timeout));

	return FUNCTION_STATUS_OK;	
}

static uint8_t write8(const adxl343_handle *handle,  uint8_t RegAddr, const uint8_t* data)
{
	if((handle == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;	
	uint8_t write_buff[3];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2.  write slave address with lsb bit 0	followed Register address and then 1 bytes of data
	*/
	
	write_buff[0] = I2C_ADDR_W(handle->i2c_address) ;
	write_buff[1] = RegAddr;
	write_buff[2] = *data;
	ASSERT(i2c_write(write_buff, 3, handle->i2c_timeout));
	
	
	return FUNCTION_STATUS_OK;	
}

static uint8_t write16(const adxl343_handle *handle,  uint8_t RegAddr, const uint8_t* data)
{
	if((handle == NULL) || (data == NULL) || isRegInvalid(RegAddr))
		return FUNCTION_STATUS_ARGUMENT_ERROR;	
	
	uint8_t write_buff[4];	
		
	/*sequence to read 1 byte data from RegAddr
	* 1. start
	* 2. write slave address with lsb bit 0	followed Register address and then 2 bytes of data
	*/	
	write_buff[0] = I2C_ADDR_W(handle->i2c_address) ;
	write_buff[1] = RegAddr;
	write_buff[2] = data[0];
	write_buff[3] = data[1];
	ASSERT(i2c_write(write_buff, 4, handle->i2c_timeout));
		
	return FUNCTION_STATUS_OK;	
}