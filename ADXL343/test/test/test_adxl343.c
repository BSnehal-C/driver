#ifdef TEST

#include "unity.h"

#include "adxl343.h"
#include "FunctionStatus.h"                     //!< Include to use the generic function status enumeration type
#include "mock_i2c_driver.h"



adxl343_handle handle = {

    .is_initialized = true
};

void setUp(void)
{
    

}

void tearDown(void)
{
}


/*FunctionStatus i2c_read__(unsigned char* dataToRead, long long unsigned int length, unsigned int timeout, int x)
{
    if(x ==0)
        *dataToRead = 0xE5;
    else
        *dataToRead = 0x0;
    return FUNCTION_STATUS_OK;
}*/
/*
*@brief: this test case tests if getDeviceID read correct device id.  
*/
void test_getCorrectDeviceId(void)
{
    uint8_t device_id ;
    uint8_t Expected_device_id = 0xe5;


    i2c_write_IgnoreAndReturn(FUNCTION_STATUS_OK);
    i2c_write_IgnoreAndReturn(FUNCTION_STATUS_OK);      
   

    i2c_read_ExpectAndReturn(NULL,1,0,FUNCTION_STATUS_OK); 
    i2c_read_IgnoreArg_dataToRead(); 
    i2c_read_ReturnThruPtr_dataToRead(&Expected_device_id);
   
    
    FunctionStatus result = getDeviceID(&handle, &device_id);

    TEST_ASSERT_EQUAL_UINT8(Expected_device_id, device_id); 
}

/*
*@brief: this test case tests if getDeviceID returns with FUNCTION_STATUS_OK with correct Device id.  
*/
void test_return_getDeviceValue(void)
{
    uint8_t device_id = 0xE5;
    uint8_t Expected_device_id;
    i2c_write_IgnoreAndReturn(FUNCTION_STATUS_OK);
    i2c_write_IgnoreAndReturn(FUNCTION_STATUS_OK);
       
    i2c_read_ExpectAndReturn(NULL,1,0,FUNCTION_STATUS_OK); 
    i2c_read_IgnoreArg_dataToRead(); 
    i2c_read_ReturnThruPtr_dataToRead(&device_id);
   
    
    FunctionStatus result = getDeviceID(&handle, &Expected_device_id);
    TEST_ASSERT(result == FUNCTION_STATUS_OK);
    TEST_ASSERT_EQUAL_UINT8(Expected_device_id, device_id); 
}



#endif // TEST
