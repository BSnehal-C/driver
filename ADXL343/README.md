# About ADXL343

The ADXL343 is a  3-axis accelerometer by Analog Devices. It has with 10- to 13-bit resolution with measurement at up to ±16 g. Digital output data is formatted as 16-bit twos complement and is accessible through either a SPI (3- or 4-wire) or I2C digital interface.

#  ADXL343 Driver
Driver ADXL343 is the driver of adxl343 developed to work in bypass mode with data ready interrupt. It provides acceleration scaled reading in bypass mode of operation and provides functions to initialize required registers.
This driver is scalable to add full functionality of adxl343.

## Limitations:
1. Functions provided to get acceleration scaled reading only in Bypass Mode.
2. Don't Supports readings with in left justified configuration.
3. Functions provided to set and get registers which required only to operate in Bypass Mode with data ready interrupt.
4. Support for SPI communication is not provided.

## Prerequisite
1. The required hardware connections must be established to function ADXL343 in I2C mode.
2. I2C driver must be initialized.

## Brief Explanation
### Initialization (adxl343_init):

The function initializes the ADXL343 with the provided configuration (adxl343_conf).
It configure handle with the I2C slave address and timeout provided in adxl343_conf. This handle can be used in later stage of I2C communication.
Verifies successful communication by checking the device ID.
Configures data rate, data range, and resolution.
Returns error code if initialization fails.

for example:

```
  static adxl343_handle handle;
  
  const adxl343_config config = {
  
  	.i2c_address = I2C_ADDR1,	
   
  	.i2c_timeout = 100,
   
  	.range = ADXL343_RANGE_2_G,
   
  	.data_rate = ADXL343_DATARATE_100_HZ,
   
  	.full_res = true; 
   
   }
    
  int main(void)
  {  
  	adxl343_init(&config, &handle);   
  }

```

### Starting Measurement

Measurement can be started by calling API start_measurment().
Asynchronous readings of acceleration data can lead to accessing the acceleration data registers while they are being updated.
To avoid this, it is recommended to enable DATA_READY interrupt functionality, so that the host processor samples immediately after the DATA_READY interrupt goes high.
DATA READY interrupt can be enabled using API "enable_interrupt" after starting measurement.


### Runtime 

Acceleration scaled data can be read by using APIs get_DATAX(), get_DATAY(), get_DATAZ() once DATA_READY interrupt occurred.
These APIs returns error code for unsuccessful communication.


### Updating configuration at runtime


#### Below APIs are given to write and read configuration at runtime.

setDataRate, getDataRate 

setDataRange, getDataRange

update_full_res_bit



#### Generic functions to write and read data to/from registers.

setRegister

getRegister


### It is recommended that to stop measurement before changing configuration in run time, then start again with below APIs.

start_measurment

stop_measurment



## Folder structure
/src includes ADXL343 source and header files .

/test Includes unit test code with ceeding, cmock supporting files

/doc Includes all reference documents and data sheet of adxl343.

### Folders to be added
/examples includes driver ADXL343 sample application code.


## Instructions to Build files
### Prerequisite
gcc must be installed on your system

Note: This driver is tested on gcc version 13.2.0

Driver can be build by running Run.Bat file provided in driver\ADXL343 folder 

## Reference Documents

https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf
https://electronics.stackexchange.com/questions/545643/g-lsb-meaning-in-accelerometer-measurements
https://www.analog.com/media/en/technical-documentation/application-notes/AN-1025.pdf

# Unit Testing

##Prerequisite
1. Ruby 2.5.1 installed
2. ceedling installed
   ```
       Ceedling:: 0.31.1
       CMock:: 2.5.4
       Unity:: 2.5.4
       CException:: 1.3.3
   ```

## How to build test
Open the command line interface (CLI), navigate to the test folder directory, and execute the following command:

ceedling test:all

##Limitations
Only a test case has been included in \ADXL343\test\test directory to illustrate the unit test framework with the getDeviceId function.
![image](https://github.com/BSnehal-C/driver/assets/151748124/4a7e5ecd-77e5-43cb-8670-4b044b79e1c7)
