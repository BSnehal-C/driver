# About ADXL343

The ADXL343 is a  3-axis accelerometer by Analog Devices. It has with 10- to 13-bit resolution with measurement at up to ±16 g. Digital output data is formatted as 16-bit twos complement and is accessible through either a SPI (3- or 4-wire) or I2C digital interface.

#  ADXL343 Driver
Driver ADXL343 is the driver of adxl343 developed to work in bypass mode with data ready interrupt. It provides acceleration scaled reading in bypass mode of operation and provide functions to initialze required registers.
This driver is scalable to add full functionality of adxl343.

## Limitations:
1. Functions provided to get acceleration scaled reading only in Bypass Mode.
2. Dont Supports readings with in left justied configuration.
3. Functions provided to set and get registers which required only to operate in Bypass Mode with data ready interrupt.
4. Support for SPI communication is not provided.

## Prerequisite
1. The required hardware connections must be established to function ADXL343 in I2C mode.
2. I2C driver must be initialized.

## Folder structure
/src includes ADXL343 source and header files .

/test Includes unit test code with ceeding, cmock supporting files

/doc Includes all reference documents and data sheet of adxl343.

### Folders to be added
/examples includes driver ADXL343 sample application code.


## Reference Documents

https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf
https://electronics.stackexchange.com/questions/545643/g-lsb-meaning-in-accelerometer-measurements
https://www.analog.com/media/en/technical-documentation/application-notes/AN-1025.pdf

