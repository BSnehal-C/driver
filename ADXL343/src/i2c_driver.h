#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

/**
 * @file i2c_driver.h
 * @brief I2C Driver Module Interface
 *
 * @defgroup hal Hardware Abstraction Layer (HAL)
 * @brief Abstracts hardware specifics through a unified API.
 *
 * This module provides an interface for I2C (Inter-Integrated Circuit) communication. It abstracts the lower-level
 * details of hardware interaction, facilitating communication with I2C peripherals. The module is designed to be
 * portable, efficient, and easy to use, with support for both blocking and non-blocking operations. It includes
 * functions for initializing the I2C bus, reading from and writing to I2C devices, and handling common I2C
 * operations with robust error management.
 *
 * Usage of this module allows for clear, concise, and efficient I2C communication implementations, supporting
 * a wide range of I2C slave devices. It is suitable for projects requiring communication with sensors, memory
 * modules, and other I2C-compatible peripherals.
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
#include "FunctionStatus.h"                     //!< Include to use the generic function status enumeration type

// --------------------------------------------------------------------------------------------------------------------
// Constant and macro definitions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// Type definitions. 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// Function declarations 
// --------------------------------------------------------------------------------------------------------------------


/** -------------------------------------------------------------------------------------------------------------------
 * @brief Writes byte(s) of data to the I2C bus.
 *
 * This function sends a sequence of bytes over the I2C bus. It is designed to transmit both control and data 
 * bytes to an I2C slave device. The data to write should include the device address, register address, and the data 
 * bytes to be written to the device. This function blocks until the transmission is complete or the timeout expires.
 *
 * @param dataToWrite A pointer to a buffer containing the sequence of bytes to be transmitted. This buffer should 
 *                    include the target device's address, the register address within the device, and the data to 
 *                    be written.
 * @param length      The number of bytes to transmit from the dataToWrite buffer.
 * @param timeout     The maximum duration to wait for the transmission to complete, in milliseconds. A timeout 
 *                    value of 0 indicates a non-blocking operation, where the function will return immediately 
 *                    if the bus is busy.
 *
 * @return FunctionStatus  Returns FUNCTION_STATUS_OK if the transmission was successful.
 *                         Returns FUNCTION_STATUS_ERROR for non-specific errors.
 *                         Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
 *                         Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
 * --------------------------------------------------------------------------------------------------------------------
 */
extern FunctionStatus i2c_write(const uint8_t* dataToWrite, size_t length, uint32_t timeout);

/** -------------------------------------------------------------------------------------------------------------------
 * @brief Reads byte(s) of data from the I2C bus.
 *
 * This function reads a sequence of bytes from an I2C slave device into a specified buffer. It is typically used 
 * to read data or status information from an I2C device, with the read operation initiating at the device's current 
 * register pointer or at a specified register address included in a preceding write transaction.
 *
 * @param dataToRead  A pointer to a buffer where the read data will be stored. The caller must ensure that the 
 *                    buffer is large enough to hold the number of bytes specified by the length parameter.
 * @param length      The number of bytes to read into the dataToRead buffer.
 * @param timeout     The maximum duration to wait for the read operation to complete, in milliseconds. A timeout 
 *                    value of 0 indicates a non-blocking operation, where the function will return immediately 
 *                    if the bus is busy.
 *
 * @return FunctionStatus  Returns FUNCTION_STATUS_OK if the read operation was successful.
 *                         Returns FUNCTION_STATUS_ERROR for non-specific errors.
 *                         Returns FUNCTION_STATUS_ARGUMENT_ERROR if null pointers or invalid arguments are passed.
 *                         Returns FUNCTION_STATUS_TIMEOUT if the operation did not complete within the specified timeout period.
 * --------------------------------------------------------------------------------------------------------------------
 */
extern FunctionStatus i2c_read(uint8_t* dataToRead, size_t length, uint32_t timeout);



/** @} */

#endif /* INC_I2C_DRIVER_H_ */
