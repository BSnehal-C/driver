// --------------------------------------------------------------------------------------------------------------------
/// \file  FunctionStatus.h
/// \brief The global definition of the FunctionStatus Enumerator (as funtion return status)
// --------------------------------------------------------------------------------------------------------------------
// 

#ifndef INC_FUNCTIONSTATUS_H_
#define INC_FUNCTIONSTATUS_H_

/**
 * \enum FunctionStatus
 * @{
 */
typedef enum
{
    // General rage
    FUNCTION_STATUS_OK = 0,                             //!< The function execution was successful
    FUNCTION_STATUS_ERROR,                              //!< The function execution caused an unspecific error
    FUNCTION_STATUS_NOT_INITIALIZED,                    //!< The function execution was aborted because the module
                                                        //!< instance is not initialized
    FUNCTION_STATUS_ALREADY_INITIALIZED,                //!< The function execution was aborted because the module
                                                        //!< instance is already initialized
    FUNCTION_STATUS_ARGUMENT_ERROR,                     //!< At least one argument of the function was NULL (invalid)
    FUNCTION_STATUS_BOUNDARY_ERROR,                     //!< At least one argument is out of boundary
    FUNCTION_STATUS_TIMEOUT,                            //!< A timeout occurred during function execution
	
	
	
    
    // Feel free to add customized return status if you need. ;)

} FunctionStatus;

#endif /* INC_FUNCTIONSTATUS_H_ */