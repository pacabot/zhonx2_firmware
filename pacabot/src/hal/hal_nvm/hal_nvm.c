/*---------------------------------------------------------------------------
 *
 *      hal_nvm.c
 *
 *---------------------------------------------------------------------------*/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

/* Declarations for this module */
#include "hal/hal_nvm.h"

#include <stdio.h>
#include <string.h>

/* ST Lib declarations */
#include "stm32f4xx_flash.h"


// Just to give something in the open() function
char hal_nvm_handle = 1;


/**
* @brief Initializes the Non-Volatile Memory interfaces.
*
* This function initializes the Non-Volatile Memory interface.
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_init(void)
{
    return HAL_NVM_E_SUCCESS;
}


/**
* @brief Shutdowns the Non-Volatile Memory interfaces.
*
* It must perform all operations required to cleanup after the interface use.
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_terminate(void)
{
    return HAL_NVM_E_SUCCESS;
}


/**
* @brief Opens the Non-Volatile Memory interfaces.
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_open(HAL_NVM_HANDLE *handle, void *params)
{
    UNUSED(params);
    *handle = (HAL_NVM_HANDLE)&hal_nvm_handle;

    return HAL_NVM_E_SUCCESS;
}


/**
* @brief Closes the Non-Volatile Memory interfaces.
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_close(HAL_NVM_HANDLE handle)
{
    return HAL_NVM_E_SUCCESS;
}


/**
* @brief Read from Non-Volatile Memory.
*
* The function reads from Non-Volatile Memory.
*
* @param[in] params    params for the connection.
* @param[out] handle    valid pointer toward a HAL_NVM_HANDLE handle
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_INVAL if one argument is invalid
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_read(HAL_NVM_HANDLE *handle,
                 unsigned char *dest, unsigned char *src, int length)
{
    // Check handle
    if (!handle || !dest || !src || length < 0)
    {
        return HAL_NVM_E_ERROR;
    }

    // Copy data
    memcpy(dest, src, length);

    return HAL_NVM_E_SUCCESS;
}


/**
* @brief Write in Non-Volatile Memory.
*
* The function writes in Non-Volatile Memory.
*
* @param[in] params    params for the connection.
* @param[out] handle    valid pointer toward a HAL_NVM_HANDLE handle
*
* @return #HAL_NVM_E_SUCCESS if the operation is successful,
*         #HAL_NVM_E_INVAL if one argument is invalid
*         #HAL_NVM_E_ERROR otherwise.
*/
int hal_nvm_write(HAL_NVM_HANDLE handle,
                  void *dst, void *src, int length)
{
    (void)handle;
    /* Legacy raw writes are retired; use the transactional fw_store API. */
    (void)dst; (void)src; (void)length;
    return HAL_NVM_E_ERROR;
}

int hal_nvm_init_sector(HAL_NVM_HANDLE handle, unsigned long address)
{
    (void)handle; (void)address;
    return HAL_NVM_E_ERROR;
}
