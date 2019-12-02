/**
 * \addtogroup C28XX_HW Parallel ESC Access
 * @{
 */

/**
\file C28xxsystem.h
\author TI
\brief Defines and Macros to access the ESC for C28x MCU

 */


#ifndef _C28XXSYSTEM_H_
#define _C28XXSYSTEM_H_


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include <string.h>
//#include <malloc.h> //commented for C28x


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

void c28xx_strcpy(void * dst,const void *src,size_t len);
void * c28xx_memcpy(void *s1, const void *s2, size_t n);
int c28xx_memcmp(const void *cs, const void *ct, size_t n);


/** @}*/
#endif
