/**
\addtogroup C28XX_HW Parallel ESC Access
@{
*/

/**
\file c28xxsystem.c
\author TI
\brief Implementation
This file contains the interface to the ESC via MCI
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include <ethercat/ecat_def.h>
#include <ethercat/ecatappl.h>
#include <ethercat/ecatslv.h>
#include "c28xxsystem.h"


/*--------------------------------------------------------------------------------------
------
------    local Types and Defines
------
--------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    local functions
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    functions
------
-----------------------------------------------------------------------------------------*/



/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     void

 \brief    This function is used to handle strings, string values (ASCII 8 format) are stored
             as 16 bit words on C28x MCU. The string values (obj entry and names 0x1008, 0x1009,
             0x100A) needs to be transmitted as 8=bit ASCII to etherCAT master.

             below function takes care of the values being transmitted properly

*////////////////////////////////////////////////////////////////////////////////////////

void c28xx_strcpy(void * dst,const void *src,size_t len)
{
    unsigned int i= 0 ;
    unsigned int j = 0;

    unsigned int *dst1, *src1;
    dst1 = (unsigned int*)dst;
    src1 = (unsigned int*)src;

    for(i = 0; i < len;i+=2)
    {
        dst1[j] =  src1[i+1]<<8 | src1[i];
        j++;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     void

 \brief    This function is used to implement memcpy for C28x. the incoming size is in no.
             of bytes from the EtherCAT stack. So the function adjusts the size to no. of
             16 bit words.
*////////////////////////////////////////////////////////////////////////////////////////

void * c28xx_memcpy(void *s1, const void *s2, size_t n)
{
    n = n >> 1; //divide by 2 as the size is coming in no.of bytes
    return memcpy(s1, s2, n);
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     void

 \brief    This function is used to implement memcpy for C28x. the incoming size is in no.
             of bytes from the EtherCAT stack. So the function adjusts the size to no. of
             16 bit words.
*////////////////////////////////////////////////////////////////////////////////////////

int c28xx_memcmp(const void *cs, const void *ct, size_t n)
{
    n = n >> 1;
    return memcmp(cs, ct, n);
}

/** @} */
