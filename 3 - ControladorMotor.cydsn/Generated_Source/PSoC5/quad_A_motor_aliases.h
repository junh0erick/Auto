/*******************************************************************************
* File Name: quad_A_motor.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_quad_A_motor_ALIASES_H) /* Pins quad_A_motor_ALIASES_H */
#define CY_PINS_quad_A_motor_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define quad_A_motor_0			(quad_A_motor__0__PC)
#define quad_A_motor_0_INTR	((uint16)((uint16)0x0001u << quad_A_motor__0__SHIFT))

#define quad_A_motor_INTR_ALL	 ((uint16)(quad_A_motor_0_INTR))

#endif /* End Pins quad_A_motor_ALIASES_H */


/* [] END OF FILE */
