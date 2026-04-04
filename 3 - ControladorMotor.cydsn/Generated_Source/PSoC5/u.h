/*******************************************************************************
* File Name: u.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_u_H) /* Pins u_H */
#define CY_PINS_u_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "u_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 u__PORT == 15 && ((u__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    u_Write(uint8 value);
void    u_SetDriveMode(uint8 mode);
uint8   u_ReadDataReg(void);
uint8   u_Read(void);
void    u_SetInterruptMode(uint16 position, uint16 mode);
uint8   u_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the u_SetDriveMode() function.
     *  @{
     */
        #define u_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define u_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define u_DM_RES_UP          PIN_DM_RES_UP
        #define u_DM_RES_DWN         PIN_DM_RES_DWN
        #define u_DM_OD_LO           PIN_DM_OD_LO
        #define u_DM_OD_HI           PIN_DM_OD_HI
        #define u_DM_STRONG          PIN_DM_STRONG
        #define u_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define u_MASK               u__MASK
#define u_SHIFT              u__SHIFT
#define u_WIDTH              1u

/* Interrupt constants */
#if defined(u__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in u_SetInterruptMode() function.
     *  @{
     */
        #define u_INTR_NONE      (uint16)(0x0000u)
        #define u_INTR_RISING    (uint16)(0x0001u)
        #define u_INTR_FALLING   (uint16)(0x0002u)
        #define u_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define u_INTR_MASK      (0x01u) 
#endif /* (u__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define u_PS                     (* (reg8 *) u__PS)
/* Data Register */
#define u_DR                     (* (reg8 *) u__DR)
/* Port Number */
#define u_PRT_NUM                (* (reg8 *) u__PRT) 
/* Connect to Analog Globals */                                                  
#define u_AG                     (* (reg8 *) u__AG)                       
/* Analog MUX bux enable */
#define u_AMUX                   (* (reg8 *) u__AMUX) 
/* Bidirectional Enable */                                                        
#define u_BIE                    (* (reg8 *) u__BIE)
/* Bit-mask for Aliased Register Access */
#define u_BIT_MASK               (* (reg8 *) u__BIT_MASK)
/* Bypass Enable */
#define u_BYP                    (* (reg8 *) u__BYP)
/* Port wide control signals */                                                   
#define u_CTL                    (* (reg8 *) u__CTL)
/* Drive Modes */
#define u_DM0                    (* (reg8 *) u__DM0) 
#define u_DM1                    (* (reg8 *) u__DM1)
#define u_DM2                    (* (reg8 *) u__DM2) 
/* Input Buffer Disable Override */
#define u_INP_DIS                (* (reg8 *) u__INP_DIS)
/* LCD Common or Segment Drive */
#define u_LCD_COM_SEG            (* (reg8 *) u__LCD_COM_SEG)
/* Enable Segment LCD */
#define u_LCD_EN                 (* (reg8 *) u__LCD_EN)
/* Slew Rate Control */
#define u_SLW                    (* (reg8 *) u__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define u_PRTDSI__CAPS_SEL       (* (reg8 *) u__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define u_PRTDSI__DBL_SYNC_IN    (* (reg8 *) u__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define u_PRTDSI__OE_SEL0        (* (reg8 *) u__PRTDSI__OE_SEL0) 
#define u_PRTDSI__OE_SEL1        (* (reg8 *) u__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define u_PRTDSI__OUT_SEL0       (* (reg8 *) u__PRTDSI__OUT_SEL0) 
#define u_PRTDSI__OUT_SEL1       (* (reg8 *) u__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define u_PRTDSI__SYNC_OUT       (* (reg8 *) u__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(u__SIO_CFG)
    #define u_SIO_HYST_EN        (* (reg8 *) u__SIO_HYST_EN)
    #define u_SIO_REG_HIFREQ     (* (reg8 *) u__SIO_REG_HIFREQ)
    #define u_SIO_CFG            (* (reg8 *) u__SIO_CFG)
    #define u_SIO_DIFF           (* (reg8 *) u__SIO_DIFF)
#endif /* (u__SIO_CFG) */

/* Interrupt Registers */
#if defined(u__INTSTAT)
    #define u_INTSTAT            (* (reg8 *) u__INTSTAT)
    #define u_SNAP               (* (reg8 *) u__SNAP)
    
	#define u_0_INTTYPE_REG 		(* (reg8 *) u__0__INTTYPE)
#endif /* (u__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_u_H */


/* [] END OF FILE */
