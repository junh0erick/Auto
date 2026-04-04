/*******************************************************************************
* File Name: esfuerzo.h  
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

#if !defined(CY_PINS_esfuerzo_H) /* Pins esfuerzo_H */
#define CY_PINS_esfuerzo_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "esfuerzo_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 esfuerzo__PORT == 15 && ((esfuerzo__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    esfuerzo_Write(uint8 value);
void    esfuerzo_SetDriveMode(uint8 mode);
uint8   esfuerzo_ReadDataReg(void);
uint8   esfuerzo_Read(void);
void    esfuerzo_SetInterruptMode(uint16 position, uint16 mode);
uint8   esfuerzo_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the esfuerzo_SetDriveMode() function.
     *  @{
     */
        #define esfuerzo_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define esfuerzo_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define esfuerzo_DM_RES_UP          PIN_DM_RES_UP
        #define esfuerzo_DM_RES_DWN         PIN_DM_RES_DWN
        #define esfuerzo_DM_OD_LO           PIN_DM_OD_LO
        #define esfuerzo_DM_OD_HI           PIN_DM_OD_HI
        #define esfuerzo_DM_STRONG          PIN_DM_STRONG
        #define esfuerzo_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define esfuerzo_MASK               esfuerzo__MASK
#define esfuerzo_SHIFT              esfuerzo__SHIFT
#define esfuerzo_WIDTH              1u

/* Interrupt constants */
#if defined(esfuerzo__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in esfuerzo_SetInterruptMode() function.
     *  @{
     */
        #define esfuerzo_INTR_NONE      (uint16)(0x0000u)
        #define esfuerzo_INTR_RISING    (uint16)(0x0001u)
        #define esfuerzo_INTR_FALLING   (uint16)(0x0002u)
        #define esfuerzo_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define esfuerzo_INTR_MASK      (0x01u) 
#endif /* (esfuerzo__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define esfuerzo_PS                     (* (reg8 *) esfuerzo__PS)
/* Data Register */
#define esfuerzo_DR                     (* (reg8 *) esfuerzo__DR)
/* Port Number */
#define esfuerzo_PRT_NUM                (* (reg8 *) esfuerzo__PRT) 
/* Connect to Analog Globals */                                                  
#define esfuerzo_AG                     (* (reg8 *) esfuerzo__AG)                       
/* Analog MUX bux enable */
#define esfuerzo_AMUX                   (* (reg8 *) esfuerzo__AMUX) 
/* Bidirectional Enable */                                                        
#define esfuerzo_BIE                    (* (reg8 *) esfuerzo__BIE)
/* Bit-mask for Aliased Register Access */
#define esfuerzo_BIT_MASK               (* (reg8 *) esfuerzo__BIT_MASK)
/* Bypass Enable */
#define esfuerzo_BYP                    (* (reg8 *) esfuerzo__BYP)
/* Port wide control signals */                                                   
#define esfuerzo_CTL                    (* (reg8 *) esfuerzo__CTL)
/* Drive Modes */
#define esfuerzo_DM0                    (* (reg8 *) esfuerzo__DM0) 
#define esfuerzo_DM1                    (* (reg8 *) esfuerzo__DM1)
#define esfuerzo_DM2                    (* (reg8 *) esfuerzo__DM2) 
/* Input Buffer Disable Override */
#define esfuerzo_INP_DIS                (* (reg8 *) esfuerzo__INP_DIS)
/* LCD Common or Segment Drive */
#define esfuerzo_LCD_COM_SEG            (* (reg8 *) esfuerzo__LCD_COM_SEG)
/* Enable Segment LCD */
#define esfuerzo_LCD_EN                 (* (reg8 *) esfuerzo__LCD_EN)
/* Slew Rate Control */
#define esfuerzo_SLW                    (* (reg8 *) esfuerzo__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define esfuerzo_PRTDSI__CAPS_SEL       (* (reg8 *) esfuerzo__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define esfuerzo_PRTDSI__DBL_SYNC_IN    (* (reg8 *) esfuerzo__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define esfuerzo_PRTDSI__OE_SEL0        (* (reg8 *) esfuerzo__PRTDSI__OE_SEL0) 
#define esfuerzo_PRTDSI__OE_SEL1        (* (reg8 *) esfuerzo__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define esfuerzo_PRTDSI__OUT_SEL0       (* (reg8 *) esfuerzo__PRTDSI__OUT_SEL0) 
#define esfuerzo_PRTDSI__OUT_SEL1       (* (reg8 *) esfuerzo__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define esfuerzo_PRTDSI__SYNC_OUT       (* (reg8 *) esfuerzo__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(esfuerzo__SIO_CFG)
    #define esfuerzo_SIO_HYST_EN        (* (reg8 *) esfuerzo__SIO_HYST_EN)
    #define esfuerzo_SIO_REG_HIFREQ     (* (reg8 *) esfuerzo__SIO_REG_HIFREQ)
    #define esfuerzo_SIO_CFG            (* (reg8 *) esfuerzo__SIO_CFG)
    #define esfuerzo_SIO_DIFF           (* (reg8 *) esfuerzo__SIO_DIFF)
#endif /* (esfuerzo__SIO_CFG) */

/* Interrupt Registers */
#if defined(esfuerzo__INTSTAT)
    #define esfuerzo_INTSTAT            (* (reg8 *) esfuerzo__INTSTAT)
    #define esfuerzo_SNAP               (* (reg8 *) esfuerzo__SNAP)
    
	#define esfuerzo_0_INTTYPE_REG 		(* (reg8 *) esfuerzo__0__INTTYPE)
#endif /* (esfuerzo__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_esfuerzo_H */


/* [] END OF FILE */
