/*******************************************************************************
* File Name: quad_B_pendulo.h  
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

#if !defined(CY_PINS_quad_B_pendulo_H) /* Pins quad_B_pendulo_H */
#define CY_PINS_quad_B_pendulo_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "quad_B_pendulo_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 quad_B_pendulo__PORT == 15 && ((quad_B_pendulo__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    quad_B_pendulo_Write(uint8 value);
void    quad_B_pendulo_SetDriveMode(uint8 mode);
uint8   quad_B_pendulo_ReadDataReg(void);
uint8   quad_B_pendulo_Read(void);
void    quad_B_pendulo_SetInterruptMode(uint16 position, uint16 mode);
uint8   quad_B_pendulo_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the quad_B_pendulo_SetDriveMode() function.
     *  @{
     */
        #define quad_B_pendulo_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define quad_B_pendulo_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define quad_B_pendulo_DM_RES_UP          PIN_DM_RES_UP
        #define quad_B_pendulo_DM_RES_DWN         PIN_DM_RES_DWN
        #define quad_B_pendulo_DM_OD_LO           PIN_DM_OD_LO
        #define quad_B_pendulo_DM_OD_HI           PIN_DM_OD_HI
        #define quad_B_pendulo_DM_STRONG          PIN_DM_STRONG
        #define quad_B_pendulo_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define quad_B_pendulo_MASK               quad_B_pendulo__MASK
#define quad_B_pendulo_SHIFT              quad_B_pendulo__SHIFT
#define quad_B_pendulo_WIDTH              1u

/* Interrupt constants */
#if defined(quad_B_pendulo__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in quad_B_pendulo_SetInterruptMode() function.
     *  @{
     */
        #define quad_B_pendulo_INTR_NONE      (uint16)(0x0000u)
        #define quad_B_pendulo_INTR_RISING    (uint16)(0x0001u)
        #define quad_B_pendulo_INTR_FALLING   (uint16)(0x0002u)
        #define quad_B_pendulo_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define quad_B_pendulo_INTR_MASK      (0x01u) 
#endif /* (quad_B_pendulo__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define quad_B_pendulo_PS                     (* (reg8 *) quad_B_pendulo__PS)
/* Data Register */
#define quad_B_pendulo_DR                     (* (reg8 *) quad_B_pendulo__DR)
/* Port Number */
#define quad_B_pendulo_PRT_NUM                (* (reg8 *) quad_B_pendulo__PRT) 
/* Connect to Analog Globals */                                                  
#define quad_B_pendulo_AG                     (* (reg8 *) quad_B_pendulo__AG)                       
/* Analog MUX bux enable */
#define quad_B_pendulo_AMUX                   (* (reg8 *) quad_B_pendulo__AMUX) 
/* Bidirectional Enable */                                                        
#define quad_B_pendulo_BIE                    (* (reg8 *) quad_B_pendulo__BIE)
/* Bit-mask for Aliased Register Access */
#define quad_B_pendulo_BIT_MASK               (* (reg8 *) quad_B_pendulo__BIT_MASK)
/* Bypass Enable */
#define quad_B_pendulo_BYP                    (* (reg8 *) quad_B_pendulo__BYP)
/* Port wide control signals */                                                   
#define quad_B_pendulo_CTL                    (* (reg8 *) quad_B_pendulo__CTL)
/* Drive Modes */
#define quad_B_pendulo_DM0                    (* (reg8 *) quad_B_pendulo__DM0) 
#define quad_B_pendulo_DM1                    (* (reg8 *) quad_B_pendulo__DM1)
#define quad_B_pendulo_DM2                    (* (reg8 *) quad_B_pendulo__DM2) 
/* Input Buffer Disable Override */
#define quad_B_pendulo_INP_DIS                (* (reg8 *) quad_B_pendulo__INP_DIS)
/* LCD Common or Segment Drive */
#define quad_B_pendulo_LCD_COM_SEG            (* (reg8 *) quad_B_pendulo__LCD_COM_SEG)
/* Enable Segment LCD */
#define quad_B_pendulo_LCD_EN                 (* (reg8 *) quad_B_pendulo__LCD_EN)
/* Slew Rate Control */
#define quad_B_pendulo_SLW                    (* (reg8 *) quad_B_pendulo__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define quad_B_pendulo_PRTDSI__CAPS_SEL       (* (reg8 *) quad_B_pendulo__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define quad_B_pendulo_PRTDSI__DBL_SYNC_IN    (* (reg8 *) quad_B_pendulo__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define quad_B_pendulo_PRTDSI__OE_SEL0        (* (reg8 *) quad_B_pendulo__PRTDSI__OE_SEL0) 
#define quad_B_pendulo_PRTDSI__OE_SEL1        (* (reg8 *) quad_B_pendulo__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define quad_B_pendulo_PRTDSI__OUT_SEL0       (* (reg8 *) quad_B_pendulo__PRTDSI__OUT_SEL0) 
#define quad_B_pendulo_PRTDSI__OUT_SEL1       (* (reg8 *) quad_B_pendulo__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define quad_B_pendulo_PRTDSI__SYNC_OUT       (* (reg8 *) quad_B_pendulo__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(quad_B_pendulo__SIO_CFG)
    #define quad_B_pendulo_SIO_HYST_EN        (* (reg8 *) quad_B_pendulo__SIO_HYST_EN)
    #define quad_B_pendulo_SIO_REG_HIFREQ     (* (reg8 *) quad_B_pendulo__SIO_REG_HIFREQ)
    #define quad_B_pendulo_SIO_CFG            (* (reg8 *) quad_B_pendulo__SIO_CFG)
    #define quad_B_pendulo_SIO_DIFF           (* (reg8 *) quad_B_pendulo__SIO_DIFF)
#endif /* (quad_B_pendulo__SIO_CFG) */

/* Interrupt Registers */
#if defined(quad_B_pendulo__INTSTAT)
    #define quad_B_pendulo_INTSTAT            (* (reg8 *) quad_B_pendulo__INTSTAT)
    #define quad_B_pendulo_SNAP               (* (reg8 *) quad_B_pendulo__SNAP)
    
	#define quad_B_pendulo_0_INTTYPE_REG 		(* (reg8 *) quad_B_pendulo__0__INTTYPE)
#endif /* (quad_B_pendulo__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_quad_B_pendulo_H */


/* [] END OF FILE */
