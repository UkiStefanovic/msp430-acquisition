/**
 * @file    hal_7seg.h
 * @brief   Declaration of function used to write to 7seg display
 *
 * @date    2021
 * @author  Haris Turkmanovic (haris@etf.rs)
 */
#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <stdint.h>

/*Displays position inside of register*/
#define HAL_7SEG_DISPLAY_2_MASK          0x10
#define HAL_7SEG_DISPLAY_1_MASK          0x01
/*Segments position inside of registers*/
#define HAL_7SEG_SEGMENT_A_MASK          0x80
#define HAL_7SEG_SEGMENT_B_MASK          0x08
#define HAL_7SEG_SEGMENT_C_MASK          0x40
#define HAL_7SEG_SEGMENT_D_MASK          0x02
#define HAL_7SEG_SEGMENT_E_MASK          0x08
#define HAL_7SEG_SEGMENT_F_MASK          0x01
#define HAL_7SEG_SEGMENT_G_MASK          0x04

/*Displays  direction registers*/
#define HAL_7SEG_DISPLAY_1_DIR          P7DIR
#define HAL_7SEG_DISPLAY_2_DIR          P6DIR
/*Segments direction registers*/
#define HAL_7SEG_SEGMENT_A_DIR          P3DIR
#define HAL_7SEG_SEGMENT_B_DIR          P4DIR
#define HAL_7SEG_SEGMENT_C_DIR          P2DIR
#define HAL_7SEG_SEGMENT_D_DIR          P8DIR
#define HAL_7SEG_SEGMENT_E_DIR          P2DIR
#define HAL_7SEG_SEGMENT_F_DIR          P4DIR
#define HAL_7SEG_SEGMENT_G_DIR          P8DIR


/*Displays  direction registers*/
#define HAL_7SEG_DISPLAY_1_OUT          P7OUT
#define HAL_7SEG_DISPLAY_2_OUT          P6OUT
/*Segments output registers*/
#define HAL_7SEG_SEGMENT_A_OUT          P3OUT
#define HAL_7SEG_SEGMENT_B_OUT          P4OUT
#define HAL_7SEG_SEGMENT_C_OUT          P2OUT
#define HAL_7SEG_SEGMENT_D_OUT          P8OUT
#define HAL_7SEG_SEGMENT_E_OUT          P2OUT
#define HAL_7SEG_SEGMENT_F_OUT          P4OUT
#define HAL_7SEG_SEGMENT_G_OUT          P8OUT

#define HAL_7SEG_SEGMENT_A_ON  HAL_7SEG_SEGMENT_A_OUT &=~ HAL_7SEG_SEGMENT_A_MASK
#define HAL_7SEG_SEGMENT_B_ON  HAL_7SEG_SEGMENT_B_OUT &=~ HAL_7SEG_SEGMENT_B_MASK
#define HAL_7SEG_SEGMENT_C_ON  HAL_7SEG_SEGMENT_C_OUT &=~ HAL_7SEG_SEGMENT_C_MASK
#define HAL_7SEG_SEGMENT_D_ON  HAL_7SEG_SEGMENT_D_OUT &=~ HAL_7SEG_SEGMENT_D_MASK
#define HAL_7SEG_SEGMENT_E_ON  HAL_7SEG_SEGMENT_E_OUT &=~ HAL_7SEG_SEGMENT_E_MASK
#define HAL_7SEG_SEGMENT_F_ON  HAL_7SEG_SEGMENT_F_OUT &=~ HAL_7SEG_SEGMENT_F_MASK
#define HAL_7SEG_SEGMENT_G_ON  HAL_7SEG_SEGMENT_G_OUT &=~ HAL_7SEG_SEGMENT_G_MASK


#define HAL_7SEG_SEGMENT_A_OFF  HAL_7SEG_SEGMENT_A_OUT |= HAL_7SEG_SEGMENT_A_MASK
#define HAL_7SEG_SEGMENT_B_OFF  HAL_7SEG_SEGMENT_B_OUT |= HAL_7SEG_SEGMENT_B_MASK
#define HAL_7SEG_SEGMENT_C_OFF  HAL_7SEG_SEGMENT_C_OUT |= HAL_7SEG_SEGMENT_C_MASK
#define HAL_7SEG_SEGMENT_D_OFF  HAL_7SEG_SEGMENT_D_OUT |= HAL_7SEG_SEGMENT_D_MASK
#define HAL_7SEG_SEGMENT_E_OFF  HAL_7SEG_SEGMENT_E_OUT |= HAL_7SEG_SEGMENT_E_MASK
#define HAL_7SEG_SEGMENT_F_OFF  HAL_7SEG_SEGMENT_F_OUT |= HAL_7SEG_SEGMENT_F_MASK
#define HAL_7SEG_SEGMENT_G_OFF  HAL_7SEG_SEGMENT_G_OUT |= HAL_7SEG_SEGMENT_G_MASK

#define HAL_7SEG_DISPLAY_1_ON   HAL_7SEG_DISPLAY_1_OUT &=~  HAL_7SEG_DISPLAY_1_MASK
#define HAL_7SEG_DISPLAY_2_ON   HAL_7SEG_DISPLAY_2_OUT &=~  HAL_7SEG_DISPLAY_2_MASK

#define HAL_7SEG_DISPLAY_1_OFF  HAL_7SEG_DISPLAY_1_OUT |=  HAL_7SEG_DISPLAY_1_MASK
#define HAL_7SEG_DISPLAY_2_OFF  HAL_7SEG_DISPLAY_2_OUT |=  HAL_7SEG_DISPLAY_2_MASK



/**
 * @brief Initialize all functionalities related to the 7seg display
 */
void HAL_7Seg_Init();

/**
 * @brief   Function used to write to 7seg display
 * @param   digit - value 0-9 to be displayed
 * @return  0       - if digit is successfully written to display
 * @return  1       - if digit isn't successfully written to display
 * This function write digit to the previously enabled display. Display can be enabled
 * by using macros
 *
 */
uint8_t HAL_7Seg_WriteDigit(uint8_t digit);

#endif /* FUNCTION_H_ */
