/**
  ******************************************************************************
  * @file    usart.c
  * @author  MCD Application Team
  * @brief   This file provides code for the configuration of the USART
  *          instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <utilities_conf.h>
#include "stm32_lpm.h"

/*private variables*/
static struct
{
  char buffTx[256];                         /* structure have to be simplified*/
  char buffRx[256];
  int rx_idx_free;
  int rx_idx_toread;
  __IO HAL_UART_StateTypeDef gState;
  __IO HAL_UART_StateTypeDef RxState;
} uart_context;

/* private function */
static void receive(char rx);

/******************************************************************************
  * @brief Handler on Rx IRQ
  * @param handle to the UART
  * @retval void
  ******************************************************************************/
void HW_UART_Modem_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its = READ_REG(huart->Instance->CR3);;
  uint32_t errorflags;
  int rx_ready = 0;



  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if (((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
	//__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_WUF);
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);

    /* forbid stop mode */
    UTIL_LPM_SetStopMode(LPM_UART_RX_Id, UTIL_LPM_DISABLE);


    /* Enable the UART Data Register not empty Interrupts */
    SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);

    /* Set the UART state ready to be able to start again the process */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;

  }


  /* UART in mode Receiver ---------------------------------------------------*/
  if (((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
  {
    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
      /*RXNE flag is auto cleared by reading the data*/
      *huart->pRxBuffPtr++ = (uint8_t)READ_REG(huart->Instance->RDR);

      /* allow stop mode*/
      UTIL_LPM_SetStopMode(LPM_UART_RX_Id, UTIL_LPM_ENABLE);

      if (--huart->RxXferCount == 0U)
      {
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
        huart->RxState = HAL_UART_STATE_READY;
        rx_ready = 1;  /* not used RxTC callback*/
      }
    }
    else
    {
      /* Clear RXNE interrupt flag */
      __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
      return;
    }
  }

  /* If error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags != RESET)
  {
    /* Error on receiving */
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    /*   *((huart->pRxBuffPtr)-1) = 0x01;   */        /*we skip the overrun case*/
    rx_ready = 0;                                     // Skipping the data if error flags are set
  }

  if (rx_ready)
  {
    /*character in the ring buffer*/
    receive(*((huart->pRxBuffPtr) - 1));
  }
}


/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	receive(*((UartHandle->pRxBuffPtr) - 1));
}*/

/******************************************************************************
  * @brief To check if data has been received
  * @param none
  * @retval Reset no data / set data
  ******************************************************************************/
FlagStatus HW_UART_Modem_IsNewCharReceived(void)
{
  FlagStatus status;


  uint32_t primask_bit = __get_PRIMASK();

  __disable_irq();

  status = ((uart_context.rx_idx_toread == uart_context.rx_idx_free) ? RESET : SET);


  __set_PRIMASK(primask_bit);
  return status;
}




/******************************************************************************
  * @brief Get the received character
  * @param none
  * @retval Return the data received
  ******************************************************************************/
uint8_t HW_UART_Modem_GetNewChar(void)
{
  uint8_t NewChar;
  uint8_t isfound=0;

  uint32_t primask_bit = __get_PRIMASK();

  __disable_irq();

  NewChar = uart_context.buffRx[uart_context.rx_idx_toread];
  uart_context.rx_idx_toread = (uart_context.rx_idx_toread + 1) % sizeof(uart_context.buffRx);
  if(NewChar == (uint8_t)'+')
  {
	  isfound++;
  }

  __set_PRIMASK(primask_bit);
  return NewChar;
}



/******************************************************************************
  * @brief Store in ring buffer the received character
  * @param none
  * @retval none
  ******************************************************************************/
static void receive(char rx)
{
  int next_free;


	/** no need to clear the RXNE flag because it is auto cleared by reading the data*/
    if(rx != '\0')   // Removing Null characters before it finds its way into the data.
    {
		uart_context.buffRx[uart_context.rx_idx_free] = rx;
		next_free = (uart_context.rx_idx_free + 1) % sizeof(uart_context.buffRx);
		if (next_free != uart_context.rx_idx_toread)
		{
		/* this is ok to read as there is no buffer overflow in input */
			uart_context.rx_idx_free = next_free;
		}
    }

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
