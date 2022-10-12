/*******************************************************************************
 * Copyright (c) 2021 Renesas Electronics Corporation
 * All Rights Reserved.
 *
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    hal_arduino.cpp
 * @brief   zmod4xxx hardware abstraction layer for Arduino platform
 * @version 2.4.2
 * @author Renesas Electronics Corporation
 */

#include "stm32g0xx.h"
#include "st_i2c.h"
#include "zmod4xxx_types.h"
#include "i2c.h"

#define BSP_MAX_DELAY      0xFFFFFFFFU

static BSP_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static BSP_StatusTypeDef I2C_RequestMemoryWrite(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static BSP_StatusTypeDef I2C_RequestMemoryRead(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static BSP_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart);
static void I2C_Flush_TXDR(I2C_TypeDef *I2Cx);
static void I2C_TransferConfig(I2C_TypeDef *I2Cx,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);
static BSP_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart);
static BSP_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart);
static BSP_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart);

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  I2Cx I2C Instance.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval BSP status
  */
BSP_StatusTypeDef BSP_I2C_Mem_Write(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	  uint8_t                    *pBuffPtr;      /*!< Pointer to I2C transfer buffer            */

    uint16_t                   XferSize;       /*!< I2C transfer size                         */

    __IO uint16_t              XferCount;      /*!< I2C transfer counter                      */

    uint32_t tickstart = 0U;

    if((pData == NULL) || (Size == 0U))
    {
      return  BSP_ERROR;
    }

    /* Init tickstart for timeout management*/
    tickstart = BSP_GetTick(); //

    if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != BSP_OK)
    {
      return BSP_TIMEOUT;
    }

    /* Prepare transfer parameters */
    pBuffPtr  = pData;
    XferCount = Size;

    /* Send Slave Address and Memory Address */
    if(I2C_RequestMemoryWrite(I2Cx, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != BSP_OK)
    {
        return BSP_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if(XferCount > MAX_NBYTE_SIZE)
    {
      XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
    }
    else
    {
      XferSize = XferCount;
      I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
    }

    do
    {
      /* Wait until TXIS flag is set */
      if(I2C_WaitOnTXISFlagUntilTimeout(I2Cx, Timeout, tickstart) != BSP_OK)
      {
          return BSP_ERROR;
      }

      /* Write data to TXDR */
      I2Cx->TXDR = (*pBuffPtr++);
      XferCount--;
      XferSize--;

      if((XferSize == 0U) && (XferCount!=0U))
      {
        /* Wait until TCR flag is set */
        if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_TCR, RESET, Timeout, tickstart) != BSP_OK)
        {
          return BSP_TIMEOUT;
        }

        if(XferCount > MAX_NBYTE_SIZE)
        {
          XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          XferSize = XferCount;
          I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }

    }while(XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if(I2C_WaitOnSTOPFlagUntilTimeout(I2Cx, Timeout, tickstart) != BSP_OK)
    {
        return BSP_ERROR;
    }

    /* Clear STOP Flag */
    __I2C_CLEAR_FLAG(I2Cx, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(I2Cx);

    return BSP_OK;
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  I2Cx I2C Instance.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
BSP_StatusTypeDef BSP_I2C_Mem_Read(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	 uint8_t                    *pBuffPtr;      /*!< Pointer to I2C transfer buffer            */

   uint16_t                   XferSize;       /*!< I2C transfer size                         */

   __IO uint16_t              XferCount;      /*!< I2C transfer counter                      */

	 uint32_t tickstart = 0U;

	  if((pData == NULL) || (Size == 0U))
    {
      return  BSP_ERROR;
    }

	 /* Init tickstart for timeout management*/
    tickstart = BSP_GetTick();
		if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, SET,I2C_TIMEOUT_BUSY, tickstart) != BSP_OK)
    {
      return BSP_TIMEOUT;
    }

		/* Prepare transfer parameters */
    pBuffPtr  = pData;
    XferCount = Size;

		/* Send Slave Address and Memory Address */
		if(I2C_RequestMemoryRead(I2Cx, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != BSP_OK)
    {
        return BSP_ERROR;
    }

		/* Send Slave Address */
    /* Set NBYTES to write and reload if XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if(XferCount > MAX_NBYTE_SIZE)
    {
      XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      XferSize = XferCount;
      I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

		do
    {
      /* Wait until RXNE flag is set */
      if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_RXNE, RESET, Timeout, tickstart) != BSP_OK)
      {
        return BSP_TIMEOUT;
      }

      /* Read data from RXDR */
      (*pBuffPtr++) = I2Cx->RXDR;
      XferSize--;
      XferCount--;

      if((XferSize == 0U) && (XferCount != 0U))
      {
        /* Wait until TCR flag is set */
        if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_TCR, RESET, Timeout, tickstart) != BSP_OK)
        {
          return BSP_TIMEOUT;
        }

        if(XferCount > MAX_NBYTE_SIZE)
        {
          XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          XferSize = XferCount;
          I2C_TransferConfig(I2Cx, DevAddress, XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }while( XferCount > 0U);

		 /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if(I2C_WaitOnSTOPFlagUntilTimeout(I2Cx, Timeout, tickstart) != BSP_OK)
    {
        return BSP_TIMEOUT;
    }

    /* Clear STOP Flag */
    __I2C_CLEAR_FLAG(I2Cx, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(I2Cx);

    return BSP_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  I2Cx I2C Instance.
  * @param  Flag Specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  while(__I2C_GET_FLAG(I2Cx, Flag) == Status)
  {
    /* Check for the Timeout */
    if(Timeout != BSP_MAX_DELAY)
    {
      if((Timeout == 0U)||((BSP_GetTick() - Tickstart ) > Timeout))
      {
        return BSP_TIMEOUT;
      }
    }
  }
  return BSP_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of TXIS flag.
  * @param  I2Cx I2C Instance.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart)
{
  while(__I2C_GET_FLAG(I2Cx, I2C_FLAG_TXIS) == RESET)
  {
    /* Check if a NACK is detected */
    if(I2C_IsAcknowledgeFailed(I2Cx, Timeout, Tickstart) != BSP_OK)
    {
      return BSP_ERROR;
    }

    /* Check for the Timeout */
    if(Timeout != BSP_MAX_DELAY)
    {
      if((Timeout == 0U)||((BSP_GetTick() - Tickstart) > Timeout))
      {
        return BSP_TIMEOUT;
      }
    }
  }
  return BSP_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP flag.
  * @param  I2Cx I2C Instance.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart)
{
  while(__I2C_GET_FLAG(I2Cx, I2C_FLAG_STOPF) == RESET)
  {
    /* Check if a NACK is detected */
    if(I2C_IsAcknowledgeFailed(I2Cx, Timeout, Tickstart) != BSP_OK)
    {
      return BSP_ERROR;
    }

    /* Check for the Timeout */
    if((Timeout == 0U)||((BSP_GetTick() - Tickstart) > Timeout))
    {
      return BSP_TIMEOUT;
    }
  }
  return BSP_OK;
}

/**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  I2Cx I2C Instance.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_TypeDef *I2Cx, uint32_t Timeout, uint32_t Tickstart)
{
  if(__I2C_GET_FLAG(I2Cx, I2C_FLAG_AF) == SET)
  {
    /* Wait until STOP Flag is reset */
    /* AutoEnd should be initiate after AF */
    while(__I2C_GET_FLAG(I2Cx, I2C_FLAG_STOPF) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != BSP_MAX_DELAY)
      {
      if((Timeout == 0U)||((BSP_GetTick() - Tickstart) > Timeout))
        {
          return BSP_TIMEOUT;
        }
      }
    }

    /* Clear NACKF Flag */
    __I2C_CLEAR_FLAG(I2Cx, I2C_FLAG_AF);

    /* Clear STOP Flag */
    __I2C_CLEAR_FLAG(I2Cx, I2C_FLAG_STOPF);

    /* Flush TX register */
    I2C_Flush_TXDR(I2Cx);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(I2Cx);

    return BSP_ERROR;
  }
  return BSP_OK;
}

/**
  * @brief  I2C Tx data register flush process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_Flush_TXDR(I2C_TypeDef *I2Cx)
{
  /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
  if(__I2C_GET_FLAG(I2Cx, I2C_FLAG_TXIS) != RESET)
  {
     I2Cx->TXDR = 0x00U;
  }

  /* Flush TX register if not empty */
  if(__I2C_GET_FLAG(I2Cx, I2C_FLAG_TXE) == RESET)
  {
    __I2C_CLEAR_FLAG(I2Cx, I2C_FLAG_TXE);
  }
}

/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  I2Cx I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  * @param  Size Specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_RELOAD_MODE Enable Reload mode .
  *     @arg @ref I2C_AUTOEND_MODE Enable Automatic end mode.
  *     @arg @ref I2C_SOFTEND_MODE Enable Software end mode.
  * @param  Request New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref I2C_GENERATE_STOP Generate stop condition (Size should be set to 0).
  *     @arg @ref I2C_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref I2C_GENERATE_START_WRITE Generate Restart for write request.
  * @retval None
  */
static void I2C_TransferConfig(I2C_TypeDef *I2Cx,  uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
  uint32_t tmpreg = 0U;


  /* Get the CR2 register value */
  tmpreg = I2Cx->CR2;

  /* clear tmpreg specific bits */
  tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));

  /* update tmpreg */
  tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16 ) & I2C_CR2_NBYTES) | \
            (uint32_t)Mode | (uint32_t)Request);

  /* update CR2 register */
  I2Cx->CR2 = tmpreg;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  I2Cx I2C Instance.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_RequestMemoryWrite(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  I2C_TransferConfig(I2Cx,DevAddress,MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if(I2C_WaitOnTXISFlagUntilTimeout(I2Cx, Timeout, Tickstart) != BSP_OK)
  {
      return BSP_TIMEOUT;
  }

  /* If Memory address size is 8Bit */
  if(MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    I2Cx->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    I2Cx->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if(I2C_WaitOnTXISFlagUntilTimeout(I2Cx, Timeout, Tickstart) != BSP_OK)
    {
        return BSP_TIMEOUT;
    }

    /* Send LSB of Memory Address */
    I2Cx->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TCR flag is set */
  if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_TCR, RESET, Timeout, Tickstart) != BSP_OK)
  {
    return BSP_TIMEOUT;
  }

return BSP_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  I2Cx I2C Instance.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval BSP status
  */
static BSP_StatusTypeDef I2C_RequestMemoryRead(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  I2C_TransferConfig(I2Cx,DevAddress,MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if(I2C_WaitOnTXISFlagUntilTimeout(I2Cx, Timeout, Tickstart) != BSP_OK)
  {
    return BSP_ERROR;

  }

  /* If Memory address size is 8Bit */
  if(MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
   I2Cx->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    I2Cx->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if(I2C_WaitOnTXISFlagUntilTimeout(I2Cx, Timeout, Tickstart) != BSP_OK)
    {
        return BSP_ERROR;
    }

    /* Send LSB of Memory Address */
   I2Cx->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TC flag is set */
  if(I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_TC, RESET, Timeout, Tickstart) != BSP_OK)
  {
    return BSP_TIMEOUT;
  }

  return BSP_OK;
}

uint8_t i2c_read_len(I2C_TypeDef *I2Cx,  uint8_t Addr , uint8_t Reg, uint8_t len,uint8_t *buf)
{

	/* 1.保证I2C外设不在使用中. */
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx));

	    /* Check if the I2C is already enabled */
    if ((I2Cx->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable I2C peripheral */
      LL_I2C_Enable(I2Cx);
    }

    /* Disable Pos */
    CLEAR_BIT(I2C1->CR1, I2C_CR1_PE_Pos);

		/* Enable Acknowledge */
    CLEAR_BIT(I2Cx->CR1, I2C_CR1_NACKIE);

  /* 2.发送START信号 */
    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx));

  /* 2.写器件地址 */
    LL_I2C_TransmitData8(I2Cx, Addr);
		  /* Wait until ADDR flag is set */
   while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)){};

    /* Clear ADDR flag */
		 LL_I2C_ClearFlag_ADDR(I2Cx);

		/* Wait until TXE flag is set */
    while(!LL_I2C_IsActiveFlag_TXE(I2Cx));

    /* 4.发送器件寄存器地址. */
    LL_I2C_TransmitData8(I2Cx, Reg);
    while(!LL_I2C_IsActiveFlag_TXE(I2Cx));

  /* 5.提供RESTART信号. */
   // LL_I2C_GenerateStopCondition(I2Cx);
    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx));

  /* 6.重新发送地址,并附带读标记. */
    LL_I2C_TransmitData8(I2Cx, Addr | 0x01);
		/* Wait until ADDR flag is set */
		while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)){};
   //



    if (len  == 0U)
    {
      /* Clear ADDR flag */
      LL_I2C_ClearFlag_ADDR(I2Cx);

      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);
    }
    else if (len == 1U)
    {
      /* Disable Acknowledge */
    SET_BIT(I2Cx->CR1, I2C_CR1_NACKIE);

      /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
         software sequence must complete before the current byte end of transfer */
      __disable_irq();

      /* Clear ADDR flag */
      LL_I2C_ClearFlag_ADDR(I2Cx);

      /* Generate Stop */
      LL_I2C_GenerateStopCondition(I2Cx);

      /* Re-enable IRQs */
      __enable_irq();
    }
    else if (len == 2U)
    {
      /* Enable Pos */
      SET_BIT(I2Cx->CR1, I2C_CR1_PE_Pos);

      /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
         software sequence must complete before the current byte end of transfer */
      __disable_irq();

      /* Clear ADDR flag */
      LL_I2C_ClearFlag_ADDR(I2Cx);

      /* Disable Acknowledge */
      CLEAR_BIT(I2Cx->CR1, I2C_CR1_NACKIE);

      /* Re-enable IRQs */
      __enable_irq();
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(I2Cx->CR1, I2C_CR1_NACKIE);
      /* Clear ADDR flag */
      LL_I2C_ClearFlag_ADDR(I2Cx);
    }

    while (len > 0U)
    {
      if (len <= 3U)
      {
        /* One byte */
        if (len == 1U)
        {
          /* Wait until RXNE flag is set */
					while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};


          /* Read data from DR */
         *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
          buf++;

          /* Update counter */
          len--;
        }
        /* Two bytes */
        else if (len == 2U)
        {
          /* Wait until BTF flag is set */
        	while(!LL_I2C_IsActiveFlag_BTF(I2Cx)){}

          /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
             software sequence must complete before the current byte end of transfer */
          __disable_irq();

          /* Generate Stop */
           LL_I2C_GenerateStopCondition(I2Cx);


          /* Read data from DR */
           *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
          buf++;

          /* Update counter */
          len--;

          /* Re-enable IRQs */
          __enable_irq();

          /* Read data from DR */
          *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
          buf++;

          /* Update counter */
          len--;
        }
        /* 3 Last bytes */
        else
        {
          /* Wait until BTF flag is set */
           while(!LL_I2C_IsActiveFlag_BTF(I2Cx)){}


          /* Disable Acknowledge */
          CLEAR_BIT(I2Cx->CR1, I2C_CR1_NACKIE);

          /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
             software sequence must complete before the current byte end of transfer */
          __disable_irq();

          /* Read data from DR */
           *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
            buf++;

          /* Update counter */
          len--;

          /* Wait until BTF flag is set */
					while(!LL_I2C_IsActiveFlag_BTF(I2Cx)){}

          /* Generate Stop */
           LL_I2C_GenerateStopCondition(I2Cx);

          /* Read data from DR */
            *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
           buf++;

          /* Update counter */
					len--;


          /* Re-enable IRQs */
          __enable_irq();

          /* Read data from DR */
           *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);


          /* Increment Buffer pointer */
           buf++;

          /* Update counter */
         	len--;
        }
      }
      else
      {
        /* Wait until RXNE flag is set */
        while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};

        /* Read data from DR */
            *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
           buf++;

         /* Update counter */
					len--;


        if (LL_I2C_IsActiveFlag_BTF(I2Cx))
        {
            /* Read data from DR */
            *buf  = (uint8_t)LL_I2C_ReceiveData8(I2Cx);

          /* Increment Buffer pointer */
           buf++;

         /* Update counter */
					len--;
        }
      }
    }

  return 0;
}


uint8_t i2c_write_len(I2C_TypeDef *I2Cx,  uint8_t addr , uint8_t reg, uint8_t len,uint8_t *buf)
{
    int i=0;
  /* 1.保证I2C外设不在使用中. */
    while(LL_I2C_IsActiveFlag_BUSY(I2Cx));


    /* Check if the I2C is already enabled */
    if ((I2Cx->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable I2C peripheral */
      LL_I2C_Enable(I2Cx);
    }

    /* Disable Pos */
    CLEAR_BIT(I2Cx->CR1, I2C_CR1_PE_Pos);


  /* 2.发送START信号 */
    LL_I2C_GenerateStartCondition(I2Cx);
    while(!LL_I2C_IsActiveFlag_SB(I2Cx));

  /* 2.写器件地址 */
    LL_I2C_TransmitData8(I2Cx, addr);
		LL_mDelay(1);
    while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));

    LL_I2C_ClearFlag_ADDR(I2Cx);

  /* 3.地址位已经置位,通常TXE也会完成,为了谨慎,再查一下. */
		while(!LL_I2C_IsActiveFlag_TXE(I2Cx)){}

  /* 4.发送器件寄存器地址. */
    LL_I2C_TransmitData8(I2Cx, reg);

		i = len;
    while(i>0)
    {
			while(!LL_I2C_IsActiveFlag_TXE(I2Cx)){};
      /* 5.写入寄存器内容 */
      LL_I2C_TransmitData8(I2Cx, *buf);
      buf++;
			i--;

			if(LL_I2C_IsActiveFlag_BTF(I2Cx)==SET && i!=0)
			{
				  LL_I2C_TransmitData8(I2Cx, *buf);
					buf++;
					i--;

			}
    }

		while(!LL_I2C_IsActiveFlag_BTF(I2Cx)){}


  /* 6.传送结束条件. */
    LL_I2C_GenerateStopCondition(I2Cx);


    return 0;

}



int8_t I2C_write_reg_8bit(uint8_t SlaveAddr_IC, uint8_t addr_reg, uint8_t value)
{

	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2C1)==SET){
		counter++;
		if( counter == 25000 ){//aproximate 150ms
			Error_Handler();
			return ERROR;
		}
	}

	LL_I2C_HandleTransfer(I2C1, 0x64,LL_I2C_ADDRSLAVE_7BIT, 1,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	LL_I2C_TransmitData8(I2C1, addr_reg);

	counter=0;
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET){
		counter++;
		if( counter == 25000 ){//aproximate 150ms
			LL_I2C_ClearFlag_TXE(I2C1);
			Error_Handler();
			return ERROR;
		}
	}

	LL_I2C_TransmitData8(I2C1, value);
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET);

	LL_I2C_ClearFlag_STOP(I2C1);
	return SUCCESS;
}


int8_t I2C_write_reg_16bit(uint8_t SlaveAddr_IC, uint8_t addr_reg, uint16_t value)
{

	SlaveAddr_IC = SlaveAddr_IC<<1;

	uint32_t counter = 0;
	while(LL_I2C_IsActiveFlag_BUSY(I2C1)==SET){
		counter++;
		if( counter == 25000 ){//aproximate 150ms
			Error_Handler();
			return ERROR;
		}
	}

	LL_I2C_HandleTransfer(I2C1, SlaveAddr_IC,LL_I2C_ADDRSLAVE_7BIT, 3,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	LL_I2C_TransmitData8(I2C1, addr_reg);

	counter=0;
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET){
		counter++;
		if( counter == 25000){//aproximate 150ms
			LL_I2C_ClearFlag_TXE(I2C1);
			Error_Handler();
			return ERROR;
		}
	}

	LL_I2C_TransmitData8(I2C1, (uint8_t)(value>>8)); //byte1
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	LL_I2C_TransmitData8(I2C1, (uint8_t) value ); //byte2
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET);

	LL_I2C_ClearFlag_STOP(I2C1);
	return SUCCESS;

}


uint8_t I2C_read_reg_8bit(uint8_t SlaveAddr_IC, uint8_t addr_reg,uint8_t value)
{
	uint32_t counter = 0;

	while(LL_I2C_IsActiveFlag_BUSY(I2C1)==SET){
		counter++;
		if( counter == 25000){//aproximate 150ms
			Error_Handler();
			return 0xFF;
		}
	}

	LL_I2C_HandleTransfer(I2C1, (SlaveAddr_IC<<1)|0,LL_I2C_ADDRSLAVE_7BIT, 1,LL_I2C_MODE_SOFTEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	LL_I2C_TransmitData8(I2C1, addr_reg);

	counter=0;
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET){
		counter++;
		if( counter == 25000){//aproximate 150ms
			LL_I2C_ClearFlag_TXE(I2C1);
			Error_Handler();
			return 0xFF;
		}
	}
	while(LL_I2C_IsActiveFlag_TC(I2C1)==RESET);

	LL_I2C_HandleTransfer(I2C1, (SlaveAddr_IC<<1)|1, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ); //LL_I2C_MODE_SOFTEND

	while(LL_I2C_IsActiveFlag_RXNE(I2C1)==RESET);

	value = LL_I2C_ReceiveData8(I2C1);

	while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET);
	LL_I2C_ClearFlag_STOP(I2C1);

	return value;
}

uint16_t I2C_read_reg_16bit(uint8_t SlaveAddr_IC, uint8_t addr_reg,uint8_t * value)
{

	uint8_t i=0;
	uint32_t counter = 0;

	while(LL_I2C_IsActiveFlag_BUSY(I2C1)==SET){
		counter++;
		if( counter == 25000 ){//aproximate 150ms
			Error_Handler();
			return 0xFF;
		}
	}

	LL_I2C_HandleTransfer(I2C1, (SlaveAddr_IC<<1)|0,LL_I2C_ADDRSLAVE_7BIT, 1,LL_I2C_MODE_SOFTEND,LL_I2C_GENERATE_START_WRITE ); //LL_I2C_GENERATE_START_READ
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET);

	LL_I2C_TransmitData8(I2C1, addr_reg);

	counter=0;
	while(LL_I2C_IsActiveFlag_TXE(I2C1)==RESET){
		counter++;
		if( counter == 25000 ){//aproximate 150ms
			LL_I2C_ClearFlag_TXE(I2C1);
			Error_Handler();
			return 0xFF;
		}
	}
	while(LL_I2C_IsActiveFlag_TC(I2C1)==RESET);

	LL_I2C_HandleTransfer(I2C1, (SlaveAddr_IC<<1)|1, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ); //LL_I2C_MODE_SOFTEND

	while(!LL_I2C_IsActiveFlag_STOP(I2C1)){

		if(LL_I2C_IsActiveFlag_RXNE(I2C1)){
			value[i] = LL_I2C_ReceiveData8(I2C1);
			i++;
		}
	}

	LL_I2C_ClearFlag_STOP(I2C1);

	return (value[0]<<8) | value[1];
}



void st_delay(uint32_t st_time)
{
	LL_mDelay(st_time);
}

int8_t st_i2c_write(uint8_t slave_addr, uint8_t reg_addr,uint8_t *data_buf, uint8_t len)
{

}
int8_t st_i2c_read(uint8_t slave_addr, uint8_t reg_addr,uint8_t *data_buf, uint8_t len)
{



}

zmod4xxx_err init_hardware(zmod4xxx_dev_t *dev)
{
//    dev->read = st_i2c_read;
//    dev->write = st_i2c_write;
    dev->delay_ms = st_delay;
    return ZMOD4XXX_OK;
}

zmod4xxx_err deinit_hardware()
{
    return ZMOD4XXX_OK;
}
