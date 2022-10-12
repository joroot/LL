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
 * @file    hal_arduino.h
 * @brief   zmod4xxx hardware abstraction layer for Arduino platform
 * @version 2.4.2
 * @author Renesas Electronics Corporation
 */

#ifndef _ST_I2C_H
#define _ST_I2C_H

#include "zmod4xxx_types.h"
#include "stm32g031xx.h"

typedef enum
{
  BSP_OK       = 0x00U,
  BSP_ERROR    = 0x01U,
  BSP_BUSY     = 0x02U,
  BSP_TIMEOUT  = 0x03U
} BSP_StatusTypeDef;

#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_DIR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOPF   (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TC      (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TCR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TXIS    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define MAX_NBYTE_SIZE      255U
#define SlaveAddr_SHIFT     7U
#define SlaveAddr_MSK       0x06U

/** @defgroup I2C_Flag_definition I2C Flag definition
  * @{
  */
#define I2C_FLAG_TXE                    I2C_ISR_TXE
#define I2C_FLAG_TXIS                   I2C_ISR_TXIS
#define I2C_FLAG_RXNE                   I2C_ISR_RXNE
#define I2C_FLAG_ADDR                   I2C_ISR_ADDR
#define I2C_FLAG_AF                     I2C_ISR_NACKF
#define I2C_FLAG_STOPF                  I2C_ISR_STOPF
#define I2C_FLAG_TC                     I2C_ISR_TC
#define I2C_FLAG_TCR                    I2C_ISR_TCR
#define I2C_FLAG_BERR                   I2C_ISR_BERR
#define I2C_FLAG_ARLO                   I2C_ISR_ARLO
#define I2C_FLAG_OVR                    I2C_ISR_OVR
#define I2C_FLAG_PECERR                 I2C_ISR_PECERR
#define I2C_FLAG_TIMEOUT                I2C_ISR_TIMEOUT
#define I2C_FLAG_ALERT                  I2C_ISR_ALERT
#define I2C_FLAG_BUSY                   I2C_ISR_BUSY
#define I2C_FLAG_DIR                    I2C_ISR_DIR

/** @defgroup I2C_RELOAD_END_MODE I2C Reload End Mode
  * @{
  */
#define  I2C_RELOAD_MODE                I2C_CR2_RELOAD
#define  I2C_AUTOEND_MODE               I2C_CR2_AUTOEND
#define  I2C_SOFTEND_MODE               (0x00000000U)
/**
  * @}
  */

	/** @defgroup I2C_START_STOP_MODE I2C Start or Stop Mode
  * @{
  */
#define  I2C_NO_STARTSTOP               (0x00000000U)
#define  I2C_GENERATE_STOP              I2C_CR2_STOP
#define  I2C_GENERATE_START_READ        (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN)
#define  I2C_GENERATE_START_WRITE       I2C_CR2_START
/**
  * @}
  */

	/** @defgroup I2C_MEMORY_ADDRESS_SIZE I2C Memory Address Size
  * @{
  */
#define I2C_MEMADD_SIZE_8BIT            (0x00000001U)
#define I2C_MEMADD_SIZE_16BIT           (0x00000002U)
/**
  * @}
  */

/** @brief  Check whether the specified I2C flag is set or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_TXIS    Transmit interrupt status
  *            @arg @ref I2C_FLAG_RXNE    Receive data register not empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_TC      Transfer complete (master mode)
  *            @arg @ref I2C_FLAG_TCR     Transfer complete reload
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *            @arg @ref I2C_FLAG_BUSY    Bus busy
  *            @arg @ref I2C_FLAG_DIR     Transfer direction (slave mode)
  *
  * @retval The new state of __FLAG__ (SET or RESET).
  */
#define __I2C_GET_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->ISR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

/** @brief  Clear the I2C pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *
  * @retval None
  */
#define __I2C_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__FLAG__) == I2C_FLAG_TXE) ? ((__HANDLE__)->ISR |= (__FLAG__)) \
                                                                                 : ((__HANDLE__)->ICR = (__FLAG__)))


#define I2C_RESET_CR2(__HANDLE__)                 ((__HANDLE__)->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)))

#define I2C_MEM_ADD_MSB(__ADDRESS__)              ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))

BSP_StatusTypeDef BSP_I2C_Mem_Write(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
BSP_StatusTypeDef BSP_I2C_Mem_Read(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);


uint8_t i2c_read_len(I2C_TypeDef *I2Cx,  uint8_t Addr , uint8_t Reg, uint8_t len,uint8_t *buf);

uint8_t i2c_write_len(I2C_TypeDef *I2Cx,  uint8_t addr , uint8_t reg, uint8_t len,uint8_t *buf);


int8_t I2C_write_reg_8bit(uint8_t SlaveAddr_IC, uint8_t addr_reg, uint8_t value);
int8_t I2C_write_reg_16bit(uint8_t SlaveAddr_IC, uint8_t addr_reg, uint16_t value);
uint8_t I2C_read_reg_8bit(uint8_t SlaveAddr_IC, uint8_t addr_reg,uint8_t value);
uint16_t I2C_read_reg_16bit(uint8_t SlaveAddr_IC, uint8_t addr_reg,uint8_t * value);

int8_t st_i2c_write(uint8_t slave_addr, uint8_t reg_addr,uint8_t *data_buf, uint8_t len);
int8_t st_i2c_read(uint8_t slave_addr, uint8_t reg_addr,uint8_t *data_buf, uint8_t len);


/**
 * @brief   Initialize the target hardware
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err init_hardware(zmod4xxx_dev_t *dev);



void st_delay(uint32_t st_time);


/**
 * @brief   deinitialize hardware if it is required
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
zmod4xxx_err deinit_hardware();

#endif //_HAL_ARDUINO_H
