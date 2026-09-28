/*
 * Copyright (c) 2019-2020, Dmitry Slepynin (DiSlord) dislordlive@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

//*****************************************************************************
//********************************** SPI1 bus *********************************
//*****************************************************************************
// STM32 SPI transfer mode:
// in 8 bit mode:
// if you write *(uint8_t*)(&SPI1->DR)  = (uint8_t) data, then data send as << data
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << dataLoByte, after send dataHiByte
// in 16 bit mode
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << data

// SPI init in 8 bit mode
#define SPI_CR2_8BIT  0x0700
#define SPI_CR2_16BIT 0x0F00

//*****************************************************
// SPI bus baud rate (PPL/BR_DIV)
//*****************************************************
#define	SPI_BR_DIV2   (0x00000000U)
#define	SPI_BR_DIV4   (SPI_CR1_BR_0)
#define	SPI_BR_DIV8   (SPI_CR1_BR_1)
#define	SPI_BR_DIV16  (SPI_CR1_BR_1|SPI_CR1_BR_0)
#define	SPI_BR_DIV32  (SPI_CR1_BR_2)
#define	SPI_BR_DIV64  (SPI_CR1_BR_2|SPI_CR1_BR_0)
#define	SPI_BR_DIV128 (SPI_CR1_BR_2|SPI_CR1_BR_1)
#define	SPI_BR_DIV256 (SPI_CR1_BR_2|SPI_CR1_BR_1|SPI_CR1_BR_0)

#define SPI_BR_SET(spi, br)  (spi->CR1 = (spi->CR1& ~(SPI_CR1_BR))|br)

// SD card SPI speed on work and on initialization (100-400kHz need), for ili9341.c and fw_update.c
#define SD_SPI_SPEED        SPI_BR_DIV2
#define SD_INIT_SPI_SPEED   SPI_BR_DIV256

//*****************************************************
// SPI bus activity macros
//*****************************************************
// The RXNE flag is set depending on the FRXTH bit value in the SPIx_CR2 register:
// • If FRXTH is set, RXNE goes high and stays high until the RXFIFO level is greater or equal to 1/4 (8-bit).
#define SPI_RX_IS_NOT_EMPTY(spi)  (spi->SR&SPI_SR_RXNE)
#define SPI_RX_IS_EMPTY(spi)     (((spi->SR&SPI_SR_RXNE) == 0))

// The TXE flag is set when transmission TXFIFO has enough space to store data to send.
// 0: Tx buffer not empty, bit is cleared automatically when the TXFIFO level becomes greater than 1/2
// 1: Tx buffer empty, flag goes high and stays high until the TXFIFO level is lower or equal to 1/2 of the FIFO depth
#define SPI_TX_IS_NOT_EMPTY(spi)  (((spi->SR&(SPI_SR_TXE)) == 0))
#define SPI_TX_IS_EMPTY(spi)     (spi->SR&SPI_SR_TXE)

// When BSY is set, it indicates that a data transfer is in progress on the SPI (the SPI bus is busy).
#define SPI_IS_BUSY(spi)     (spi->SR & SPI_SR_BSY)

// Tx or Rx in process
#define SPI_IN_TX_RX(spi)    ((spi->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || SPI_IS_BUSY(spi))

//*****************************************************
// SPI send data macros
//*****************************************************
#define SPI_WRITE_8BIT(spi, data)  *(__IO uint8_t*)(&spi->DR) = (uint8_t) data
#define SPI_WRITE_16BIT(spi, data) *(__IO uint16_t*)(&spi->DR) = (uint16_t) data

//*****************************************************
// SPI read data macros
//*****************************************************
#define SPI_READ_8BIT(spi)       *(__IO uint8_t*)(&spi->DR)
#define SPI_READ_16BIT(spi)      *(__IO uint16_t*)(&spi->DR)

//*****************************************************
// DMA channels macros
//*****************************************************
#define dmaChannelSetMemory(ch, addr)          {(ch)->CMAR = (uint32_t)(addr);}
#define dmaChannelSetPeripheral(ch, addr)      {(ch)->CPAR = (uint32_t)(addr);}
#define dmaChannelSetTransactionSize(ch, size) {(ch)->CNDTR= (uint32_t)(size);}
#define dmaChannelGetTransactionSize(ch)       ((ch)->CNDTR)
#define dmaChannelSetMode(ch, mode)            {(ch)->CCR  = (uint32_t)(mode);}
#define dmaChannelEnable(ch)                   {(ch)->CCR |= STM32_DMA_CR_EN;}
#define dmaChannelDisable(ch)                  {(ch)->CCR &=~STM32_DMA_CR_EN;}
#define dmaChannelWaitCompletion(ch)           {while ((ch)->CNDTR > 0); (ch)->CCR = 0;}

#define STM32_DMA_CR_BYTE       (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE)
#define STM32_DMA_CR_HWORD      (STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD)

#ifdef TINYSA4
#define SPI_MODE_LCD          0x00
#define SPI_MODE_SD_CARD      0x01
#define SPI_MODE_SD_CARD_LOW  0x02
#define SPI_MODE_SI           0x03
#define SPI_MODE_PE           0x04
void set_SPI_mode(uint16_t mode);
void start_SI4432_SPI_mode(void);
void stop_SI4432_SPI_mode(void);
void start_PE4312_SPI_mode(void);
void stop_PE4312_SPI_mode(void);
#endif
