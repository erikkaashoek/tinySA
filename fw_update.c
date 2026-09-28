/*
 * Firmware update from the SD card (tinySA ULTRA), see fw_update.h
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
 */
#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include "spi.h"
#include "fw_update.h"
#include <string.h>

#ifdef __SD_FIRMWARE_UPDATE__

#ifdef __WATCHDOG__
#error "The firmware update runs about 10 s with interrupts off and does not refresh the IWDG"
#endif

//*****************************************************************************
// Image header and trailer of this firmware. The linker script places the header after the
// vector table and the trailer at the end of the image.
//*****************************************************************************
extern const char __fw_image_size__[];   // linker script symbol, its address is the image size

_Static_assert(sizeof(VERSION) <= sizeof(((fw_header_t *)0)->version), "VERSION too long for fw_header_t");
_Static_assert(sizeof(fw_trailer_t) == 12, "fw_trailer_t must be 12 bytes");

__attribute__((used, section(".fwinfo")))
const fw_header_t fw_header = {
  .magic       = {FW_HEADER_MAGIC0, FW_HEADER_MAGIC1},
  .hdr_version = FW_HEADER_VERSION,
  .hdr_size    = sizeof(fw_header_t),
  .model       = FW_MODEL_TINYSA4,
  .load_addr   = FLASH_START_ADDRESS,
  .image_size  = (uint32_t)__fw_image_size__,
  .flags       = 0,
  .version     = VERSION,
};

__attribute__((used, section(".fwtrailer")))
const fw_trailer_t fw_trailer = {
  .magic       = FW_TRAILER_MAGIC,
  .image_size  = (uint32_t)__fw_image_size__,
  .crc         = FW_CRC_UNSET,
};

//*****************************************************************************
// Limits and layout
//*****************************************************************************
#define FWU_APP_START       FLASH_START_ADDRESS
#define FWU_APP_END         SAVE_CONFIG_ADDR    // never erase or program at or above this address
#define FWU_MAX_PAGES       ((FWU_APP_END - FWU_APP_START) / FLASH_PAGESIZE)
#define FWU_MAX_EXTENTS     64                  // contiguous sector runs of the image file
#define FWU_SECTOR_SIZE     512
#define FWU_MIN_BATTERY_MV  3500

extern const char __ram0_start__[], __ram0_end__[];   // RAM of this MCU, from the linker script

// FWU_APP_END as a symbol, STM32F303xC.ld checks that it is the end of flash0
#define FWU_STR_(x)         #x
#define FWU_STR(x)          FWU_STR_(x)
__asm__(".global __fwu_app_end__\n\t.set __fwu_app_end__, " FWU_STR(FWU_APP_END));
#define FWU_PAGE_BUF        ((uint8_t *)spi_buffer)                    // 2 KB page buffer in SRAM (4 KB spi_buffer)
#define FWU_PAGE0_BUF       ((uint8_t *)spi_buffer + FLASH_PAGESIZE)   // page 0, read before the erase starts

// Progress bar on the update screen, drawn by the CCM code
#define FWU_BAR_X           40
#define FWU_BAR_Y           130
#define FWU_BAR_W           400
#define FWU_BAR_H           24
#define FWU_COLOR_BAR_BG    RGB565( 64, 64, 64)
#define FWU_COLOR_WRITE     RGB565(  0,128,255)
#define FWU_COLOR_RETRY     RGB565(255,255,  0)
#define FWU_COLOR_NOCARD    RGB565(255,  0,255)
#define FWU_COLOR_ERROR     RGB565(255,  0,  0)
#define FWU_COLOR_DONE      RGB565(  0,255,  0)

//*****************************************************************************
// CCM RAM code and data
// The flasher runs from CCM RAM with interrupts off while the application flash is erased.
// Rules for FWU_TEXT functions: no calls into flash (library functions, memcpy/memset, struct
// copies, string literals, switch tables, OS or HAL functions), only register access.
// .ram4_init.* is copied from flash to CCM at boot by crt1 (__init_ram_areas).
//*****************************************************************************
#define FWU_TEXT    __attribute__((section(".ram4_init.fwu_text"), noinline))
#define FWU_BSS     __attribute__((section(".ram4.fwu_bss")))
#define FWU_INLINE  static inline __attribute__((always_inline))

typedef struct {
  uint32_t lba;        // first card sector
  uint32_t count;      // number of sectors
} fwu_extent_t;

typedef struct {
  uint32_t image_size;                  // bytes to program
  uint32_t n_pages;                     // flash pages to program
  uint32_t image_crc;                   // CRC32 of the whole image
  uint32_t n_ext;                       // used entries in ext[]
  uint32_t byte_addr;                   // SDSC card: command argument is a byte address
  uint32_t bar_w;                       // width of the progress bar already drawn in bar_color
  uint32_t bar_color;
  uint32_t page_crc[FWU_MAX_PAGES];     // CRC32 of the image bytes in each flash page
  fwu_extent_t ext[FWU_MAX_EXTENTS];    // file sectors -> card sectors
} fwu_job_t;

FWU_BSS static fwu_job_t fwu_job;
FWU_BSS static uint32_t fwu_vtor[16] __attribute__((aligned(128)));   // exception vectors in CCM

// CCM functions called from flash code
__attribute__((long_call)) uint32_t fwu_crc32(uint32_t crc, const void *data, uint32_t len);
__attribute__((long_call)) static int fwu_dry_run(void);
__attribute__((long_call, noreturn)) static void fwu_run(void);

// Internal results of the CCM code
#define FWU_R_OK      0
#define FWU_R_NOCARD  1
#define FWU_R_READ    2
#define FWU_R_CRC     3
#define FWU_R_FLASH   4
#define FWU_R_VERIFY  5

// TIM2 is the free running 32 bit ChibiOS system timer (10 kHz), it runs with interrupts off
#define FWU_NOW()           (STM32_ST_TIM->CNT)
#define FWU_TICKS(ms)       ((uint32_t)(ms) * (CH_CFG_ST_FREQUENCY / 1000))
#define FWU_DSB()           __asm volatile ("dsb 0xF" ::: "memory")
#define FWU_ISB()           __asm volatile ("isb 0xF" ::: "memory")

#define FWU_SD_CS_LOW()     palClearLine(LINE_SD_CS)
#define FWU_SD_CS_HIGH()    palSetLine(LINE_SD_CS)
#define FWU_LCD_CS_LOW()    palClearPad(GPIO_LCD_CS_PORT, GPIO_LCD_CS)
#define FWU_LCD_CS_HIGH()   palSetPad(GPIO_LCD_CS_PORT, GPIO_LCD_CS)
#define FWU_LCD_DC_CMD()    palClearPad(GPIO_LCD_CD_PORT, GPIO_LCD_CD)
#define FWU_LCD_DC_DATA()   palSetPad(GPIO_LCD_CD_PORT, GPIO_LCD_CD)
#define FWU_CARD_PRESENT()  ((palReadPort(GPIOB) & (1U << GPIOB_SD_CD)) == 0)
#define FWU_JOG_PUSHED()    ((palReadPort(GPIOA) & (1U << GPIOA_PUSH)) != 0)

// CRC32 as zlib, chainable: crc = fwu_crc32(crc, next data, length), start with 0
FWU_TEXT uint32_t fwu_crc32(uint32_t crc, const void *data, uint32_t len) {
  const uint8_t *p = data;
  crc = ~crc;
  while (len--) {
    crc ^= *p++;
    for (int i = 0; i < 8; i++)
      crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1));
  }
  return ~crc;
}

FWU_TEXT static void fwu_delay(uint32_t ms) {
  uint32_t t0 = FWU_NOW();
  while (FWU_NOW() - t0 < FWU_TICKS(ms))
    ;
}

// SPI1 byte exchange, returns after the byte is fully transferred (RX FIFO stays empty)
FWU_INLINE uint8_t fwu_spi_xfer(uint8_t b) {
  while ((SPI1->SR & SPI_SR_TXE) == 0)
    ;
  *(volatile uint8_t *)&SPI1->DR = b;
  while ((SPI1->SR & SPI_SR_RXNE) == 0)
    ;
  return *(volatile uint8_t *)&SPI1->DR;
}

// Wait until SPI1 is idle, drop received data and clear an overrun
FWU_INLINE void fwu_spi_flush(void) {
  while (SPI1->SR & (SPI_SR_FTLVL | SPI_SR_BSY))
    ;
  while (SPI1->SR & SPI_SR_RXNE)
    (void)*(volatile uint8_t *)&SPI1->DR;
  (void)SPI1->SR;
}

//*****************************************************************************
// LCD progress bar (same SPI speed as the SD card, only the chip selects change)
//*****************************************************************************
FWU_INLINE void fwu_lcd_command(uint8_t cmd) {
  FWU_LCD_DC_CMD();
  fwu_spi_xfer(cmd);
  FWU_LCD_DC_DATA();
}

FWU_INLINE void fwu_lcd_data16(uint32_t v) {
  fwu_spi_xfer(v >> 8);
  fwu_spi_xfer(v);
}

// color is a pixel_t value as in the palette (sent low byte first)
FWU_TEXT static void fwu_lcd_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color) {
  fwu_spi_flush();
  FWU_SD_CS_HIGH();
  FWU_LCD_CS_LOW();
  fwu_lcd_command(0x2A);            // column address set
  fwu_lcd_data16(x);
  fwu_lcd_data16(x + w - 1);
  fwu_lcd_command(0x2B);            // page address set
  fwu_lcd_data16(y);
  fwu_lcd_data16(y + h - 1);
  fwu_lcd_command(0x2C);            // memory write
  for (uint32_t n = w * h; n; n--) {
    fwu_spi_xfer(color);
    fwu_spi_xfer(color >> 8);
  }
  FWU_LCD_CS_HIGH();
}

// Show done pages of all pages in color
FWU_TEXT static void fwu_bar(uint32_t done, uint32_t color) {
  uint32_t w = FWU_BAR_W * done / fwu_job.n_pages;
  if (color != fwu_job.bar_color || w < fwu_job.bar_w) {
    fwu_job.bar_w = 0;                                  // redraw all
    fwu_job.bar_color = color;
    fwu_lcd_fill(FWU_BAR_X, FWU_BAR_Y, FWU_BAR_W, FWU_BAR_H, FWU_COLOR_BAR_BG);
  }
  if (w > fwu_job.bar_w)
    fwu_lcd_fill(FWU_BAR_X + fwu_job.bar_w, FWU_BAR_Y, w - fwu_job.bar_w, FWU_BAR_H, color);
  fwu_job.bar_w = w;
}

//*****************************************************************************
// SD card sector read, polled SPI. The firmware initializes the card before the update,
// fwu_sd_init() does it again for a card that was removed or disturbed during the update.
//*****************************************************************************
#define FWU_CMD0            (0x40 + 0)    // GO_IDLE_STATE
#define FWU_CMD1            (0x40 + 1)    // SEND_OP_COND (MMC)
#define FWU_CMD8            (0x40 + 8)    // SEND_IF_COND
#define FWU_CMD16           (0x40 + 16)   // SET_BLOCKLEN
#define FWU_CMD17           (0x40 + 17)   // READ_SINGLE_BLOCK
#define FWU_CMD55           (0x40 + 55)   // APP_CMD
#define FWU_CMD58           (0x40 + 58)   // READ_OCR
#define FWU_ACMD41          (0x40 + 41)   // SD_SEND_OP_COND

FWU_TEXT static bool fwu_sd_wait_ready(uint32_t ms) {
  uint32_t t0 = FWU_NOW();
  do {
    if (fwu_spi_xfer(0xFF) == 0xFF)
      return true;
  } while (FWU_NOW() - t0 < FWU_TICKS(ms));
  return false;
}

FWU_TEXT static uint8_t fwu_sd_cmd(uint8_t cmd, uint32_t arg) {
  if (!fwu_sd_wait_ready(500))
    return 0xFF;
  fwu_spi_xfer(cmd);
  fwu_spi_xfer(arg >> 24);
  fwu_spi_xfer(arg >> 16);
  fwu_spi_xfer(arg >> 8);
  fwu_spi_xfer(arg);
  fwu_spi_xfer(cmd == FWU_CMD0 ? 0x95 : cmd == FWU_CMD8 ? 0x87 : 0x01);  // CRC is checked for CMD0 and CMD8 only
  uint8_t r1;
  int n = 100;
  while (((r1 = fwu_spi_xfer(0xFF)) & 0x80) && --n)
    ;
  return r1;
}

// 32 bit response after R1 (R7 of CMD8, OCR of CMD58), most significant byte first
FWU_TEXT static uint32_t fwu_sd_rx32(void) {
  uint32_t v = 0;
  for (int i = 0; i < 4; i++)
    v = (v << 8) | fwu_spi_xfer(0xFF);
  return v;
}

// Card initialization, same sequence as disk_initialize() in ili9341.c. Sets fwu_job.byte_addr.
FWU_TEXT static bool fwu_sd_init(void) {
  bool ok = false, byte_addr = true;
  fwu_spi_flush();
  FWU_LCD_CS_HIGH();
  FWU_SD_CS_HIGH();
  SPI_BR_SET(SPI1, SD_INIT_SPI_SPEED);
  for (int i = 0; i < 10; i++)            // 80 clocks with the card not selected
    fwu_spi_xfer(0xFF);
  FWU_SD_CS_LOW();
  if (fwu_sd_cmd(FWU_CMD0, 0) == 0x01) {
    uint32_t t0 = FWU_NOW();
    bool ready = false;
    if (fwu_sd_cmd(FWU_CMD8, 0x1AA) == 0x01) {
      // SDv2: accepts 2.7-3.6 V, wait up to 1 s until it leaves the idle state, then read the CCS bit
      if (fwu_sd_rx32() == 0x1AA) {
        do {
          if (fwu_sd_cmd(FWU_CMD55, 0) <= 1 && fwu_sd_cmd(FWU_ACMD41, 0x40000000) == 0) {
            ready = true;
            break;
          }
          fwu_delay(10);
        } while (FWU_NOW() - t0 < FWU_TICKS(1000));
        if (ready && fwu_sd_cmd(FWU_CMD58, 0) == 0) {
          byte_addr = (fwu_sd_rx32() & 0x40000000) == 0;
          ok = true;
        }
      }
    } else {
      // SDv1 or MMC, byte addresses with a 512 byte block length
      bool sd1 = fwu_sd_cmd(FWU_CMD55, 0) <= 1 && fwu_sd_cmd(FWU_ACMD41, 0) <= 1;
      do {
        if (sd1 ? (fwu_sd_cmd(FWU_CMD55, 0) <= 1 && fwu_sd_cmd(FWU_ACMD41, 0) == 0) : fwu_sd_cmd(FWU_CMD1, 0) == 0) {
          ready = true;
          break;
        }
        fwu_delay(10);
      } while (FWU_NOW() - t0 < FWU_TICKS(1000));
      ok = ready && fwu_sd_cmd(FWU_CMD16, FWU_SECTOR_SIZE) == 0;
    }
  }
  FWU_SD_CS_HIGH();
  fwu_spi_xfer(0xFF);
  SPI_BR_SET(SPI1, SD_SPI_SPEED);
  if (ok)
    fwu_job.byte_addr = byte_addr;
  return ok;
}

FWU_TEXT static bool fwu_sd_read_sector(uint32_t lba, uint8_t *dst) {
  bool ok = false;
  fwu_spi_flush();
  FWU_LCD_CS_HIGH();
  FWU_SD_CS_LOW();
  if (fwu_sd_cmd(FWU_CMD17, fwu_job.byte_addr ? lba * FWU_SECTOR_SIZE : lba) == 0) {
    uint32_t t0 = FWU_NOW();
    uint8_t token;
    do {
      token = fwu_spi_xfer(0xFF);
    } while (token == 0xFF && FWU_NOW() - t0 < FWU_TICKS(200));
    if (token == 0xFE) {                  // start block token
      for (uint32_t i = 0; i < FWU_SECTOR_SIZE; i++)
        dst[i] = fwu_spi_xfer(0xFF);
      fwu_spi_xfer(0xFF);                 // CRC, not checked
      fwu_spi_xfer(0xFF);
      ok = true;
    }
  }
  FWU_SD_CS_HIGH();
  fwu_spi_xfer(0xFF);                     // one byte after CS up
  return ok;
}

// Card sector of file sector s
FWU_TEXT static uint32_t fwu_lba(uint32_t s) {
  for (uint32_t i = 0; i < fwu_job.n_ext; i++) {
    if (s < fwu_job.ext[i].count)
      return fwu_job.ext[i].lba + s;
    s -= fwu_job.ext[i].count;
  }
  return 0xFFFFFFFFU;
}

// The card counts as missing only when the detect switch reads "no card" for 250 ms without
// interruption, a short glitch on the detect line is ignored
FWU_TEXT static bool fwu_card_missing(void) {
  uint32_t t0 = FWU_NOW();
  while (!FWU_CARD_PRESENT())
    if (FWU_NOW() - t0 >= FWU_TICKS(250))
      return true;
  return false;
}

// Read image page p from the card into buf, pad with 0xFF, check the page CRC
FWU_TEXT static int fwu_read_page(uint32_t p, uint8_t *buf) {
  uint32_t off = p * FLASH_PAGESIZE;
  uint32_t len = fwu_job.image_size - off;
  if (len > FLASH_PAGESIZE)
    len = FLASH_PAGESIZE;
  for (uint32_t s = 0; s * FWU_SECTOR_SIZE < len; s++) {
    uint32_t lba = fwu_lba(off / FWU_SECTOR_SIZE + s);
    int tries = 3;
    while (!fwu_sd_read_sector(lba, buf + s * FWU_SECTOR_SIZE)) {
      if (fwu_card_missing())
        return FWU_R_NOCARD;
      if (--tries == 0)
        return FWU_R_READ;
    }
  }
  for (uint32_t i = len; i < FLASH_PAGESIZE; i++)
    buf[i] = 0xFF;
  if (fwu_crc32(0, buf, len) != fwu_job.page_crc[p])
    return FWU_R_CRC;
  return FWU_R_OK;
}

// Read all pages with the raw sector reader while the firmware still runs
FWU_TEXT static int fwu_dry_run(void) {
  for (uint32_t p = 0; p < fwu_job.n_pages; p++) {
    int r = fwu_read_page(p, FWU_PAGE_BUF);
    if (r != FWU_R_OK)
      return r;
  }
  return FWU_R_OK;
}

//*****************************************************************************
// Flash erase / program, never outside FWU_APP_START..FWU_APP_END
//*****************************************************************************
FWU_TEXT static int fwu_flash_wait(uint32_t ms) {
  uint32_t t0 = FWU_NOW();
  while (FLASH->SR & FLASH_SR_BSY)
    if (FWU_NOW() - t0 > FWU_TICKS(ms))
      return FWU_R_FLASH;
  uint32_t sr = FLASH->SR;
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;   // write 1 to clear
  return (sr & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) ? FWU_R_FLASH : FWU_R_OK;
}

FWU_TEXT static int fwu_flash_erase(uint32_t addr) {
  if (addr < FWU_APP_START || addr >= FWU_APP_END)
    return FWU_R_FLASH;
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = addr;
  FLASH->CR |= FLASH_CR_STRT;
  int r = fwu_flash_wait(100);
  FLASH->CR &= ~FLASH_CR_PER;
  return r;
}

FWU_TEXT static int fwu_flash_program(uint32_t addr, const uint16_t *src) {
  if (addr < FWU_APP_START || addr + FLASH_PAGESIZE > FWU_APP_END)
    return FWU_R_FLASH;
  int r = FWU_R_OK;
  volatile uint16_t *dst = (volatile uint16_t *)addr;
  FLASH->CR |= FLASH_CR_PG;
  for (uint32_t i = 0; i < FLASH_PAGESIZE / 2 && r == FWU_R_OK; i++) {
    if (src[i] == 0xFFFF)                 // erased value
      continue;
    dst[i] = src[i];
    r = fwu_flash_wait(10);
  }
  FLASH->CR &= ~FLASH_CR_PG;
  return r;
}

FWU_TEXT static bool fwu_flash_equal(uint32_t addr, const uint32_t *src) {
  const volatile uint32_t *f = (const volatile uint32_t *)addr;
  for (uint32_t i = 0; i < FLASH_PAGESIZE / 4; i++)
    if (f[i] != src[i])
      return false;
  return true;
}

//*****************************************************************************
// Update sequence
//*****************************************************************************
// Reset into the firmware. With the jog button pressed the reset would start the ROM DFU bootloader (BOOT0).
FWU_TEXT __attribute__((noreturn)) static void fwu_reset(void) {
  while (FWU_JOG_PUSHED())
    ;
  fwu_delay(50);
  FWU_DSB();
  SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk;
  FWU_DSB();
  while (true)
    ;
}

// Any exception after the flash erase started ends here: red bar, the unit needs DFU
FWU_TEXT static void fwu_fault(void) {
  fwu_job.bar_color = 0;
  fwu_bar(fwu_job.n_pages, FWU_COLOR_ERROR);
  while (true)
    ;
}

// Pause before the next attempt, no input needed: a missing card (magenta) is waited for,
// other errors are retried after a pause, yellow at first, red when they keep failing.
// A card that was removed, or keeps failing, is initialized again.
FWU_TEXT static void fwu_retry_wait(int r, uint32_t done, uint32_t attempt) {
  if (r == FWU_R_NOCARD) {
    fwu_bar(fwu_job.n_pages, FWU_COLOR_NOCARD);
    uint32_t t0 = FWU_NOW();
    while (FWU_NOW() - t0 < FWU_TICKS(1000))  // card back for 1 s without interruption
      if (!FWU_CARD_PRESENT())
        t0 = FWU_NOW();
  } else if (attempt < 3) {
    fwu_bar(done, FWU_COLOR_RETRY);
    fwu_delay(100);
  } else {
    fwu_bar(fwu_job.n_pages, FWU_COLOR_ERROR);
    fwu_delay(1000);
  }
  if (r == FWU_R_NOCARD || ((r == FWU_R_READ || r == FWU_R_CRC) && attempt >= 1))
    fwu_sd_init();
}

// Erase, program and verify one page, retry until it succeeds. Pages 1.. are read from the
// card, page 0 is in FWU_PAGE0_BUF since the start of the update.
FWU_TEXT static void fwu_update_page(uint32_t p) {
  uint32_t addr = FWU_APP_START + p * FLASH_PAGESIZE;
  uint8_t *buf = p == 0 ? FWU_PAGE0_BUF : FWU_PAGE_BUF;
  for (uint32_t attempt = 0; ; attempt++) {
    int r = p == 0 ? FWU_R_OK : fwu_read_page(p, buf);
    if (r == FWU_R_OK) {
      if (p != 0 && fwu_flash_equal(addr, (const uint32_t *)buf))
        return;                                         // already programmed
      r = fwu_flash_erase(addr);
      if (r == FWU_R_OK)
        r = fwu_flash_program(addr, (const uint16_t *)buf);
      if (r == FWU_R_OK && !fwu_flash_equal(addr, (const uint32_t *)buf))
        r = FWU_R_VERIFY;
      if (r == FWU_R_OK)
        return;
    }
    fwu_retry_wait(r, p, attempt);
  }
}

// CRC of the image as it will be after page 0 is programmed: page 0 from FWU_PAGE0_BUF, the rest from flash
FWU_TEXT static uint32_t fwu_final_crc(void) {
  uint32_t len0 = fwu_job.image_size < FLASH_PAGESIZE ? fwu_job.image_size : FLASH_PAGESIZE;
  uint32_t crc = fwu_crc32(0, FWU_PAGE0_BUF, len0);
  return fwu_crc32(crc, (const void *)(FWU_APP_START + len0), fwu_job.image_size - len0);
}

// Entered with interrupts off, never returns
FWU_TEXT static void fwu_run(void) {
  // Exceptions go to CCM, the vector table in flash page 0 is erased first
  for (int i = 0; i < 16; i++)
    fwu_vtor[i] = (uint32_t)fwu_fault;
  SCB->VTOR = (uint32_t)fwu_vtor;
  FWU_DSB();
  FWU_ISB();
  fwu_spi_flush();
  fwu_job.bar_w = 0;
  fwu_job.bar_color = 0;
  fwu_bar(0, FWU_COLOR_WRITE);
  // Page 0 goes to RAM first, so the end of the update does not need the card. Nothing is
  // erased yet: when the card cannot be read the old firmware is started again.
  for (uint32_t attempt = 0; fwu_read_page(0, FWU_PAGE0_BUF) != FWU_R_OK; attempt++) {
    if (attempt >= 10) {
      fwu_bar(fwu_job.n_pages, FWU_COLOR_ERROR);
      fwu_delay(2000);
      fwu_reset();
    }
    fwu_delay(200);
    if (attempt >= 1)
      fwu_sd_init();
  }
  // Unlock the flash, the firmware does not lock it after saving the config
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  if (FLASH->CR & FLASH_CR_LOCK)
    fwu_reset();                        // nothing erased yet, restart the old firmware
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
  FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER);
  // Erase the vector table first: an interrupted update then leaves a firmware that does not
  // start (DFU needed) instead of a mix of old and new code
  for (uint32_t attempt = 0; fwu_flash_erase(FWU_APP_START) != FWU_R_OK; attempt++)
    fwu_retry_wait(FWU_R_FLASH, 0, attempt);
  for (uint32_t pass = 0; ; pass++) {
    for (uint32_t p = 1; p < fwu_job.n_pages; p++) {
      fwu_update_page(p);
      fwu_bar(p, FWU_COLOR_WRITE);
    }
    // Check the complete image before the vector table is written
    if (fwu_final_crc() == fwu_job.image_crc)
      break;
    fwu_retry_wait(FWU_R_VERIFY, 0, pass);  // check all pages again, unchanged pages are skipped
  }
  fwu_update_page(0);
  FLASH->CR |= FLASH_CR_LOCK;
  fwu_bar(fwu_job.n_pages, FWU_COLOR_DONE);
  fwu_delay(500);
  fwu_reset();
}

//*****************************************************************************
// Flash resident part, runs in the sweep thread
//*****************************************************************************
extern bool dirty;
volatile bool fwu_active = false;
static uint8_t fwu_saved_auto_capture;

static const char * const fwu_errors[FWU_E_COUNT] = {
  [FWU_OK]        = "OK",
  [FWU_E_BUSY]    = "busy, try again",
  [FWU_E_BATTERY] = "battery too low, charge first",
  [FWU_E_NOCARD]  = "no SD card or card error",
  [FWU_E_OPEN]    = "file not found",
  [FWU_E_READ]    = "file read error",
  [FWU_E_FORMAT]  = "not a firmware file for SD update",
  [FWU_E_MODEL]   = "firmware for another model",
  [FWU_E_SIZE]    = "image size error",
  [FWU_E_VECTOR]  = "vector table error",
  [FWU_E_TRAILER] = "file incomplete",
  [FWU_E_CRC]     = "CRC error",
  [FWU_E_FRAG]    = "file too fragmented, copy it again",
  [FWU_E_DRYRUN]  = "SD card sector read error",
};

const char *fwu_error_text(int err) {
  return (err >= 0 && err < FWU_E_COUNT) ? fwu_errors[err] : "error";
}

// Take the SD card and SPI1 for the update. Call fwu_end() afterwards, also on error.
int fwu_begin(void) {
  chSysLock();
  fwu_active = true;                    // the shell refuses all commands from now on
  chSysUnlock();
  fwu_saved_auto_capture = auto_capture;
  auto_capture = false;                 // no screen copies over USB
  systime_t start = chVTGetSystemTimeX();
  while (shell_direct_cmd && chVTGetSystemTimeX() - start < MS2ST(2000))
    chThdSleepMilliseconds(5);
  if (shell_direct_cmd)
    return FWU_E_BUSY;
  int16_t vbat = adc_vbat_read();
  if (vbat > 0 && vbat < FWU_MIN_BATTERY_MV)
    return FWU_E_BATTERY;
  if (!SD_Inserted())
    return FWU_E_NOCARD;
#ifdef __MCU_CLOCK_SHIFT__
  clock_at_48MHz();                     // flash timing and USB need the nominal clock
#endif
  SD_PowerOff();                        // full card initialization on the next mount
  return FWU_OK;
}

void fwu_end(void) {
  SD_PowerOff();
  auto_capture = fwu_saved_auto_capture;
  dirty = true;                         // restore MCU clock and hardware settings on the next sweep
  fwu_active = false;
}

// Read and check the header and vector table of an image file
int fwu_read_header(const char *path, fw_header_t *hdr) {
  uint32_t *w = (uint32_t *)spi_buffer;
  UINT br;
  if (f_mount(fs_volume, "", 1) != FR_OK)
    return FWU_E_NOCARD;
  if (f_open(fs_file, path, FA_READ) != FR_OK)
    return FWU_E_OPEN;
  FSIZE_t fsize = f_size(fs_file);
  FRESULT res = f_read(fs_file, w, FWU_SECTOR_SIZE, &br);
  f_close(fs_file);
  if (res != FR_OK)
    return FWU_E_READ;
  // The header follows the vector table (first 16 system vectors at least)
  uint32_t hofs = 0;
  for (uint32_t o = 16 * 4; o + sizeof(fw_header_t) <= br; o += 4)
    if (w[o / 4] == FW_HEADER_MAGIC0 && w[o / 4 + 1] == FW_HEADER_MAGIC1) {
      hofs = o;
      break;
    }
  if (hofs == 0)
    return FWU_E_FORMAT;
  memcpy(hdr, &w[hofs / 4], sizeof(fw_header_t));
  hdr->version[sizeof(hdr->version) - 1] = 0;
  if (hdr->hdr_version < 1 || hdr->hdr_size < sizeof(fw_header_t) || hdr->load_addr != FWU_APP_START)
    return FWU_E_FORMAT;
  if (hdr->model != FW_MODEL_TINYSA4)
    return FWU_E_MODEL;
  uint32_t size = hdr->image_size;
  if ((size & 3) || size < hofs + hdr->hdr_size + sizeof(fw_trailer_t) ||
      size > FWU_APP_END - FWU_APP_START || size > fsize)
    return FWU_E_SIZE;
  uint32_t sp = w[0], reset = w[1];
  if (sp <= (uint32_t)__ram0_start__ || sp > (uint32_t)__ram0_end__ || !(reset & 1) ||
      reset < FWU_APP_START + hofs + hdr->hdr_size || reset >= FWU_APP_START + size)
    return FWU_E_VECTOR;
  return FWU_OK;
}

// Read the whole image: page CRCs, image CRC, trailer check and the file sector map.
// Then read it once more with the raw sector reader used during the update.
int fwu_prepare(const char *path, const fw_header_t *hdr) {
  fwu_job_t *j = &fwu_job;
  uint8_t *buf = (uint8_t *)spi_buffer;          // SRAM, FatFs reads with DMA
  const uint32_t size = hdr->image_size;
  const uint32_t trailer_ofs = size - sizeof(fw_trailer_t);
  fw_trailer_t trailer;
  uint32_t crc = 0, body_crc = 0;
  UINT br;
  int err = FWU_OK;

  if (f_mount(fs_volume, "", 1) != FR_OK)
    return FWU_E_NOCARD;
  FIL *fp = fs_file;
  if (f_open(fp, path, FA_READ) != FR_OK)
    return FWU_E_OPEN;
  FATFS *fs = fp->obj.fs;
  j->image_size = size;
  j->n_pages = (size + FLASH_PAGESIZE - 1) / FLASH_PAGESIZE;
  j->n_ext = 0;
  j->byte_addr = !SD_BlockAddressing();
  memset(&trailer, 0, sizeof(trailer));
  // Read 512 bytes at sector aligned offsets: f_read then updates fp->clust for every sector
  for (uint32_t off = 0; off < size; off += FWU_SECTOR_SIZE) {
    uint32_t n = size - off;
    if (n > FWU_SECTOR_SIZE)
      n = FWU_SECTOR_SIZE;
    if (f_read(fp, buf, FWU_SECTOR_SIZE, &br) != FR_OK || br < n) {
      err = FWU_E_READ;
      break;
    }
    // Card sector of this file sector
    DWORD clst = fp->clust;
    if (clst < 2 || clst >= fs->n_fatent) {
      err = FWU_E_READ;
      break;
    }
    uint32_t lba = (uint32_t)fs->database + (uint32_t)fs->csize * (clst - 2) + ((off / FWU_SECTOR_SIZE) & (fs->csize - 1));
    if (j->n_ext && j->ext[j->n_ext - 1].lba + j->ext[j->n_ext - 1].count == lba)
      j->ext[j->n_ext - 1].count++;
    else if (j->n_ext < FWU_MAX_EXTENTS) {
      j->ext[j->n_ext].lba = lba;
      j->ext[j->n_ext].count = 1;
      j->n_ext++;
    } else {
      err = FWU_E_FRAG;
      break;
    }
    // CRCs
    uint32_t page = off / FLASH_PAGESIZE;
    if (off % FLASH_PAGESIZE == 0)
      j->page_crc[page] = 0;
    j->page_crc[page] = fwu_crc32(j->page_crc[page], buf, n);
    crc = fwu_crc32(crc, buf, n);
    if (off < size - 4)
      body_crc = fwu_crc32(body_crc, buf, off + n <= size - 4 ? n : size - 4 - off);
    // Trailer bytes (can be split over two sectors)
    for (uint32_t i = off < trailer_ofs ? trailer_ofs - off : 0; i < n; i++)
      ((uint8_t *)&trailer)[off + i - trailer_ofs] = buf[i];
  }
  f_close(fp);
  if (err != FWU_OK)
    return err;
  if (trailer.magic != FW_TRAILER_MAGIC || trailer.image_size != size)
    return FWU_E_TRAILER;
  if (trailer.crc != FW_CRC_UNSET && trailer.crc != body_crc)
    return FWU_E_CRC;
  j->image_crc = crc;
  // Dry run with the reader of the update
  ili9341_bulk_finish();
  set_SPI_mode(SPI_MODE_SD_CARD);
  if (fwu_dry_run() != FWU_R_OK)
    return FWU_E_DRYRUN;
  return FWU_OK;
}

uint32_t fwu_image_crc(void) {
  return fwu_job.image_crc;
}

uint32_t fwu_extent_count(void) {
  return fwu_job.n_ext;
}

bool fwu_card_byte_addressing(void) {
  return fwu_job.byte_addr != 0;
}

uint32_t fwu_running_crc(void) {
  return fwu_crc32(0, (const void *)FWU_APP_START, fw_header.image_size);
}

static void fwu_legend(int y, uint16_t color, const char *text) {
  background_color = color;
  ili9341_fill(FWU_BAR_X, y, 12, 12);
  ili9341_set_background(LCD_BG_COLOR);
  lcd_printf(FWU_BAR_X + 20, y + 1, "%s", text);
}

// Update screen: versions, warning, progress bar frame and color legend
void fwu_draw_screen(const fw_header_t *hdr) {
  int x = 10, y = 10;
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_clear_screen();
  lcd_set_font(FONT_NORMAL);
  lcd_printf(x, y, "FIRMWARE UPDATE");
  y += 2 * bFONT_STR_HEIGHT;
  lcd_printf(x, y, "Installed: %s", fw_header.version);
  y += bFONT_STR_HEIGHT + 4;
  lcd_printf(x, y, "New:       %s", hdr->version);
  y += 2 * bFONT_STR_HEIGHT;
  lcd_printf(x, y, "Do not switch off or remove the SD card.");
  y += bFONT_STR_HEIGHT + 4;
  lcd_printf(x, y, "The tinySA restarts when the update is done.");
  // Bar frame
  ili9341_set_background(LCD_FG_COLOR);
  ili9341_fill(FWU_BAR_X - 2, FWU_BAR_Y - 2, FWU_BAR_W + 4, FWU_BAR_H + 4);
  background_color = FWU_COLOR_BAR_BG;
  ili9341_fill(FWU_BAR_X, FWU_BAR_Y, FWU_BAR_W, FWU_BAR_H);
  ili9341_set_background(LCD_BG_COLOR);
  y = FWU_BAR_Y + FWU_BAR_H + 16;
  fwu_legend(y, FWU_COLOR_WRITE,  "writing");
  y += bFONT_STR_HEIGHT + 6;
  fwu_legend(y, FWU_COLOR_RETRY,  "retrying");
  y += bFONT_STR_HEIGHT + 6;
  fwu_legend(y, FWU_COLOR_NOCARD, "no SD card: insert it, the update continues");
  y += bFONT_STR_HEIGHT + 6;
  fwu_legend(y, FWU_COLOR_ERROR,  "error, still retrying");
  y += bFONT_STR_HEIGHT + 6;
  fwu_legend(y, FWU_COLOR_DONE,   "done, restarting");
  y = LCD_HEIGHT - 3 * bFONT_STR_HEIGHT - 10;
  lcd_printf(x, y, "If the tinySA does not start after an interrupted update:");
  y += bFONT_STR_HEIGHT + 4;
  lcd_printf(x, y, "hold the jog button while switching on (DFU) and use a PC.");
  lcd_set_font(FONT_SMALL);
  ili9341_bulk_finish();
}

// Program the image prepared by fwu_prepare() and reset, never returns
void fwu_start(void) {
  extern uint32_t __ram4_init_text__[], __ram4_init__[], __ram4_clear__[];
  ili9341_bulk_finish();
  set_SPI_mode(SPI_MODE_SD_CARD);
  usbDisconnectBus(&USBD1);             // the PC sees the tinySA leave instead of a hanging port
  chThdSleepMilliseconds(100);
  adc_stop_analog_watchdog();           // stop the touch ADC
  __disable_irq();
  // Fresh copy of the CCM code from flash
  for (uint32_t *d = __ram4_init__, *s = __ram4_init_text__; d < __ram4_clear__; )
    *d++ = *s++;
  FWU_DSB();
  FWU_ISB();
  fwu_run();
}

#endif // __SD_FIRMWARE_UPDATE__
