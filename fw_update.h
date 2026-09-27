/*
 * Firmware update from the SD card (tinySA ULTRA)
 *
 * The update runs in two parts:
 * - fw_update.c, flash resident: check the image file, map it to SD card sectors and
 *   verify that the sectors can be read (dry run), all while the firmware runs normally.
 * - a small self-contained routine in CCM RAM that, with interrupts off, reads the image
 *   sector by sector from the SD card, erases and programs the application flash, verifies
 *   it and resets. It never touches the config/preset area at SAVE_CONFIG_ADDR and above.
 *
 * Image format: the linker script (STM32F303xC.ld) places fw_header_t directly after the
 * vector table and a fw_trailer_t at the end of the image. Only images with this header can
 * be installed from the SD card; older images and other models need DFU.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 */
#ifndef __FW_UPDATE_H__
#define __FW_UPDATE_H__

#include <stdint.h>
#include <stdbool.h>

#define FW_HEADER_MAGIC0    0x796E6974U   // "tiny"
#define FW_HEADER_MAGIC1    0x57464153U   // "SAFW"
#define FW_HEADER_VERSION   1
#define FW_MODEL_TINYSA4    4             // tinySA ULTRA / ULTRA+ (STM32F303)
#define FW_TRAILER_MAGIC    0x444E4546U   // "FEND"
#define FW_CRC_UNSET        0xFFFFFFFFU   // trailer CRC not set by the build

// Image header, placed after the vector table. Newer versions only append fields.
typedef struct {
  uint32_t magic[2];      // FW_HEADER_MAGIC0, FW_HEADER_MAGIC1
  uint16_t hdr_version;   // FW_HEADER_VERSION
  uint16_t hdr_size;      // sizeof(fw_header_t)
  uint32_t model;         // FW_MODEL_TINYSA4
  uint32_t load_addr;     // FLASH_START_ADDRESS
  uint32_t image_size;    // bytes from load_addr up to and including the trailer
  uint32_t flags;         // reserved, 0
  char     version[48];   // VERSION string, zero terminated
} fw_header_t;

// Image trailer, the last 12 bytes of the image
typedef struct {
  uint32_t magic;         // FW_TRAILER_MAGIC
  uint32_t image_size;    // same as fw_header_t.image_size
  uint32_t crc;           // CRC32 (zlib) of the image without this word, or FW_CRC_UNSET
} fw_trailer_t;

// Error codes
enum {
  FWU_OK = 0,
  FWU_E_BUSY,       // a shell command is still running
  FWU_E_BATTERY,    // battery too low
  FWU_E_NOCARD,     // no SD card or card error
  FWU_E_OPEN,       // file not found
  FWU_E_READ,       // file read error
  FWU_E_FORMAT,     // no firmware header: not a firmware file or an older version
  FWU_E_MODEL,      // firmware for another model
  FWU_E_SIZE,       // image size not valid or larger than the file
  FWU_E_VECTOR,     // vector table not valid
  FWU_E_TRAILER,    // end of image not found: incomplete file
  FWU_E_CRC,        // CRC mismatch
  FWU_E_FRAG,       // file too fragmented
  FWU_E_DRYRUN,     // raw sector read check failed
  FWU_E_COUNT
};

extern const fw_header_t fw_header;          // header of the running firmware
extern volatile bool fwu_active;             // firmware update in progress, the shell refuses commands

const char *fwu_error_text(int err);
int  fwu_begin(void);                        // take SPI1/SD card for the update (sweep thread)
void fwu_end(void);                          // give them back after a check or a cancel
int  fwu_read_header(const char *path, fw_header_t *hdr);
int  fwu_prepare(const char *path, const fw_header_t *hdr);  // full file check, sector map and dry run
uint32_t fwu_image_crc(void);                // CRC32 of the prepared image
uint32_t fwu_extent_count(void);             // number of contiguous sector runs of the prepared image
bool fwu_card_byte_addressing(void);         // prepared card uses byte addresses (SDSC)
uint32_t fwu_running_crc(void);              // CRC32 of the running firmware image
void fwu_draw_screen(const fw_header_t *hdr); // update screen, before fwu_start()
void fwu_start(void) __attribute__((noreturn));  // program the prepared image and reset

#endif
