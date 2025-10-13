/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include <string.h>
#include "usbcfg.h"
static int flash_wait_for_last_operation(void)
{
  while (FLASH->SR == FLASH_SR_BSY) {
    //WWDG->CR = WWDG_CR_T;
  }
  return FLASH->SR;
}

static void flash_erase_page0(uint32_t page_address)
{
  flash_wait_for_last_operation();
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = page_address;
  FLASH->CR |= FLASH_CR_STRT;
  flash_wait_for_last_operation();
  FLASH->CR &= ~FLASH_CR_PER;
}

int flash_erase_page(uint32_t page_address)
{
  chSysLock();
  flash_erase_page0(page_address);
  chSysUnlock();
  return 0;
}

void flash_program_half_word(uint32_t address, uint16_t data)
{
  flash_wait_for_last_operation();
  FLASH->CR |= FLASH_CR_PG;
  *(__IO uint16_t*)address = data;
  flash_wait_for_last_operation();
  FLASH->CR &= ~FLASH_CR_PG;
}

void flash_unlock(void)
{
  // unlock sequence
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}

uint32_t
checksum(const void *start, size_t len)
{
  uint32_t *p = (uint32_t*)start;
  uint32_t *tail = (uint32_t*)(start + len);
  uint32_t value = 0;
  while (p < tail) {
    value = __ROR(value, 31) + *p;
//    if (SDU1.config->usbp->state == USB_ACTIVE) shell_printf("%x, %x\r\n", *p, value);
    p++;
  }
  return value;
}


int
config_save(void)
{
  uint16_t *src = (uint16_t*)&config;
  uint16_t *dst = (uint16_t*)SAVE_CONFIG_ADDR;
  int count = sizeof(config_t) / sizeof(uint16_t);


  config.magic = CONFIG_MAGIC;
  config.checksum = checksum(&config, sizeof config - sizeof config.checksum);

  flash_unlock();

  /* erase flash pages */
  flash_erase_page((uint32_t)dst);
#if SAVE_CONFIG_SIZE > FLASH_PAGESIZE       // two flash pages
  flash_erase_page(((uint32_t)dst) + FLASH_PAGESIZE);
#endif

  /* write to flash */
  while (count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }

  return 0;
}

int
config_recall(void)
{
  const config_t *src = (const config_t*)SAVE_CONFIG_ADDR;
  void *dst = &config;

  if (src->magic != CONFIG_MAGIC)
    return -1;
//  if (SDU1.config->usbp->state == USB_ACTIVE) shell_printf("Checksum %x\r\n", src->checksum);
  if (checksum(src, sizeof config - sizeof config.checksum) != src->checksum)
    return -1;

  /* duplicated saved data onto sram to be able to modify marker/trace */
  memcpy(dst, src, sizeof(config_t));

#ifdef __USE_SD_CARD__
  // Migrate from old config without sd_icon_save member
  if (config.sd_icon_save == 0)
    config.sd_icon_save = SDIS_DEFAULT;
#endif // __USE_SD_CARD__

  return 0;
}


//int16_t lastsaveid = 0;

int
caldata_save(uint16_t id)
{
  if (id >= SAVEAREA_MAX)
    return -1;
  uint16_t *src = (uint16_t*)&setting;
  uint16_t *dst;
  int count = sizeof(setting_t) / sizeof(uint16_t);

  dst = (uint16_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);

  setting.magic = SETTING_MAGIC;
  setting.checksum = 0x12345678;
  setting.checksum = checksum(
      &setting,
//      (sizeof (setting)) - sizeof setting.checksum
      (void *)&setting.checksum - (void *) &setting
      );

  flash_unlock();

  /* erase flash pages */
  void *p = dst;
  void *tail = p + sizeof(setting_t);
  while (p < tail) {
    flash_erase_page((uint32_t)p);
    p += FLASH_PAGESIZE;
  }

  /* write settings to flash */
  while (count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }
#if 0
  // Flash stored trace to flash
  count = sizeof(stored_t) /  sizeof(uint16_t);
  src = (uint16_t*)&stored_t[0];
  while (count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }
#endif
  /* after saving data, make active configuration points to flash */
//  active_props = (setting_t*)saveareas[id];
//  lastsaveid = id;

  return 0;
}

setting_t *
caldata_pointer(uint16_t id)
{
  setting_t *src;

  if (id >= SAVEAREA_MAX)
    return NULL;

  // point to saved area on the flash memory
  src = (setting_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);

  if (src->magic != SETTING_MAGIC)
    return NULL;
//  if (SDU1.config->usbp->state == USB_ACTIVE) shell_printf("Checksum %x\r\n", src->checksum);
  if (checksum(src,
//               (sizeof (setting)) - sizeof src->checksum
               (void *)&setting.checksum - (void *) &setting
               ) != src->checksum)
    return NULL;
  return src;
}

int
caldata_recall(uint16_t id)
{
  setting_t *src;
  void *dst = &setting;

  if (id >= SAVEAREA_MAX)
    return -1;

  // point to saved area on the flash memory
  src = (setting_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);

  if (src->magic != SETTING_MAGIC)
    return -1;
//  if (SDU1.config->usbp->state == USB_ACTIVE) shell_printf("Checksum %x\r\n", src->checksum);
  if (checksum(src,
//               (sizeof (setting)) - sizeof src->checksum
               (void *)&setting.checksum - (void *) &setting
               ) != src->checksum)
    return -1;

  /* active configuration points to save data on flash memory */
//  active_props = src;
//  lastsaveid = id;

  /* duplicated saved data onto sram to be able to modify marker/trace */
  memcpy(dst, src, sizeof(setting_t));
#if 0
  // Restore stored trace
  src = &(src[1]);
  memcpy(stored_t, src, sizeof(stored_t));
#endif
  update_min_max_freq();
  update_frequencies();
  set_scale(setting.scale);
  set_reflevel(setting.reflevel);
  set_level_meter_or_waterfall();
  update_rbw();
  sweep_mode = SWEEP_ENABLE;
#ifdef __ULTRA__
  ultra_start = (config.ultra_start == ULTRA_AUTO ? ULTRA_THRESHOLD : config.ultra_start);
#endif

//  if (setting.show_stored)
//    enableTracesAtComplete(TRACE_STORED_FLAG);
  return 0;
}
#if 0
const setting_t *
caldata_ref(int id)
{
  const setting_t *src;
  if (id < 0 || id >= SAVEAREA_MAX)
    return NULL;
  src = (const setting_t*)saveareas[id];

  if (src->magic != SETTING_MAGIC)
    return NULL;
  if (checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return NULL;
  return src;
}
#endif

void
clear_all_config_prop_data(void)
{
  flash_unlock();

  /* erase flash pages */
  void *p = (void*)SAVE_CONFIG_ADDR;
  void *tail = p + SAVE_CONFIG_AREA_SIZE;
  while (p < tail) {
    flash_erase_page((uint32_t)p);
    p += FLASH_PAGESIZE;
  }
}

