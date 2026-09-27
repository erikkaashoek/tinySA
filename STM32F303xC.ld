/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * STM32F303xC memory setup.
 */
MEMORY
{
    flash0  : org = 0x08000000, len = 240k
    flash1  : org = 0x00000000, len = 0
    flash2  : org = 0x00000000, len = 0
    flash3  : org = 0x00000000, len = 0
    flash4  : org = 0x00000000, len = 0
    flash5  : org = 0x00000000, len = 0
    flash6  : org = 0x00000000, len = 0
    flash7  : org = 0x0803C000, len = 16k
    ram0    : org = 0x20000000, len = 40k
    ram1    : org = 0x00000000, len = 0
    ram2    : org = 0x00000000, len = 0
    ram3    : org = 0x00000000, len = 0
    ram4    : org = 0x10000000, len = 8k
    ram5    : org = 0x00000000, len = 0
    ram6    : org = 0x00000000, len = 0
    ram7    : org = 0x00000000, len = 0
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* Flash region to be saved calibration data */
REGION_ALIAS("CALDATA_FLASH", flash7);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* Generic rules inclusion. The code rules of rules.ld (rules_code.ld) are inlined
   below so that the firmware image header (.fwinfo) directly follows the vectors.*/
INCLUDE rules_stacks.ld

ENTRY(Reset_Handler)

SECTIONS
{
    vectors : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    /* Firmware image header (fw_update.c), used by the firmware update from the SD card */
    .fwinfo : ALIGN(4)
    {
        KEEP(*(.fwinfo))
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    xtors : ALIGN(4) SUBALIGN(4)
    {
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
        __fini_array_start = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } > XTORS_FLASH AT > XTORS_FLASH_LMA

    .text : ALIGN(4) SUBALIGN(4)
    {
        *(.text)
        *(.text.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > TEXT_FLASH AT > TEXT_FLASH_LMA

    .rodata : ALIGN(4)
    {
        . = ALIGN(4);
        __rodata_base__ = .;
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > RODATA_FLASH AT > RODATA_FLASH_LMA

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
     } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA
}

INCLUDE rules_data.ld

SECTIONS
{
    /* Firmware image trailer (fw_update.h fw_trailer_t), must be the last data in flash:
       end magic "FEND", image size, CRC32 (0xFFFFFFFF = not set) */
    .fwtrailer : ALIGN(4)
    {
        LONG(0x444E4546)
        LONG(__fw_image_size__)
        LONG(0xFFFFFFFF)
        __fw_image_end__ = .;
    } > flash0

    .calsave (NOLOAD) : ALIGN(4)
    {
        *(.calsave)
    } > CALDATA_FLASH
}

/* Size of the firmware image (.bin file) including the trailer */
__fw_image_size__ = __fw_image_end__ - ORIGIN(flash0);

ASSERT(ADDR(.fwinfo) == ORIGIN(flash0) + SIZEOF(vectors), "firmware header must follow the vectors")
ASSERT(LOADADDR(.fwtrailer) >= LOADADDR(.data) + SIZEOF(.data), "firmware trailer must be the last data in flash")
ASSERT(LOADADDR(.fwtrailer) >= LOADADDR(.ram4_init) + SIZEOF(.ram4_init), "firmware trailer must be the last data in flash")
