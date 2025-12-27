##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

#Build target
ifeq ($(TARGET),)
  TARGET = F072
else
  TARGET = F303
endif

# Compiler options here.
ifeq ($(USE_OPT),)
 ifeq ($(TARGET),F303)
USE_OPT = -Og -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage -std=gnu11
#USE_OPT+=-fstack-protector-strong
 else
USE_OPT = -Og -fno-inline-small-functions -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nano.specs -fstack-usage -fsingle-precision-constant 
 endif
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = --print-memory-usage
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

# format va.b-n-gxxxxxxx
# or     va.b-nn-gxxxxxxx
# or     va.b-nnn-gxxxxxxx
# or     ...

ifeq ($(VERSION),)
  VERSION="$(PROJECT)_$(shell git describe --tags --long)"
endif

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x220
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x100
endif

ifeq ($(TARGET),F303)
  USE_FPU = hard
  USE_PROCESS_STACKSIZE = 0x480
  USE_EXCEPTIONS_STACKSIZE = 0x200
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
ifeq ($(TARGET),F303)
PROJECT = tinySA4
else
PROJECT = tinySA
endif

# Imported source files and paths
#CHIBIOS = ../ChibiOS-RT
CHIBIOS = ChibiOS
PROJ = .
# Startup files.

ifeq ($(TARGET),F303)
 include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk
 include $(CHIBIOS)/os/hal/hal.mk
 include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
 include NANOVNA_STM32_F303/board.mk
else
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f0xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F0xx/platform.mk
include NANOVNA_STM32_F072/board.mk
endif

include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
ifeq ($(TARGET),F303)
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
else
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
endif
# Other files (optional).
#include $(CHIBIOS)/test/rt/test.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
#include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
#LDSCRIPT= $(STARTUPLD)/STM32F072xB.ld
ifeq ($(TARGET),F303)
 LDSCRIPT= STM32F303xC.ld
else
 LDSCRIPT= STM32F072xB.ld
endif

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
ifeq ($(TARGET),F303)
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(STREAMSSRC) \
       FatFs/ff.c \
       FatFs/ffunicode.c \
       usbcfg.c \
       NANOVNA_STM32_F303/adc.c \
       main.c plot.c ui.c ili9341.c tlv320aic3204.c si5351.c numfont20x22.c Font5x7.c Font10x14.c flash.c si4468.c  Font7x13b.c rtc.c
else
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(STREAMSSRC) \
       usbcfg.c \
       NANOVNA_STM32_F072/adc.c \
       main.c plot.c ui.c ili9341.c numfont20x22.c Font5x7.c Font10x14.c flash.c si4432.c  Font7x13b.c
endif

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC)  \
         $(STREAMSINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

ifeq ($(TARGET),F303)
 MCU  = cortex-m4
else
 MCU  = cortex-m0
endif

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
ELF  = $(CP) -O elf

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
ifeq ($(TARGET),F303)
 UDEFS = -DARM_MATH_CM4 -DVERSION=\"$(VERSION)\" -DTINYSA_F303 -D__FPU_USED -DST7796S -DTINYSA4
#Enable if install external 32.768kHz clock quartz on PC14 and PC15 pins on STM32 CPU
UDEFS+= -DVNA_USE_LSE
# Use R as usb pullup
UDEFS+= -DUSB_DP_R_VDD
#-DCH_DBG_STATISTICS 
else
UDEFS = -DSHELL_CMD_TEST_ENABLED=FALSE -DSHELL_CMD_MEM_ENABLED=FALSE -DARM_MATH_CM0 -DVERSION=\"$(VERSION)\" -DTINYSA_F072 -DTINYSA3
endif

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk
#include $(CHIBIOS)/memory.mk


ifeq ($(TARGET),F303)
clean:
	rm -f -rf build/tinySA4.* build/lst/*.* build/obj/*.*
else
clean:
	rm -f -rf build/$(PROJECT).* build/lst/*.* build/obj/*.*
endif

flash: build/$(PROJECT).bin
	-@printf "reset dfu\r" >/dev/cu.usbmodem401 # mac
	-@printf "reset dfu\r" >/dev/ttyACM0 # linux
	sleep 2
	dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D $<

dfu:	build/$(PROJECT).hex
	-@#c:/work/dfu/HEX2DFU $< build/$(PROJECT).dfu # win
	-@hex2dfu -i $< -o build/$(PROJECT).dfu # mac / linux


TAGS: Makefile
ifeq ($(TARGET),F303)
	@etags *.[ch] NANOVNA_STM32_F303/*.[ch] $(shell find ChibiOS/os/hal/ports/STM32/STM32F3xx ChibiOS/os -name \*.\[ch\] -print) 
else
	@etags *.[ch] NANOVNA_STM32_F072/*.[ch] $(shell find ChibiOS/os/hal/ports/STM32/STM32F0xx ChibiOS/os -name \*.\[ch\] -print) 
endif
	@ls -l TAGS

