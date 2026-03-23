# Copilot Instructions for tinySA

## Build Environment

Building requires the ARM GCC toolchain and MSYS2 utilities from ChibiStudio.

Before running `make`, set up the environment by running `C:\ChibiStudio\start_gcc113.bat`.
This configures the following PATH entries:

- `C:\ChibiStudio\tools\msys2\usr\bin` — provides `make`, `bash`, and other Unix utilities
- `C:\ChibiStudio\tools\openocd\bin` — OpenOCD for flashing
- `C:\ChibiStudio\tools\GNU Tools ARM Embedded\11.3 2022.08\bin` — ARM GCC 11.3 toolchain

Or set up the PATH manually in a terminal before building:

```bat
set PATH=C:\ChibiStudio\tools\msys2\usr\bin;%PATH%
set PATH=C:\ChibiStudio\tools\openocd\bin;%PATH%
set PATH=C:\ChibiStudio\tools\GNU Tools ARM Embedded\11.3 2022.08\arm-none-eabi\bin;%PATH%
set PATH=C:\ChibiStudio\tools\GNU Tools ARM Embedded\11.3 2022.08\bin;%PATH%
```

## Building

To build for the **tinySA Ultra** (STM32F303):

```sh
make TARGET=F303
```

To flash:

```sh
make TARGET=F303 dfu flash
```

## Project Structure

- `sa_core.c` / `sa_cmd.c` / `ui.c` — main application logic
- `si4468.c` / `si4432.c` — RF chip drivers
- `main.c` — entry point
- `NANOVNA_STM32_F303/` — board-specific files for the tinySA (F303 target)
- `ChibiOS/` — RTOS

## remote control protocol

See 'PROTOCOL.md' for details on the remote control protocol used by the tinySA.

tinySA is a resource constraint device. All code must use minimum CPU cycles, RAM and ROM. The codebase is written in C, with some assembly for critical sections. C++ is not used to avoid the overhead of exceptions and RTTI.
