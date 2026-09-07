# ChibiOS 21.11.5 port

This branch ports both firmware targets from the historical tinySA ChibiOS
fork to the official ChibiOS `ver21.11.5` release.

The draft currently pins the public integration commit
`db35f6df137058c612dc13e4488ac923219d881a` from
`PhysicistJohn/chibios`. It contains two focused commits on top of the release
tag:

1. restore the standalone STM32F0 TIM14 GPT interrupt service;
2. preserve active USBv1 endpoint-zero PMA buffers when configuration
   endpoints are rebuilt.

Those fixes are also proposed independently to current ChibiOS in PRs #84 and
#85 and to `stable-21.11.x` in draft PRs #86 and #87. The temporary fork URL
will be replaced with the canonical ChibiOS URL and merged stable commit before
this port is marked ready.

## Port changes

- Adopt the RT7/HAL9 configuration markers, OS library settings, build rules,
  Cortex-M port paths, and license include required by ChibiOS 21.11.5.
- Raise kernel-aware F303 interrupt priorities to respect the RT7
  fast-interrupt reservation.
- Migrate board GPIO initialization, DMA allocation, ADC group definitions,
  PAL line events, USB serial hooks, endpoint configuration, PWM configuration,
  queue reset calls, time conversions, and thread diagnostics.
- Update the project-local F303 ADC LLD while retaining its tinySA-specific
  behavior.
- Convert custom `BaseSequentialStream` VMTs to the current layout, including
  the required `instance_offset` field.
- Preserve the legacy hard-FPU build's `-fsingle-precision-constant` behavior
  explicitly. ChibiOS 21.11.x no longer adds it automatically; omitting it
  promotes unsuffixed sweep constants to software double precision and causes
  a measurable self-test sweep regression.
- Split the F303 HardFault entry into an assembly-only MSP/PSP veneer and an
  ordinary non-returning C reporter, while reserving a 1 KiB exception stack.

## Build

With an Arm GNU toolchain on `PATH`:

```sh
git submodule update --init --recursive
make TARGET=F303 -j8
make clean
make TARGET=F072 -j8
```

## Qualification boundary

The predecessor RC5 image using the same application port and equivalent
ChibiOS fixes was exercised on a tinySA Ultra+ ZS407. Its exact DFU write and
readback, USB runtime, warm reset retention, cold boot, and all fourteen
built-in self-tests passed. A fresh official-versus-candidate comparison had
one first-cold measured-level threshold failure in case 2; three repeats,
including a second cold run, passed. Physical forced-fault injection was not
performed.

The clean public branch has different commit identity and embedded version, so
it is a new binary. It must be rebuilt and receive a final physical smoke and
self-test run before this draft is promoted to ready.
