# Kobra Compatibility Patches

This document lists compatibility additions on branch `anycubic_MCU_adaptions`.

## 1) Added Modules (Anycubic / Kobra)

### CS1237 module
- `src/sensor_cs1237.c`
- `src/sensor_cs1237.h`

### LIS2DW12 module
- `src/sensor_lis2dw12.c` (active module used by `src/Makefile`)

### OTA protocol module
- `src/otacmd.c`
- `src/ota_data.c`
- `src/ota_data.h`

### STM32 flash support used by OTA
- `src/stm32/flash.c`
- `src/stm32/flash.h`
- `src/stm32/printf.c`
- `src/stm32/printf.h`
- `src/stm32/flash_hal/stm32f1xx_hal.c`
- `src/stm32/flash_hal/stm32f1xx_hal_flash.c`
- `src/stm32/flash_hal/stm32f1xx_hal_flash_ex.c`
- `src/stm32/flash_hal/stm32f1xx_hal_cortex.c`
- `src/stm32/flash_hal/stm32f1xx_hal.h`
- `src/stm32/flash_hal/stm32f1xx_hal_flash.h`
- `src/stm32/flash_hal/stm32f1xx_hal_flash_ex.h`
- `src/stm32/flash_hal/stm32f1xx_hal_cortex.h`
- `src/stm32/flash_hal/stm32f1xx_hal_conf.h`
- `src/stm32/flash_hal/stm32f1xx_hal_def.h`
- `src/stm32/flash_hal/stm32_hal_legacy.h`
- `src/stm32/flash_hal/cmsis_armcc.h`
- `src/stm32/flash_hal/stddef.h`
- `src/stm32/flash_hal/stdint.h`

### Debug/log helper files added with OTA bring-up
- `src/stm32/Debug_serial.c`
- `src/generic/Debug_serial_irq.c`
- `src/generic/Debug_serial_irq.h`

### Build/config wiring added for the above modules
- `src/Makefile` (adds `ota_data.c`, `otacmd.c`, `sensor_cs1237.c`, `sensor_lis2dw12.c`)
- `src/stm32/Makefile` (adds flash/HAL/debug objects)
- `src/Kconfig` (adds `CONFIG_FIRMWARE_VERSION`, default tuning)
- `src/stm32/Kconfig` (adds OTA flash layout addresses)
- `scripts/buildcommands.py` (uses `CONFIG_FIRMWARE_VERSION` for firmware version string)

### Parked compatibility shim not currently compiled
- `src/pcbacmds.c`
- Purpose:
  - holds a dummy compatibility implementation for legacy `pcba_read` /
    `pcba_write` protocol commands found in some extracted Anycubic MCU
    dictionaries
- Current state:
  - intentionally **not** referenced by `src/Makefile`
  - intentionally **not** included in the generated command dictionary
  - kept only as a placeholder/reference until an original firmware that
    definitely belongs to this target is reversed and confirmed

## 2) Vanilla Klipper Files Patched for go-klipper Compatibility

### `src/adccmds.c`
- Patched command behavior/signature:
  - go-klipper expected command:  
    `query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c`
  - vanilla command:  
    `query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u bytes_per_report=%c min_value=%hu max_value=%hu range_check_count=%c`
- Patched response behavior:
  - go-klipper mode sends single-value reports:  
    `analog_in_state oid=%c next_clock=%u value=%hu`
  - vanilla mode sends batched data:  
    `analog_in_state oid=%c next_clock=%u values=%*s`
- Switch control:
  - file-local define `ADC_GO_KLIPPER_COMPAT`
  - default: `1` (go-klipper mode active)
  - set to `0` for original vanilla behavior
- Transparency:
  - **Not fully transparent** (protocol behavior differs by define)

### `src/i2c_software.c`
- Added compatibility command:
  - go-klipper command accepted:  
    `i2c_set_software_bus oid=%c scl_pin=%u sda_pin=%u rate=%u address=%u`
  - mapped internally to vanilla behavior (`i2c_set_sw_bus`) by converting  
    `rate -> pulse_ticks = CONFIG_CLOCK_FREQ / rate / 2`
- Original vanilla command kept:
  - `i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u`
- Transparency:
  - **Transparent/additive** (old + new command names both work)

### `src/spi_software.c`
- Added compatibility command:
  - go-klipper command accepted:  
    `spi_set_software_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u rate=%u`
  - mapped internally to vanilla behavior (`spi_set_sw_bus`) by converting  
    `rate -> pulse_ticks = CONFIG_CLOCK_FREQ / rate`
- Original vanilla command kept:
  - `spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u`
- Transparency:
  - **Transparent/additive** (old + new command names both work)

### `src/stepper.c` and `src/Kconfig`
- Added compatibility constant alias for host feature probing:
  - vanilla constant: `STEPPER_STEP_BOTH_EDGE`
  - added alias: `STEPPER_BOTH_EDGE`
- `src/Kconfig` adds alias config symbol:
  - `HAVE_STEPPER_BOTH_EDGE` (defaults to `HAVE_STEPPER_OPTIMIZED_BOTH_EDGE`)
- Why this was done:
  - host code decides whether to request `invert_step=-1` (step on both edges)
    from advertised MCU constants
  - exporting `STEPPER_*BOTH_EDGE` only when edge optimization is actually built
    avoids host/firmware capability mismatch
  - this prevents compatibility/startup issues on builds where
    `WANT_STEPPER_OPTIMIZED_BOTH_EDGE` is disabled
- Default behavior impact vs vanilla:
  - default path remains effectively unchanged because
    `WANT_STEPPER_OPTIMIZED_BOTH_EDGE` is still `y` by default
  - non-default builds differ intentionally: when the option is disabled,
    the firmware no longer advertises `STEPPER_*BOTH_EDGE` capability
- Transparency:
  - **Additive for default configs; intentionally non-transparent for
    non-default configs with edge optimization disabled**

### `src/basecmd.c`
- Exported existing reset path as a host command:
  - `DECL_COMMAND_FLAGS(config_reset, HF_IN_SHUTDOWN, "config_reset")`
- Effect:
  - host can explicitly send `config_reset` while in shutdown handling path
- Transparency:
  - **Transparent/additive**
