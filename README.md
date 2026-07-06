Welcome to the (Kobra-S1 adapted) Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).


This is a fork of klipper, adapter for Kobra K3 / KS1 and similar printers of the "K3" based family from Anycubic.

Klippy changes:
It has additional klippy extras modules added (ported from Anycubic's go-klipper) to support the CS1237 strain gauge for probing and LIS2DW12 for resonance measurements.
The klippy part is compatible with the (pretty ancient) V1.4.0 MCU firmware, so it's possible to run klippy with this old MCU firmware, but you will get WARNINGS regarding missing commands in the MCU from klippy at startup.

MCU changes:
The Anycubic released MCU parts were also backported by me, so MCU can also rebuild to new/current klipper base (to get rid of the warnings) and still have support for the Anycubic sensors.
However: Anycubic has NOT released all MCU changes they did, so two commands in the CS1237 are missing needed for Automatic flow / PA calibration with go-klipper, so automatic flow calibration won't work anymore.

This port tries to keep this MCU changes compatible with GO-Klipper as much as possible, so even though it is a newer MCU base, it's still 95% compatible with GO-Klipper (Flow calibration won't work).
I ported also the OTA part, so it's possible to flash via the Anycubic MCU OTA mechanism the MCU.

So the MCU can be used with vanilla-klipper as well with the ancient GO-Klipper, however some sacrifices had to be made to make that possible, see KOBRA_COMPATIBILITY_PATCHES.md.

The folder mcu_build/ contains a kobra compatible MCU build for reference.

## Build firmware

### STM32 MCU

The toolchain installer is intended for debian based systems (like e.g. Raspbian, Ubuntu). On 
other distributions, install the equivalent `arm-none-eabi-gcc` and build
dependencies with your distribution package manager instead.

```bash
./install-mcu-build-toolchain.sh
rm -f .config
make menuconfig
./build.sh --clean
```

### Linux MCU

In `make menuconfig` set:

- `Micro-controller Architecture` = `Linux process`
- `GPIO pins to set at micro-controller startup` should be no set (empty string)

Then build and flash:

```bash
rm -f .config
make menuconfig
make clean
make flash
```

## MCU OTA Flashing

The `mcu_ota_flasher.py` script allows you to flash the MCU firmware on the printer via Anycubic's proprietary over-the-air (OTA) protocol.

It implements the Anycubic MCU OTA mechanism to transfer the build MCU binary to the MCU / Toolhead MCU.

Anycubic is not using Katapult bootloader by default, but a proprietary one.

Anycubic's MCU OTA code writes the new MCU data first into a passive flash section and validates the MCU CRC (to detect transfer issues, it does NOT validate if the MCU you send is guaranteed compatible/valid, only that the binary was transferred okay via serial line and it contains the AC propitary CRC at the end is checked).

If that looks good it will then at next reset/powercyle copy the MCU code from the passive to the active flash bank and execute it.

### ⚠️ Warning
- **Ensure nothing is using the MCU when trying to flash, stop klipper service first** before flashing via OTA

- **Ensure you are trying to flash the right binary which is compatible to your MCU** - If you flash a MCU which crashes, the OTA thread can't run either -> Softbricked MCU, requires recovery flashing via hardware SWD interface/programmer hardware.
- **Do not interrupt the flashing process** - a failed transfer/flash try should be handled by the CRC check and MCU will just start with the old MCU code. But you never now, still better to not interrupt
- **Use this method only if you understand the risks** - consider using traditional JTAG/SWD flashing as a safer alternative if available

- **Sometimes script can't connect/initialize properly to the MCU** - If that happens, you need to reset the MCU first. On KS1 this can be done via ssh into the KS1 remote shell and execute:

```bash
echo 116 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio116/direction
sleep 1
echo 1 > /sys/class/gpio/gpio116/value
```

This will reset both MCUs. Then try again to execute the flash script.

If you have a KS3, no such GPIO is known, but at least for the toolhead MCU you can just shortly unplug and re-plug the USB-C style connector at the toolhead to reset the MCU.

If you want to immediatly get the MCUs to transfer the new flashed image from passive to active bank, you can use again the above mentioned reset sequence.
Give after the reset (or a power-cycle) the MCUs a few seconds time, so they can finish the flash copy. If that happens with the main MCU, than you have bad luck, maybe just retry then by power cycling the printer or use the flashing option via go-klipper itself:

Alternativ Go-Klipper MCU flashing method (requires rinkhals or equivalent installed on the printer):

Copy the binary to the userdata partition on the printer.

Open mainsail/fluidd and then execute the following command to flash the mcu:

OTA_START MCU=mcu UPDATE_PATH=/userdata/<name of firmware binary>.bin

(For nozzle_mcu update use MCU=nozzle_mcu, gklib/mainsail console output may or may not show progress percentage during flashing which takes only a few seconds)






### Usage Example

```bash
python3 mcu_ota_flasher.py --port /dev/ttyGS0 --baudrate 115200 --firmware mcu_build/out/klipper.bin
```

**Parameters:**
- `--port`: Serial port connected to the MCU (e.g., `/dev/ttyGS0`)
- `--baudrate`: Serial communication speed (default: 576000)
- `--firmware`: Path to the compiled firmware binary file

### Example flash output of a KS1 nozzle_mcu flashing sequence
```bash
biqu@bigtreetech-cb2:~/klipper$ python3 mcu_ota_flasher.py --firmware ./out/firmware_v2.0.10_20260706.bin --port /dev/ttyGS1

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!  WARNING - MCU / PRINTER FIRMWARE OTA FLASH - READ BEFORE PROCEEDING  !!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  This tool flashes new firmware directly onto the printer's MCU via the
  Anycubic serial/OTA MCU protocol.

  THIS CAN PERMANENTLY BRICK YOUR MCU / PRINTER MAINBOARD.

  If the flash fails partway through, the new firmware build is incompatible
  with this board, or the new firmware crashes/hangs the MCU, THE BOARD MAY
  NO LONGER BOOT and could require an SWD/JTAG hardware debugger (and
  opening/disassembling the printer) to recover it.

  YOU are solely responsible for verifying the firmware you are about to
  flash and that you understand what this script does
  before running it.

  USE THIS SCRIPT ENTIRELY AT YOUR OWN RISK. If it breaks your MCU or your
  printer, that is your responsibility - don't cry about it, it's on you.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Type 'Yes' (exactly) to acknowledge and continue (anything else aborts): Yes


=== Preparing Firmware ===
File size: 37985 bytes
CRC in firmware: 0x33DF3EED
CRC calculated: 0x33DF3EED
CRC verified

Version from klipper.dict: 2.0.10
Opening /dev/ttyGS1 at 576000 baud...

=== Querying MCU dictionary ===
WARNING: Identify attempt 1/10 made no progress, saw 1 ACK
   Resyncing sequence counter to MCU-reported value 7
Dictionary: 2764 bytes (70 chunks)
==================================

dict_vv2.0.10 loaded

Continuing with seq=13 after dictionary query
=== Startup Preflight ===
MCU config state:
   is_config=1 is_shutdown=0 crc=0x20934A55 move_count=1024

MCU is already configured. Trying reset and reconnect...

=== Querying MCU dictionary ===
WARNING: Identify attempt 1/10 made no progress, saw 1 unrelated response
Dictionary: 2764 bytes (70 chunks)
==================================

dict_vv2.0.10 loaded

Reconnected after startup reset

MCU config state:
   is_config=0 is_shutdown=0 crc=0x00000000 move_count=0

  -> allocate_oids (cmd_id=9, params=[1])
  -> config_ota (cmd_id=61, params=[0])
  -> finalize_config (crc=0x20934A55)

  Verifying config via get_config...
  Config state: is_config=1 is_shutdown=0 crc=0x20934A55
Config sent

MCU OTA state:
   flag=0 version=2.0.10 CRC32=0x33DF3EED

=== Starting OTA ===
Version: 2.0.10
CRC32: 0x33DF3EED
Size: 37985 bytes

Sending ota_start...
Sending ota_erase...

OTA Status: START (offset=0, err_code=0)
Transfer: 100% (37985/37985)
EOF reached at offset 37985, sending empty data for CRC check

============================================================
MCU CRC check:
   Version: 2.0.10
   CRC from FW: 0x33DF3EED
   CRC calculated: 0x33DF3EED
   CRC match - OTA data will be saved
============================================================

============================================================
MCU saved OTA data successfully:
   app_flag: 1 (2=APP_DOWNLOAD, will update on reboot)
   Version: 2.0.10
   CRC32: 0x33DF3EED
   Offset: 37985 bytes (0x9461)
============================================================

OTA Status: FINISH (offset=37985, err_code=0)

============================================================
OTA complete - firmware staged successfully
============================================================

State is 'finished', exiting loop


============================================================
OTA update complete
   Final state: finished
   Final progress: 100.0%
   Bytes transferred: 37985/37985
============================================================

OTA script complete.

Reset the MCU manually to apply the staged firmware.
```