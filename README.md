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


This is a fork a klipper, adapter for Kobra K3 / KS1 and similar printers of the "K3" based family from Anycubic.

Klippy changes:
It has additional klippy extras modules added (ported from Anycubics go-klipper) to support the CS1237 Straingauge for probing and LIS2DW12 for resonance measurements.
The klippy part is compatible with the (pretty ancient) V1.4.0 MCU firmware, so its possible to run klippy with this old MCU firmware, but you will get WARNINGS regarding missing commands in the MCU from klippy at startup.

MCU changes:
The Anycubic released MCU parts was also backported by me, so MCU can also rebuild to new/current klipper base (to get rid of the warnings) and still have support for the Anycubic sensors.
However: Anycubic has NOT released all MCU changes they did, so two commands in the CS1237 are missing needed for Automatic flow / PA calibration with go-klipper, so automatic flow calibration wont works anymore.

This port tries to keep this MCU changes compatible with GO-Klipper as much as possible, so even it is a newer MCU base, its still 95% compatible with GO-Klipper (Flow calibration wont work).
I ported also the OTA part, so its possible to flash via the Anycubic MCU OTA mechanism the MCU.

So the MCU can be used with vanilla-klipper as well with the ancient GO-Klipper, however some sacrifices had to be made to make that possible, see KOBRA_COMPATIBILITY_PATCHES.md.
