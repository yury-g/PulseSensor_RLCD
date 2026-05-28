# PulseSensor RLCD Dashboard

Known-working PulseSensor dashboard for the Waveshare ESP32-S3-RLCD-4.2.

This repo is the small, device-specific branch of the PulseSensor CYD work. It keeps the proven beat-detection behavior from `0.4.41-snappy-lock`, but targets the 400 x 300 monochrome ST7305 reflective LCD instead of the CYD color TFT.

GitHub repo: <https://github.com/yury-g/PulseSensor_RLCD>

## Known Working State

- Board: Waveshare ESP32-S3-RLCD-4.2
- Chip/module: ESP32-S3 with 16 MB flash and 8 MB PSRAM
- Display: ST7305 400 x 300 monochrome reflective LCD
- Display pins: `SCK=11`, `MOSI=12`, `DC=5`, `CS=40`, `RST=41`
- PulseSensor signal: rear female header `GP1` / `GPIO1` / `ADC1_CH0`
- Buttons known on board: `BOOT/GPIO0`, `KEY/GPIO18`
- Serial/upload port used during validation: `/dev/cu.usbmodem2101`
- Firmware version string: `0.4.41-snappy-lock-rlcd`
- Hardware result: flashed and confirmed working on the RLCD device on May 27, 2026.

## Wiring

Use the back 2 x 8 female header.

| PulseSensor wire | Waveshare header pin |
| --- | --- |
| Red / VCC | `3V3` |
| Black / GND | `GND` |
| Purple / Signal | `GP1` / `GPIO1` / `ADC1_CH0` |

Do not power the PulseSensor from `VBUS` for this firmware. Use `3V3` so the analog output stays inside the ESP32-S3 ADC range.

The purple signal wire goes to the lower-row `GP1` socket. In the photo orientation used during testing, that is the fourth lower socket from the left.

See:

- [Line art GP1 header](docs/wiring/back-header-line-art-gp1.svg)
- [Schematic wiring](docs/wiring/pulsesensor-rlcd-schematic.svg)
- [Block diagram](docs/wiring/pulsesensor-rlcd-block-diagram.svg)
- [Rendered RLCD dashboard hero](docs/screenshots/rlcd-dashboard-hero.svg)

## Build And Flash

```sh
/Users/narwhal2/.platformio/penv/bin/platformio run -e waveshare_rlcd42
/Users/narwhal2/.platformio/penv/bin/platformio run -e waveshare_rlcd42 -t upload
```

Serial monitor:

```sh
/Users/narwhal2/.platformio/penv/bin/platformio device monitor -p /dev/cu.usbmodem2101 -b 115200
```

## Signal Behavior

The foreground loop keeps `readPulseSensor()` as the first meaningful call after the readiness check. The firmware still uses:

- `analogReadResolution(10)`
- PulseSensor Playground `2.1.1`
- `getLatestSample()`
- `sawStartOfBeat()`
- `isInsideBeat()`
- `getBeatsPerMinute()`
- `getInterBeatIntervalMs()`
- `getPulseAmplitude()`
- dynamic threshold re-arm behavior from the CYD `0.4.41-snappy-lock` favorite

## Web Page Draft

The Shopify-style page mock for this device is in:

- [docs/shopify/page-waveshare-rlcd.html](docs/shopify/page-waveshare-rlcd.html)

Use it as the first draft for a future `pulsesensor.com/pages/waveshare-rlcd` tutorial page.

This repo also includes `manifest.json`, `index.html`, and `firmware/` artifacts for an ESP Web Tools installer once GitHub Pages is enabled.

## License

MIT. Based on the PulseSensor CYD firmware by World Famous Electronics.
