# Next Chat — Publish LiveCyberDeck to pulsesensor.com

## Goal

Take the finished **LiveCyberDeck** firmware (PulseSensor dashboard for the Waveshare
ESP32-S3-RLCD-4.2) and **publish a customer-facing pulsesensor.com webpage** for it, with
**continuity to the newest pulsesensor.com project pages**. Do as much as possible **via
Claude Code / CLI** (Shopify MCP, `gh`, web fetch, brand skill).

## What's done (handoff state — 2026-06-15)

- Firmware **`LiveCyberDeck 1.0.3`** is built, working, and hardware-validated on the RLCD.
  (Device was last *flashed* at 1.0.2; 1.0.3 is the sparkline-fill + flat-outline patch on top —
  reflash anytime via the BOOT+RST trick below.)
- **Dev repo:** <https://github.com/yury-g/PulseSensor_RLCD> — branch `claude/dashboard-v3`,
  tag `v1.0.3`. Local clone: `~/Documents/Codex/PulseSensor_RLCD`.
- **Public-facing target repo:** <https://github.com/WorldFamousElectronics/esp32-s3-RLCD-4.2>
  (private; populate + flip public as part of launch).
- The dashboard: large LIVE pulse window (solid waveform, open/filled beat circles, dashed
  `TRIGGER` line), single-line coach footer + strength meter, BPM/IBI panels with edge-to-edge
  qualified-beat sparklines (low/normal/high reference lines), battery % + voltage, SHTC3
  temp/humidity, boot splash, live `UP h:mm:ss` uptime.

## Assets ready to use on the page

- **Hero render:** `docs/screenshots/livecyberdeck-hero.png` (pixel-accurate device screenshot).
- **Wiring art:** `docs/wiring/*.svg` (back-header line art, schematic, block diagram).
- **Draft Shopify page:** `docs/shopify/page-waveshare-rlcd.html` — first draft to evolve.
- **Web installer (ESP Web Tools):** `manifest.json` + `index.html` + `firmware/*.bin` — lets
  customers flash from the browser. Needs hosting (GitHub Pages on the repo is simplest).

## Continuity references

- Newest sibling page to match for tone/structure: **`https://pulsesensor.com/pages/cyd`**.
- Current RLCD draft already live: **`https://pulsesensor.com/pages/waveshare-rlcd-prototype`**.
- Brand: use the **`pulsesensor-brand` skill** — signature is salmon red; **never purple/magenta**.
- Fetch the latest pulsesensor.com project pages first and mirror their layout, headings, and voice.

## CLI superpowers available

- **Shopify MCP** — pulsesensor.com is a Shopify store; you can create/update the actual page
  (and products/collections) directly. Confirm the target store with the user before writing.
- **`gh`** — manage the repos, enable **GitHub Pages** (to host the ESP Web Tools installer),
  cut a **release** with the firmware binaries, and publish to the WFE org repo.
- **Web fetch/search** — study the existing pulsesensor.com pages for continuity.

## Device facts for the page copy

- Board: Waveshare ESP32-S3-RLCD-4.2, ST7305 **400 x 300 monochrome reflective** LCD (no backlight).
- Wiring (rear 2 x 8 female header): Red→`3V3`, Black→`GND`, Purple signal→`GP1`/`GPIO1`/`ADC1_CH0`.
  Do **not** power the sensor from VBUS.
- Optional: single Li-ion battery (shows % + voltage), onboard SHTC3 temp/humidity.
- `KEY`/GPIO18 toggles light/dark.
- **Flashing for customers:** browser web-installer (ESP Web Tools). The board needs manual
  download-mode entry: **hold BOOT → tap RST → release BOOT** (native-USB auto-reset doesn't work).
  Document this clearly for customers.

## Suggested first steps

1. `cd ~/Documents/Codex/PulseSensor_RLCD && platformio run -e waveshare_rlcd42` to confirm the build.
2. Fetch `pulsesensor.com/pages/cyd` + the latest project pages; note shared structure/branding.
3. Decide page host: Shopify page on pulsesensor.com (via MCP) and/or GitHub Pages for the installer.
4. Evolve `docs/shopify/page-waveshare-rlcd.html` into the real page; wire in the hero + installer.
5. Publish the repo to `WorldFamousElectronics/esp32-s3-RLCD-4.2` and flip public when ready.
