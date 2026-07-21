# Del Sol Head Unit — Settings App

A phone settings page for the custom Honda Del Sol ESP32 head unit. Talks to the radio
over **Bluetooth LE** (Web Bluetooth), so it tunes the EQ and settings live *while music
streams over A2DP* — the two Bluetooth links share one controller and don't fight for the
antenna the way Wi-Fi would.

- **Equaliser** — 10-band graphic EQ with a live response curve and presets. Boosting a
  band lowers everything else by the same amount, so shaping tone never changes loudness.
- **Playing** — a mirror of the unit's OLED, track progress, and transport controls.
- **Settings** — every on-unit setting, grouped and labelled.

## Requirements

- **Chrome on Android.** Web Bluetooth is unsupported on iOS / Safari / Firefox.
- Served over **HTTPS** (GitHub Pages satisfies this). `file://` will not work.

## Install

Open <https://rydonz.github.io/Radio-UI/> in Chrome, then **Add to Home screen**. A service
worker caches it, so it opens offline in the car. Tap **Connect** and pick the radio.

Seamless reconnect (no device picker each launch) needs this flag enabled once on the phone:
`chrome://flags/#enable-web-bluetooth-new-permissions-backend`.

## Firmware

Pairs with `DelSol-Radio.ino` v0.6.0+. The BLE GATT schema (service/characteristic UUIDs
and the packed state struct) is defined in that sketch; the byte offsets in `index.html`
mirror it, and a `static_assert` in the firmware fails the build if the struct layout drifts.
