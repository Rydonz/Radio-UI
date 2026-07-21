# DelSol Head Unit — Flash Kit

Flashes firmware **v0.8.0** to the head unit over the air. No dash disassembly, no USB.
After this, further updates can be done straight from the phone (Web Update).

## On the laptop, install once

**Just Python 3** — nothing else. <https://www.python.org/downloads/>
During install, tick **"Add python.exe to PATH."** That's the whole setup.

(`espota.py` here is the OTA uploader; it uses only the Python standard library.)

## Flashing

1. Turn on the phone hotspot **"Caleb's S25 Ultra."**
2. Connect the **laptop** to that same hotspot.
3. Put the unit into **OTA mode**: hold **NEXT** while powering it on
   (or Settings → *OTA Update* → ON). The screen shows
   `Connecting…` then an **IP address** — note it.
4. Double-click **`flash.bat`**, type that IP, press Enter.
5. Progress runs to 100%, the unit reboots into the new firmware. Done.

If `flash.bat` won't run, do it by hand in a terminal in this folder:

```text
python espota.py -i <IP-from-screen> -p 3232 -f firmware.bin -r -d
```

## If it fails

- **Recover any time:** power-cycle with **NEXT** held → OTA mode again.
  A failed upload never touches the running firmware, so the unit can't be bricked.
- **"Connect failed" / timeout:** the laptop isn't on the hotspot, or the unit
  dropped off the OTA screen. Re-enter OTA mode and confirm both show the hotspot.
- **`.local` name doesn't resolve:** always use the numeric IP from the screen,
  not `delsol-radio.local` (Windows needs Bonjour for `.local`).

## After flashing — what to check

Open the settings app (<https://rydonz.github.io/Radio-UI/>), connect over Bluetooth,
and watch the footer readouts:

- **Free heap** should sit comfortably above ~30 KB. If it's very low or the unit
  keeps rebooting, BLE + A2DP ran out of memory — tell Caleb's helper and we adjust.
- **DSP load** should be well under 100%.
- Play music, drag the EQ sliders for a couple minutes, and **listen for dropouts** —
  that's the real test of Bluetooth audio + BLE control coexisting.
