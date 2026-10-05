# topotek-kht30b-control

Python UDP control library and command-line tool for the **Topotek KHT30B** gimbal camera
(pointing, gyro/encoder readout, zoom, recording, laser rangefinder, thermal data).
No dependencies beyond the Python standard library.

> **Status: early / work in progress.** Core gimbal control is verified on real hardware;
> several features are untested or had no visible effect on my unit (see table). A fuller README is coming.

## Quick start

```bash
# camera on 192.168.144.108, close the vendor GroundStation app first (it uses UDP port 9004)
python kht30b.py attitude --json        # gyro + encoder angles
python kht30b.py goto 30 10 --wait      # yaw 30 (right +), pitch 10 (down +), closed-loop
python kht30b.py zoom in 3
python kht30b.py rec on
python kht30b.py lrf single
python kht30b.py info                   # read everything the device answers
python kht30b.py -h                     # all commands
```

From Python:

```python
from kht30b import KHT30B
with KHT30B() as g:
    print(g.attitude("gyro"))
    g.goto_wait(30, 10)
```

Device IP: `KHT_IP` environment variable (default `192.168.144.108`), device port 9003, local port 9004.

## Status (tested on one KHT30B unit)

| Status | Commands |
|---|---|
| Verified on device | `attitude`, `goto` (gyro/encoder, `--wait`), `rate`, `home`, `stop`, `motor`, `zoom`, `day`/`night`, `pip`, `palette`, `res`, `codec`, `rec`, `snap`, `sd`, `lrf`, `temps` |
| No visible effect on my unit | `focus`, `dzoom`, `mag <x>`, `rotate`, `gps`/`uav` (target coordinates not computed) |
| Not yet tested on device | `ymode`, `calibrate`, `bitrate`, `focuspos`, `zfp`, `aperture`, `fog`, `trange`, `tpoint`, `gpstext`, `synctime` |

Notes from testing: yaw + = right, pitch + = down (the vendor document says otherwise for angle mode);
gyro-mode yaw undershoots 12-17 %, which `goto_wait` corrects with a closed loop;
the device sends every packet twice and answers gyro reads with identifier `GIA`.

## Tests

```bash
python -m unittest discover tests        # no device needed
python tools/test_doc.py                 # interactive device test (gimbal moves!)
```

## Safety

Keep the gimbal's path clear. Never point the laser at eyes or reflective surfaces.
`motor off` makes the gimbal go limp.

## Disclaimer

Unofficial. Not affiliated with Topotek. Protocol behaviour comes from the vendor's
public UDP&UART protocol document and from experiments on real hardware; the document itself is not included.

License: MIT
