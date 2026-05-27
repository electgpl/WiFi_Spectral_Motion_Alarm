# Spectral Motion Detector v3.2-CSI

**Author:** Electgpl

Detect human presence in a room by analyzing the WiFi spectral fingerprint.
Uses an **ESP32-S3** to monitor RSSI changes (with optional CSI subcarrier analysis)
in surrounding WiFi networks caused by people moving through the environment.

## How it works

The human body interacts with 2.4 GHz radio waves (attenuation, reflection, multipath).
When a person moves through a room, they alter the WiFi signals from nearby access points.

1. **Calibration** (60s): Scans all visible WiFi networks and computes a **median** and **noise floor (MAD)** for each AP using robust statistics. APs with noise > 3.5 dB are rejected.
2. **Adaptive threshold self-calibration** (~15s post-calibration): The system settles (5 scans), then measures the quiet ambient score (5 scans) and sets `th_high = quiet * 1.5`, `th_low = quiet * 1.3`.
3. **Monitoring**: Maintains a dual adaptive baseline that slowly tracks gradual environmental changes.
4. **Detection**: Combines three RSSI metrics per AP plus optional CSI:
   - **Deviation** (45%): RSSI difference from long-term baseline, normalized by noise floor
   - **Rate of change** (25%): Sudden RSSI jumps between consecutive scans
   - **Divergence** (10%): Difference between fast and slow adaptive baselines
   - **CSI magnitude deviation** (30% added to total): Per-subcarrier magnitude changes across 64 OFDM subcarriers
5. **Alarm**: Triggers when the combined score exceeds `th_high` in 3 out of 5 consecutive readings, with a 30-second minimum latch time and 10-second cooldown.

## CSI (Channel State Information)

The firmware includes full CSI support using the ESP-IDF CSI API. When enabled:

- Captures magnitude information across **64 OFDM subcarriers** per received frame
- Maintains per-subcarrier fast and slow baselines (EMA alpha 0.15 / 0.05)
- Computes per-AP CSI deviation and profile (subcarrier-by-subcarrier) deviation
- CSI contributes **30%** to the total detection score
- CSI weights: deviation 60%, profile deviation 40%

### CSI reception modes

| Mode | Description | CSI packet rate |
|---|---|---|
| **Promiscuous-only** (default) | Unassociated STA, captures beacon frames from surrounding APs | ~1 packet/100ms per AP |
| **Associated** | STA connects to a known AP, captures data + beacon frames | ~10-100+ packets/s |

For best results, configure Wi-Fi association in `idf.py menuconfig` > "Wi-Fi Association Configuration".

## Hardware requirements

| Component | Details |
|---|---|
| ESP32-S3 DevKit | Any model with USB-UART (e.g., ESP32-S3-DevKitC-1) |
| LED (optional) | Connect to **GPIO 2** (GND -> LED -> 220 Ohm -> GPIO2) |
| Buzzer (optional) | Connect to **GPIO 10** (GND -> Buzzer -> GPIO10) |

### GPIO pinout

| Pin | Function | Behavior |
|---|---|---|
| GPIO 2 | Status LED | Blinks during calibration, solid ON when armed |
| GPIO 10 | Buzzer output | HIGH during alarm, LOW when clear |

## Quick start

### 1. Build and flash the firmware

```powershell
cd firmware
idf.py set-target esp32s3
idf.py menuconfig      # Optional: configure Wi-Fi credentials for CSI
idf.py build
idf.py -p COM4 flash
```

Or flash pre-built binaries:

```powershell
cd firmware
python -m esptool --chip esp32s3 -p COM4 -b 460800 `
    --before default_reset --after hard_reset write_flash `
    --flash_mode dio --flash_size 8MB --flash_freq 80m `
    0x0 build/bootloader/bootloader.bin `
    0x8000 build/partition_table/partition-table.bin `
    0x10000 build/spectral_motion.bin
```

> Replace `COM4` with your port.

### 2. Run the Python monitor

```powershell
cd monitor
pip install pyserial
python spectral_monitor.py COM4
```

### 3. Calibration (60 seconds)

The monitor shows a progress bar. **Do not move** during calibration.

```
================================================================================
                        ESPECTRAL MOTION DETECTOR v3.2-CSI
================================================================================

  CALIBRATING...
  [########################################................] 60%
  Networks found: 12

  Stay still during calibration
================================================================================
```

### 4. Adaptive threshold self-calibration (~15s)

After calibration, the system settles for 5 scans then measures the quiet baseline:

```
Adaptive threshold: quiet=4.2 high=6.3 low=5.5
```

### 5. Monitoring

After threshold calibration, the system switches to monitoring mode:

```
================================================================================
                        ESPECTRAL MOTION DETECTOR v3.2-CSI
================================================================================
  SYSTEM: ACTIVE (monitoring)
  APs:       9
  Score:     3.80  Normal
--------------------------------------------------------------------------------
  BSSID              RSSI    Baseline Noise  Diff     Miss  CSI
--------------------------------------------------------------------------------
  B8:BE:F4:36:1B:D2  -40    -39.6    1.0     0.38     0     ---
  8A:C2:27:C7:1D:7C  -79    -79.0    1.2     0.85     0     ---
  ...

================================================================================
  Events:
   INFO  score=0.0  System started. Stay still during calibration...
================================================================================
```

When movement is detected:

```
  !!ALARMA DISPARADA!!
  Score:     7.20  >> UMBRAL (6.3)
```

The buzzer (if connected) sounds. The alarm auto-clears after 30 seconds if the
score drops below the low threshold.

## Project structure

```
ESP32S3_Spectral_Motion/
+-- README.md                    # This file
+-- PROYECTO.md                  # Technical docs (Spanish)
+-- TRACKING.md                  # Development history
+-- firmware/                    # ESP32-S3 firmware (ESP-IDF)
|   +-- CMakeLists.txt
|   +-- sdkconfig.defaults
|   +-- build_csi.ps1            # Build script
|   +-- build/                   # Compiled binaries
|   +-- main/
|       +-- CMakeLists.txt
|       +-- Kconfig.projbuild    # Wi-Fi association config menu
|       +-- spectral_motion.c    # Main firmware source
+-- monitor/                     # Python real-time monitor
    +-- spectral_monitor.py
```

## Building from source (with ESP-IDF)

```powershell
# Install ESP-IDF from: https://docs.espressif.com/projects/esp-idf/

cd firmware
idf.py set-target esp32s3
idf.py menuconfig      # Configure Wi-Fi credentials, CSI, etc.
idf.py build
idf.py -p COM4 flash
idf.py -p COM4 monitor
```

## Serial protocol

The ESP32-S3 outputs JSON messages at 115200 baud:

| Type | Fields | Description |
|---|---|---|
| `cal` | `pct`, `aps` | Calibration progress |
| `state` | `aps`, `score`, `alarm`, `th_high`, `th_low` | Current system state with dynamic thresholds |
| `ap` | `bssid`, `rssi`, `baseline`, `noise`, `diff`, `misses`, `csi_score`, `csi_pkt` | Per-AP status with CSI data |
| `alarm` | `state`, `score`, `reason` | Alarm event |
| `status` | `msg` | Informational message |

## Detection algorithm details

- **Calibration**: Median + MAD (robust statistics), filters noisy APs (>3.5 dB MAD)
- **Noise floor minimum**: 1.0 dB (clamped to prevent over-sensitivity)
- **Slow baseline**: EMA with alpha=0.03 (~33s time constant) -- adapts to gradual drift
- **Fast baseline**: EMA with alpha=0.30 (~3s time constant) -- tracks immediate changes
- **Combined RSSI score**: 45% deviation + 25% rate-of-change + 10% fast/slow divergence
- **CSI contribution**: +30% of per-AP CSI deviation score (magnitude + profile)
- **Score smoothing**: EMA with alpha=0.50
- **Adaptive thresholds**: After calibration, 5 settle scans + 5 quiet scans measure ambient baseline, then `th_high = quiet * 1.5`, `th_low = quiet * 1.3` (minimum 2.0/1.0)
- **Alarm logic**: M-of-N (3 of 5) + 30s minimum latch + 10s cooldown
- **AP health**: Missing APs penalized, removed after 8 consecutive misses
- **Channel lock**: Automatically selects the channel with most tracked APs for optimal CSI reception
- **Wi-Fi association**: Optional (Kconfig), connects to a known AP for data-frame CSI (higher packet rate)

## License

MIT -- Free to use, modify, and distribute.
