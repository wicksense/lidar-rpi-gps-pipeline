# LiDAR RPi GPS Pipeline

Capture Ouster LiDAR + GPS in the field on a Raspberry Pi, then georeference it offline.

This project is designed for a practical field workflow:
- Raspberry Pi handles data capture
- workstation/server handles conversion and fusion

## Hardware Setup

Validated setup:
- Raspberry Pi (capture host)
- Ouster LiDAR sensor (Ethernet to Pi)
- SparkFun u-blox ZED-F9P RTK GPS (USB to Pi)
- GPS also connected to Ouster for timing/reference
  - Interface: PPS + NMEA over UART

GPS-to-Ouster wiring reference:
- https://data.ouster.io/downloads/hardware-user-manual/hardware-user-manual-revd-os0.pdf

## Workflow Overview

1. Run `pi_capture_raw.py` on the Pi
2. Collect:
   - raw LiDAR chunks (`.pcap` by default)
   - one GPS CSV log
   - one capture manifest JSON
3. Move files to an offline machine
4. Run `offline_fuse_lidar_gps.py` to convert + georeference
5. Get output `.csv` and `.las`

## Repository Layout

- `pi_capture_raw.py`: Raspberry Pi capture script
- `offline_fuse_lidar_gps.py`: offline conversion and GPS fusion
- `src/lidar_rpi_gps_pipeline/`: package scaffold for modularization

## Requirements

Use a separate virtual environment on each machine (Pi and offline workstation/server).

## Raspberry Pi capture environment

Create and activate a venv on the Pi:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pandas numpy pyproj pynmea2 pyserial
```

Runtime dependency:
- `ouster-cli` available in `PATH` (used by `pi_capture_raw.py` during capture)

## Offline fusion environment

Create and activate a venv on the offline machine:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy pandas pyproj laspy ouster-sdk
```

Note:
- Offline conversion is done through `ouster-sdk` Python APIs.
- `ouster-cli` is not required for the offline fusion script.

## Capture on Raspberry Pi

Basic run:

```bash
python3 pi_capture_raw.py
```

Optional runtime override for GPS wait behavior:

```bash
python3 pi_capture_raw.py --wait-for-gps-fix
python3 pi_capture_raw.py --no-wait-for-gps-fix
```

### Key capture settings (inside `pi_capture_raw.py`)

- `OUSTER_HOST`: sensor IP on Pi Ethernet link
- `LIDAR_OUTPUT_MODE`: `pcap_raw` (default) or `csv`
- `CAPTURE_DURATION_SEC`: seconds per chunk (default `30`)
- `CONTINUOUS_CHUNKS`: keep chunking until `Ctrl+C`
- `WAIT_FOR_GPS_FIX_BEFORE_CAPTURE`: wait for first fix before LiDAR start
- `GPS_PORT`, `GPS_BAUD`: GPS serial settings
- `OUTPUT_DIR`: output folder on Pi
- `UTM_EPSG`: projected CRS used for GPS easting/northing

### Choosing chunk duration

- `30s` is a practical default
- longer chunks: fewer files, bigger loss if interrupted
- shorter chunks: more files, better recovery granularity

## Capture Outputs

Typical outputs per session:
- `raw_lidar_<session>_chunk0000.pcap`
- `raw_lidar_<session>_chunk0001.pcap`
- `raw_gps_<session>.csv`
- `capture_manifest_<session>.json`

File meanings:
- `raw_lidar_...`: raw LiDAR packets (not georeferenced yet)
- `raw_gps_...`: continuous GPS log with fix + projected coordinates
- `capture_manifest_...`: index of session files/settings

## Offline Fusion

The offline script supports three input styles:

## 1) Raw LiDAR + GPS CSV

```bash
python3 offline_fuse_lidar_gps.py \
  --lidar-raw /path/to/raw_chunk.pcap \
  --gps-csv /path/to/raw_gps.csv \
  --output-prefix /path/to/output/session_001
```

## 2) Manifest JSON (recommended for full sessions)

```bash
python3 offline_fuse_lidar_gps.py \
  --manifest-json /path/to/capture_manifest.json \
  --output-dir /path/to/output
```

## 3) Advanced: pre-converted LiDAR CSV (optional)

Use this only if you already converted raw data to LiDAR CSV elsewhere and want to fuse that CSV directly.
Most users should use `--lidar-raw` or `--manifest-json`.

```bash
python3 offline_fuse_lidar_gps.py \
  --lidar-csv /path/to/lidar_frame.csv \
  --gps-csv /path/to/raw_gps.csv \
  --output-prefix /path/to/output/session_001
```

### Important conversion behavior

- One raw `.pcap` can expand into many intermediate CSV frame files.
- If multiple LiDAR frames are processed, output names are auto-suffixed.
- Intermediate converted CSV files are deleted after successful run by default.
- Use `--keep-converted` to keep converted CSV intermediates.
- Use `--converted-csv-dir` to control where converted CSV intermediates are written.

## Output Files

For each processed LiDAR frame:
- `... .csv`: georeferenced point table
- `... .las`: georeferenced LAS 1.4 point cloud

CSV columns include:
- `timestamp_ns`, `x_global`, `y_global`, `z_global`

## Time Alignment Options

`offline_fuse_lidar_gps.py` supports:
- `--time-mode auto` (default)
- `--time-mode unix_ns`
- `--time-mode relative_start`

Guidance:
- use `auto` in most cases
- use `unix_ns` when both LiDAR and GPS are true epoch-ns timestamps
- use `relative_start` when clocks are not directly aligned

## GPS Time Column Options

- `--gps-time-column auto` (default; prefers `gps_epoch_ns`, falls back to `pi_time_ns`)
- `--gps-time-column gps_epoch_ns`
- `--gps-time-column pi_time_ns`

## CRS / EPSG Notes

- `UTM_EPSG` in `pi_capture_raw.py` affects projected GPS coordinates written to GPS CSV.
- `--utm-epsg` in `offline_fuse_lidar_gps.py` sets CRS metadata for outputs.
- Keep these consistent unless intentionally reprojecting.

## Sensor Offsets

If your LiDAR origin is offset from GPS antenna phase center, apply:
- `--sensor-offset-x`
- `--sensor-offset-y`
- `--sensor-offset-z`

Values are in meters and applied during fusion.

## Common Pitfalls

- GPS CSV must contain at least 2 usable rows for interpolation.
- Manifest paths from Pi can be absolute and may need adjustment on another machine.
- A single raw file can generate many output CSV/LAS files; this is expected.

## Planned Improvements

- user-friendly GUI for offline conversion/fusion
