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
4. Run `offline_fuse_lidar_gps.py` to process into mapping output
5. Get output `.las` (single merged file by default)

## Repository Layout

- `pi_capture_raw.py`: Raspberry Pi capture script
- `offline_fuse_lidar_gps.py`: offline processing (GPS fusion + SLAM map)
- `offline_gui.py`: desktop GUI launcher (MVP scaffold)
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

Optional GUI dependency:

```bash
python -m pip install pyside6
```

Run GUI:

```bash
python3 offline_gui.py
```

GUI notes:
- Mode selector supports `SLAM + GPS Anchor`, `SLAM Map`, and `GPS Fusion`.
- Includes progress indicators while processing files/scans.
- `Stop` asks for confirmation, cancels safely, and cleans partial LAS output.

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

## Offline Processing

The offline script supports three processing modes:

- `gps_fusion`: georeference using GPS timestamps
- `slam_map`: SDK SLAM map generation (local coordinates)
- `slam_gps_anchor`: SDK SLAM map generation + GPS-based global anchoring (recommended)

The offline script supports three input styles:

## 1) Raw LiDAR + GPS CSV (GPS fusion)

`--lidar-raw` supports:
- one or more `.pcap/.osf` files
- a directory path containing raw files
- a glob pattern

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode gps_fusion \
  --lidar-raw /path/to/raw_files_dir \
  --gps-csv /path/to/raw_gps.csv \
  --output-las /path/to/output/session_001.las
```

## 2) Raw LiDAR -> SLAM map (local coordinates)

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode slam_map \
  --lidar-raw /path/to/raw_files_dir \
  --output-las /path/to/output/session_001_slam.las
```

Useful SLAM parameters:
- `--slam-voxel-size` (0 = auto)
- `--slam-min-range` (set to 1.5-2.0 m to remove near-operator points)
- `--slam-max-range`
- `--slam-deskew-method` (`auto`, `none`, `constant_velocity`, `imu_deskew`)

## 3) Raw LiDAR + GPS CSV -> SLAM + GPS anchor (recommended)

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode slam_gps_anchor \
  --lidar-raw /path/to/raw_files_dir \
  --gps-csv /path/to/raw_gps.csv \
  --output-las /path/to/output/session_001_anchor.las
```

Useful anchoring parameters:
- `--anchor-min-motion-m` (default `5.0`)
- `--anchor-z-mode` (`offset` or `none`)
- `--time-mode` and `--gps-time-column` also apply in this mode
## 4) Manifest JSON (recommended for full sessions)

```bash
python3 offline_fuse_lidar_gps.py \
  --manifest-json /path/to/capture_manifest.json \
  --processing-mode slam_map \
  --output-dir /path/to/output
```

If your manifest contains Pi absolute paths (for example `/home/urp-pi5/...`) and you are running offline on another machine, remap paths with:

```bash
python3 offline_fuse_lidar_gps.py \
  --manifest-json /path/to/capture_manifest.json \
  --manifest-base-dir /path/to/copied/session/files \
  --processing-mode gps_fusion \
  --output-dir /path/to/output
```

You can also override GPS explicitly:

```bash
python3 offline_fuse_lidar_gps.py \
  --manifest-json /path/to/capture_manifest.json \
  --manifest-base-dir /path/to/copied/session/files \
  --processing-mode gps_fusion \
  --gps-csv /path/to/raw_gps.csv \
  --output-dir /path/to/output
```

## 5) Advanced: pre-converted LiDAR CSV (optional, GPS fusion only)

Use this only if you already converted raw data to LiDAR CSV elsewhere and want to fuse that CSV directly.
Most users should use `--lidar-raw` or `--manifest-json`.

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode gps_fusion \
  --lidar-csv /path/to/lidar_frame.csv \
  --gps-csv /path/to/raw_gps.csv \
  --output-prefix /path/to/output/session_001
```

### Important conversion behavior

- Raw/manifest mode writes one merged LAS file by default.
- In `gps_fusion` mode, raw scans are processed in a streaming pipeline (memory stays bounded for long runs).
- In `slam_gps_anchor` mode, SLAM trajectory is fit to GPS in XY (rigid) plus optional Z offset.
- If output filename already exists, a numeric suffix is added automatically (`_2`, `_3`, ...).
- Per-scan converted CSV files are not written unless you enable `--keep-converted`.
- If `--keep-converted` is enabled, many debug CSV files can be created.
- Use `--converted-csv-dir` to control where debug converted CSV files are written.

## Output Files

Default raw/manifest output:
- one merged `.las` file (LAS 1.4)

Advanced CSV-input mode output:
- per input LiDAR CSV: `... .csv` and `... .las`

CSV columns include:
- `timestamp_ns`, `x_global`, `y_global`, `z_global`

## Time Alignment Options

`offline_fuse_lidar_gps.py` supports:
- `--time-mode auto` (default)
- `--time-mode unix_ns`
- `--time-mode relative_start`

These options apply to `--processing-mode gps_fusion` and `--processing-mode slam_gps_anchor`.

Guidance:
- use `auto` in most cases
- use `unix_ns` when both LiDAR and GPS are true epoch-ns timestamps
- use `relative_start` when clocks are not directly aligned

## GPS Time Column Options

- `--gps-time-column auto` (default; prefers `gps_epoch_ns`, falls back to `pi_time_ns`)
- `--gps-time-column gps_epoch_ns`
- `--gps-time-column pi_time_ns`

These options apply to `--processing-mode gps_fusion` and `--processing-mode slam_gps_anchor`.

## SLAM Options

- `--slam-voxel-size` default `0.0` (auto-estimate)
- `--slam-min-range` default `1.0`
- `--slam-max-range` default `150.0`
- `--slam-deskew-method` default `auto`

## Anchoring Options

- `--anchor-min-motion-m` default `5.0` (below this, translation-only anchoring is used)
- `--anchor-z-mode` default `offset` (`none` keeps SLAM Z unchanged)

## CRS / EPSG Notes

- `UTM_EPSG` in `pi_capture_raw.py` affects projected GPS coordinates written to GPS CSV.
- `--utm-epsg` in `offline_fuse_lidar_gps.py` sets CRS metadata for georeferenced outputs (`gps_fusion`, `slam_gps_anchor`).
- `slam_map` output is local-map coordinates.
- `slam_gps_anchor` output is georeferenced in GPS projected coordinates (UTM EPSG).
- Keep these consistent unless intentionally reprojecting.

## Sensor Offsets

If your LiDAR origin is offset from GPS antenna phase center, apply:
- `--sensor-offset-x`
- `--sensor-offset-y`
- `--sensor-offset-z`

Values are in meters and applied during fusion.

## Common Pitfalls

- GPS CSV must contain at least 2 usable rows for interpolation.
- Manifest paths from Pi can be absolute and may need remapping with `--manifest-base-dir`.
- Enabling `--keep-converted` can generate many debug CSV files.
- Large captures can produce large LAS files; make sure the output drive has enough free space.

## Planned Improvements

- packaged desktop app builds (Linux/macOS/Windows)
