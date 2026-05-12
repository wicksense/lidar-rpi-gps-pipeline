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

## Mounting and Orientation

How to align (step-by-step):
1. Mount the GNSS antenna so it faces open sky.
2. Rigidly mount antenna and LiDAR to the same structure (avoid flex/wobble).
3. Estimate LiDAR center and antenna phase-center location.
4. Measure offsets in meters: `dx`, `dy`, `dz`.
5. Pass offsets in offline processing:
   - `--sensor-offset-x`
   - `--sensor-offset-y`
   - `--sensor-offset-z`

What matters vs what does not:
- For this pipeline, antenna position relative to LiDAR is the key reference.
- ZED-F9P board/chip orientation is usually not critical.
- `gps_fusion` now supports orientation handling; tune yaw/orientation options when needed.
- For handheld capture with changing heading, prefer `slam_gps_anchor` (recommended).


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

Optional test dependency:

```bash
python -m pip install pytest
```

Run GUI:

```bash
python3 offline_gui.py
```

## GUI quick start

The GUI has three tabs:
- `Manifest -> Process`: use when you have `capture_manifest_*.json`
- `Raw -> Process`: use when you have a folder of `.pcap/.osf` files directly
- `Raw -> Merge Chunks`: local SLAM merge only (no GPS fusion/georeference)

### Recommended flow (Raw tab)

1. Open `Raw -> Process`.
2. Set `Processing mode` to `SLAM + GPS Anchor (Recommended)`.
3. Fill:
   - `Raw input dir`: folder containing your raw LiDAR chunks (`.pcap/.osf`)
   - `Raw session filter (optional)`: session id like `20260317_094022`
   - `GPS CSV`: matching `raw_gps_*.csv`
   - `Output dir`: where output LAS/QA files should be written
4. Keep defaults for first run:
   - `Backend`: `pose_optimizer (recommended)`
   - `Time mode`: `auto`
   - `GPS time column`: `auto`
   - `Filter GPS outliers`: enabled
   - `Max GPS speed (m/s)`: `30.0`
   - `Filter reflectivity`: disabled (enable for crisper maps on noisy data)
   - `Min reflectivity`: `2` when enabled
   - `UTM EPSG`: `32614` (change only if your project CRS differs)
   - Leave `Raw session filter` empty to auto-infer session id from GPS CSV filename
5. Click `Run`.
6. Watch progress/logs in the bottom panel. On success, output paths are printed in the log.

### Merge-only flow (Raw -> Merge Chunks tab)

Use this when you want to combine chunked raw captures into one local map without GPS fusion.

1. Open `Raw -> Merge Chunks`.
2. Fill:
   - `Raw input dir`: folder containing your raw LiDAR chunks (`.pcap/.osf`)
   - `Raw session filter (optional)`: session id like `20260317_094022`
   - `Output dir`: where merged outputs should be written
3. Click `Run`.

Notes:
- This tab is fixed to `SLAM Map (Local)` mode.
- GPS CSV is not used.
- Output is local-frame (not georeferenced to map coordinates).

### Manifest tab flow

1. Open `Manifest -> Process`.
2. Fill:
   - `Manifest JSON`: `capture_manifest_*.json`
   - `Output dir`: destination for outputs
3. Optional:
   - `Manifest base dir`: use when manifest paths are from another machine
   - `GPS CSV override`: force a specific GPS CSV for `gps_fusion` or `slam_gps_anchor`
4. Click `Run`.

### Which mode to choose

- `SLAM + GPS Anchor`: best default for moving vehicle captures
- `SLAM Map`: local map only (no GPS georeference)
- `GPS Fusion`: direct GPS georeference, more sensitive to GPS/heading quality

### Buttons and behavior

- `Run`: starts processing in-process (no CLI subprocess).
- `Stop`: requests safe cancellation; partial outputs are cleaned up.
- `Open OSF in Viz`: enabled after runs that produce OSF (`save_osf` in SLAM modes).
  Uses `Viz accum-num` value when launching Ouster Viz.

### Example values (your March 17 data)

In `Raw -> Process`:
- `Raw input dir`: `/home/spriteadmin/Documents/LiDAR-Object-Detection/SampleData/3-17`
- `Raw session filter (optional)`: `20260317_094022`
- `GPS CSV`: `/home/spriteadmin/Documents/LiDAR-Object-Detection/SampleData/3-17/raw_gps_20260317_094022.csv`
- `Output dir`: `/home/spriteadmin/Documents/ lidar-rpi-gps-pipeline/test_outputs`
- `Processing mode`: `SLAM + GPS Anchor (Recommended)`
- `Backend`: `pose_optimizer (recommended)`

GUI notes:
- Mode selector supports `SLAM + GPS Anchor`, `SLAM Map`, and `GPS Fusion`.
- Anchor mode includes backend/options for constraint-based pose optimization.
- SLAM modes can also save an `.osf` playback file.
- `Open OSF in Viz` launches `ouster-cli source <output.osf> viz --accum-num <N>` when `Viz accum-num > 0`.
- Includes progress indicators while processing files/scans.
- `Stop` asks for confirmation, cancels safely, and cleans partial LAS output.

Run unit tests:

```bash
PYTHONPATH=src python -m pytest -q
```

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

Optional runtime override for lidar mode:

```bash
python3 pi_capture_raw.py --lidar-mode 1024x20
python3 pi_capture_raw.py --lidar-mode 2048x10
python3 pi_capture_raw.py --lidar-mode 4096x5
```

### Key capture settings (inside `pi_capture_raw.py`)

- `OUSTER_HOST`: sensor IP on Pi Ethernet link
- `LIDAR_OUTPUT_MODE`: `pcap_raw` (default) or `csv`
- `LIDAR_MODE`: sensor mode applied before capture, for example `1024x20`, `2048x10`, or `4096x5`
- `CAPTURE_DURATION_SEC`: seconds per chunk (default `30`)
- `CONTINUOUS_CHUNKS`: keep chunking until `Ctrl+C`
- `WAIT_FOR_GPS_FIX_BEFORE_CAPTURE`: wait for first fix before LiDAR start
- `GPS_PORT`, `GPS_BAUD`: GPS serial settings
- `OUTPUT_DIR`: output folder on Pi
- `UTM_EPSG`: projected CRS used for GPS easting/northing

Notes:
- The Pi script uses `ouster-cli source <sensor> config lidar_mode <mode>` before capture starts.
- Set `LIDAR_MODE = None` (or pass an empty override by editing the file) to leave the current sensor mode unchanged.
- Useful defaults:
  `1024x20` for walking / strongest pose robustness,
  `2048x10` as the balanced option,
  `4096x5` for maximum spatial crispness with smoother motion.

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
  --output-las /path/to/output/session_001_slam.las \
  --save-osf
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
  --output-las /path/to/output/session_001_anchor.las \
  --save-osf
```

Useful anchoring parameters:
- `--anchor-min-motion-m` (default `5.0`)
- `--anchor-z-mode` (`offset` or `none`)
- `--anchor-backend` (`pose_optimizer` recommended, `rigid_fit` optional simpler method)
- `--gps-outlier-filter` (`on`/`off`, default `on`)
- `--gps-outlier-max-speed-mps` (default `30.0`)
- `--reflectivity-filter` (`on`/`off`, default `off`)
- `--min-reflectivity` (default `2`, used when reflectivity filter is on)
  Ouster example `filter REFLECTIVITY 1:1` means "remove points with reflectivity exactly 1".
  In this pipeline, `--min-reflectivity 2` is equivalent behavior (keep values >= 2, so 1 is dropped).
- `--save-osf` / `--output-osf` for playback output
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
- In `slam_gps_anchor` mode, default backend uses Ouster Pose Optimizer with GPS absolute constraints.
- `slam_gps_anchor` does not silently switch backends; if `pose_optimizer` fails, the run fails.
- choose `--anchor-backend rigid_fit` explicitly if you want rigid anchoring.
- Reflectivity filtering applies to raw replay export paths (`gps_fusion` and `slam_gps_anchor` with `pose_optimizer`).
  It is not applied in `slam_map` or `slam_gps_anchor` with `rigid_fit`.
- `--save-osf` writes an OSF for playback in SLAM modes.
- In `slam_gps_anchor` mode:
  `pose_optimizer` backend writes optimized (GPS-anchored) OSF.
  `rigid_fit` backend writes local SLAM OSF (LAS remains GPS-anchored output).
- If output filename already exists, a numeric suffix is added automatically (`_2`, `_3`, ...).
- Per-scan converted CSV files are not written unless you enable `--keep-converted`.
- If `--keep-converted` is enabled, many debug CSV files can be created.
- Use `--converted-csv-dir` to control where debug converted CSV files are written.

## Output Files

Default raw/manifest output:
- one merged `.las` file (LAS 1.4)
- one QA JSON report for `slam_gps_anchor`: `*_qa.json`
- optional `.osf` playback file when `--save-osf`/`--output-osf` is used

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

## GPS Fusion Orientation Options

These options apply to `--processing-mode gps_fusion`:

- `--gps-fusion-orientation path_tangent` (default): rotate local XY by GPS path direction
- `--gps-fusion-orientation fixed_yaw`: use one fixed yaw for all points
- `--sensor-yaw-deg`: additional yaw offset in degrees (used in both modes)

Guidance:
- For moving captures where heading changes, `path_tangent` is usually better than fixed assumptions.
- If your LiDAR has a known fixed heading offset, set `--sensor-yaw-deg` accordingly.
- If orientation behavior is still not satisfactory for handheld data, use `slam_gps_anchor`.

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
- `--save-osf` save OSF playback file for SLAM modes
- `--output-osf` explicit OSF path (optional)

## Anchoring Options

- `--anchor-min-motion-m` default `5.0` (below this, translation-only anchoring is used)
- `--anchor-z-mode` default `offset` (`none` keeps SLAM Z unchanged)
- `--anchor-backend` default `pose_optimizer`
- `--poseopt-key-frame-distance` default `1.0`
- `--poseopt-constraints-every-m` default `10.0`
- `--poseopt-constraint-weights` default `0.01,0.01,0.001`
- `--poseopt-max-iterations` default `50`
- `--poseopt-map-voxel-size` default `0.5` (`0` disables map downsampling)

## Parameter Tuning Guide

Use this as a practical starting point before fine-tuning:

1. Start with `slam_gps_anchor` + `pose_optimizer` backend.
2. Keep defaults for the first run.
3. Check QA (`*_qa.json`) and map quality.
4. Adjust one parameter group at a time.

Recommended starting profiles:

- Handheld urban walk:
  `--slam-min-range 1.5 --slam-max-range 120 --poseopt-constraints-every-m 8 --poseopt-map-voxel-size 0.25`
- Slow push-cart / stable rig:
  `--slam-min-range 1.0 --slam-max-range 150 --poseopt-constraints-every-m 10 --poseopt-map-voxel-size 0.5`
- Dense downtown / heavy multipath:
  `--slam-min-range 1.5 --slam-max-range 80 --poseopt-constraints-every-m 6 --poseopt-map-voxel-size 0.2`

What each knob changes:

- `--slam-voxel-size`:
  lower = denser map, more compute; higher = smoother/coarser map, faster runtime.
- `--slam-min-range`:
  raise this to remove near-body/operator points and close-in artifacts.
- `--slam-max-range`:
  lower this in clutter/multipath environments to reduce unstable far returns.
- `--slam-deskew-method`:
  start with `auto`; use `constant_velocity` if motion distortion is visible and `auto` underperforms.
- `--poseopt-constraints-every-m`:
  lower value = more GPS constraints (more correction, more sensitivity to noisy GPS);
  higher value = fewer constraints (smoother but less anchored).
- `--poseopt-map-voxel-size`:
  lower value = larger LAS files with more detail; higher value = smaller/faster outputs.
- `--anchor-min-motion-m`:
  raise this if short/noisy tracks are over-rotated by rigid fallback.
- `--sensor-offset-x/y/z`:
  use measured antenna-to-LiDAR offsets; incorrect signs produce consistent spatial bias.
- `--gps-fusion-orientation` and `--sensor-yaw-deg` (gps_fusion mode):
  use `path_tangent` for changing heading;
  use `fixed_yaw` only when sensor heading relative to map is known and stable.

Symptoms and likely fixes:

- Thick/ghosted walls:
  increase `--slam-min-range`; try `--slam-deskew-method constant_velocity`.
- Map looks too sparse:
  decrease `--poseopt-map-voxel-size`; verify `--slam-max-range` is not too low.
- QA XY errors high:
  lower `--poseopt-constraints-every-m` moderately; verify GPS quality and offsets.
- Output file too large:
  increase `--poseopt-map-voxel-size`; increase `--slam-voxel-size`.
- gps_fusion looks directionally skewed:
  switch to `--gps-fusion-orientation path_tangent` or adjust `--sensor-yaw-deg`.

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
- GPS-fusion interpolation trims points/samples outside GPS time coverage (instead of extrapolating).
- Manifest paths from Pi can be absolute and may need remapping with `--manifest-base-dir`.
- Enabling `--keep-converted` can generate many debug CSV files.
- Large captures can produce large LAS files; make sure the output drive has enough free space.

## Planned Improvements

- packaged desktop app builds (Linux/macOS/Windows)
