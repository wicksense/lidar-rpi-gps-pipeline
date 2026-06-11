# LiDAR RPi GPS Pipeline

Capture Ouster LiDAR + GPS in the field on a Raspberry Pi, then georeference it offline.

This project is designed for a practical field workflow:
- Raspberry Pi handles data capture
- workstation/server handles conversion and fusion

## Hardware Setup

This repo now supports two capture architectures:

- Original path:
  GPS position/time is logged on the Pi, and `pi_capture_raw.py` handles capture.
- PTP path:
  GPS + PPS feed the Raspberry Pi, the Pi clock is disciplined by `chrony`, the Pi serves PTP on Ethernet, and the Ouster locks to the Pi. `pi_capture_ptp.py` then waits for GPS fix, Pi clock sync, and Ouster PTP lock before capture.

Validated hardware family:
- Raspberry Pi (capture host)
- Ouster LiDAR sensor on Pi Ethernet
- u-blox ZED-F9P class GNSS receiver

### Why the PTP path exists

The PTP path is useful when you want the Pi to be the central timing authority instead of wiring GPS timing directly into the Ouster. In that setup:

- the GPS provides location plus precise time/PPS to the Pi
- the Pi disciplines its system clock
- the Pi advertises PTP on Ethernet
- the Ouster timestamps LiDAR data from the Pi-synchronized PTP clock

That gives the offline pipeline a clean shared time basis, and for these sessions the preferred offline time column is:

- `--gps-time-column pi_time_ns`

### Wiring the GPS to the Pi for the PTP path

At a high level, the GNSS receiver needs:

- I2C data path to the Pi
- PPS pulse to one Pi GPIO input
- common ground with the Pi
- power appropriate for the GNSS board you are using

Recommended Pi-side connections for the scripts in this repo:

- I2C SDA: `GPIO2`, physical pin `3`
- I2C SCL: `GPIO3`, physical pin `5`
- PPS input: `GPIO18`, physical pin `12`
- Ground: any Pi ground pin, for example physical pin `6`

Important safety notes:

- Pi GPIO is `3.3V` only, not `5V` tolerant.
- Make sure the GNSS PPS output is safe for `3.3V` GPIO input.
- Power the GNSS board according to its own documentation. Some carrier boards accept multiple supply options; do not assume every board should be powered from the Pi the same way.
- Keep a common ground between Pi and GNSS.

Practical interpretation:

- I2C carries GNSS messages into the Pi.
- PPS gives the Pi a precise second boundary.
- `chrony` combines the coarse UTC time with PPS precision.
- `ptp4l` and `phc2sys` then let the Pi distribute that time to the Ouster over Ethernet.

### Ouster connection in the PTP path

- Connect the Ouster to the Pi over Ethernet.
- Configure the Ouster timestamp mode for PTP when using `pi_capture_ptp.py`.
- The new capture script can verify Ouster timing state through the HTTP API before recording starts.

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

1. Run `pi_capture_raw.py` on the Pi for the original serial/PPS capture path, or `pi_capture_ptp.py` for the Pi-clock/PTP timing path
2. Collect:
   - raw LiDAR chunks (`.pcap` by default)
   - one GPS CSV log
   - one capture manifest JSON
3. Move files to an offline machine
4. Run `offline_fuse_lidar_gps.py` to process into mapping output
5. Get output `.las` (single merged file by default)

## Repository Layout

- `pi_capture_raw.py`: Raspberry Pi capture script for the original capture path
- `pi_capture_ptp.py`: Raspberry Pi capture script for GPS/PPS -> Pi clock -> PTP -> Ouster
- `pi_capture_web.py`: phone-friendly local web UI for starting/stopping Pi capture and watching logs
- `setup_pi_gps_ptp_stack.sh`: helper to configure `chrony`, `ptp4l`, `phc2sys`, PPS overlay, and the bridge service on the Pi
- `pi_gps_ptp_diagnostic.sh`: helper to verify I2C, PPS, chrony, PTP, and optional Ouster HTTP timing state
- `ublox_i2c_chrony_bridge.py`: helper service that feeds UTC time from I2C GNSS into `chrony`'s SOCK refclock and publishes the latest decoded GPS fix for `pi_capture_ptp.py`
- `systemd/pi-capture-web.service`: example `systemd` unit for starting the web UI automatically on boot
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
- `ouster-cli` installed either in `PATH` or in the same virtualenv as the capture script (the Pi scripts auto-resolve the sibling CLI next to the active Python interpreter)

For `pi_capture_ptp.py`, the Pi must also have:
- `chrony` available in `PATH` (`chronyc` is used for readiness checks)
- working PTP services outside Python (`ptp4l` / `phc2sys`)
- Ouster HTTP API reachable from the Pi
- the `ublox_i2c_chrony_bridge.py` service running when using the default `bridge` GPS input mode

For the I2C/PPS GNSS stack used by the setup helper:
- `python3-smbus` or `python3-smbus2`
- `pps-tools`
- `i2c-tools`
- `linuxptp`

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

### Original capture path

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

### PTP-synced capture path

Use this when the timing path is:

- GPS + PPS -> Raspberry Pi
- Pi clock disciplined outside Python (for example `chrony`)
- Pi -> Ouster over Ethernet PTP

Basic run:

```bash
python3 pi_capture_ptp.py
```

Useful overrides:

```bash
python3 pi_capture_ptp.py --lidar-mode 1024x20
python3 pi_capture_ptp.py --gps-input-mode bridge
python3 pi_capture_ptp.py --no-wait-for-ouster-ptp-lock
```

What `pi_capture_ptp.py` adds on top of the original script:
- optional Ouster timestamp/PTP configuration before capture
- GPS fix readiness gating
- Pi clock readiness gating via `chronyc waitsync`
- Ouster PTP lock readiness gating via HTTP API
- manifest sync metadata for field-session debugging

For the I2C + PPS + PTP setup in this repo, the recommended GPS input mode is:

- `bridge`

Why:

- `ublox_i2c_chrony_bridge.py` is already the single reader of the u-blox I2C byte stream
- it now publishes the latest valid GPS fix as local JSON
- `pi_capture_ptp.py` reads that published fix and writes the normal GPS CSV
- this avoids two processes competing to read the same I2C GPS stream

Other GPS input modes are still available when they match your hardware:

- `serial`: use only when the GNSS is exposed as a serial device like `/dev/gps_ublox`
- `gpsd`: use only if you have separately installed and configured `gpsd`

Recommended offline guidance for this architecture:
- prefer `--gps-time-column pi_time_ns`
- keep `gps_epoch_ns` for QA and UTC traceability

### Web UI for road use

If you normally connect a phone to the Pi hotspot and use VNC just to open a terminal, the web UI is usually easier.

Start it on the Pi:

```bash
python3 pi_capture_web.py --host 0.0.0.0 --port 8080
```

Then connect your phone to the Pi hotspot or local Pi network and open:

```text
http://<pi-ip>:8080
```

What the web UI gives you:

- start/stop controls for both capture paths
- separate lidar resolution and sample-rate fields in the web UI
- validation against supported Ouster mode pairs (`1024x20`, `2048x10`, `4096x5`, or leave unchanged)
- min/max range fields for downstream processing defaults
- readiness toggles for GPS fix, Pi clock sync, and Ouster PTP lock
- `bridge` GPS mode for the I2C/PPS/PTP setup
- live log streaming in the browser
- quick preflight visibility for `chrony` and optional Ouster timing state

Why this is easier than VNC on a phone:

- large tap targets instead of terminal typing
- live capture logs without a full remote desktop
- much lighter bandwidth and less fiddly screen interaction

Important note:

- `pi_capture_web.py` is a local control panel with no built-in authentication.
- Use it only on trusted local networks such as the Pi hotspot you control.

#### Start the web UI automatically on boot

If your Pi capture environment lives in a virtualenv, the easiest boot setup is a `systemd` service that points directly at the venv Python.

This repo includes an example unit:

- `systemd/pi-capture-web.service`

Before installing it:

1. Edit the file and set:
   - `User`
   - `WorkingDirectory`
   - `ExecStart`
2. Make sure `ExecStart` points to your venv Python, for example:
   - `/home/<pi-user>/lidar-rpi-gps-pipeline/.venv/bin/python`

Install it on the Pi:

```bash
sudo cp systemd/pi-capture-web.service /etc/systemd/system/pi-capture-web.service
sudo systemctl daemon-reload
sudo systemctl enable pi-capture-web.service
sudo systemctl start pi-capture-web.service
```

Check status:

```bash
sudo systemctl status pi-capture-web.service
sudo journalctl -u pi-capture-web.service -f
```

After that, the web UI should come up automatically on boot and be available from your phone at:

```text
http://<pi-ip>:8080
```

### Pi bring-up for the PTP path

The PTP path has two parts:

1. System setup on the Pi
2. Capture using `pi_capture_ptp.py`

The helper scripts in this repo are for the system-setup side.

#### 1) Configure the Pi timing stack

Run on the Pi:

```bash
sudo ./setup_pi_gps_ptp_stack.sh
```

What this script does:

- enables Pi I2C in boot config
- enables a PPS GPIO overlay
- installs/configures a `chrony` SOCK + PPS refclock setup
- installs the `ublox_i2c_chrony_bridge.py` service
- configures that bridge service to publish the latest GPS fix at `/run/ublox_i2c_chrony_bridge/latest_fix.json`
- writes `ptp4l` and `phc2sys` service units
- enables and starts those services unless `--no-start` is used

Common overrides:

```bash
sudo ./setup_pi_gps_ptp_stack.sh --pps-gpio 18
sudo ./setup_pi_gps_ptp_stack.sh --iface eth0
sudo ./setup_pi_gps_ptp_stack.sh --skip-apt
```

If the script changes boot overlays, reboot the Pi before expecting `/dev/pps0` to appear.

#### 2) Verify the timing stack

Run on the Pi:

```bash
sudo ./pi_gps_ptp_diagnostic.sh
sudo ./pi_gps_ptp_diagnostic.sh --ouster-host <ouster-ip>
```

What the diagnostic checks:

- I2C visibility of the GNSS receiver
- PPS device presence and pulse activity
- `chrony` tracking and selected source state
- `ptp4l` / `phc2sys` process state
- optional Ouster `/api/v1/time` and `/api/v1/time/ptp` responses

This script is meant for bring-up and troubleshooting before you trust a field session.

#### Optional: Reduce packet-drop risk on the Pi

When capturing Ouster UDP traffic on the Pi, you may see warnings like:

- `Failed to set desired SO_RCVBUF size to 1048576`

That means Linux allowed a smaller receive buffer than `ouster-cli` requested. Capture can still work, but packet drops are more likely during short CPU or scheduler hiccups.

A practical mitigation is to raise the kernel UDP receive buffer limits:

```bash
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.rmem_default=8388608
```

To make that persistent across reboots:

```bash
sudo tee /etc/sysctl.d/90-ouster-capture.conf >/dev/null <<'EOF'
net.core.rmem_max=8388608
net.core.rmem_default=8388608
EOF
sudo sysctl --system
```

Tradeoffs:

- usually low risk on a dedicated capture Pi
- uses a bit more RAM per socket
- helps absorb short packet bursts
- does not fix deeper issues like bad links, wrong `udp_dest`, CPU overload, or slow storage

For most field-capture setups in this repo, `8 MB` is a reasonable starting point.

#### 3) Start capture

Once the system stack is healthy:

```bash
python3 pi_capture_ptp.py --lidar-mode 1024x20
```

The script will:

- start GPS logging
- wait for a valid fix if enabled
- wait for Pi clock sync from `chrony` if enabled
- wait for Ouster PTP lock if enabled
- then begin chunked raw LiDAR capture

If you prefer the phone-first workflow, start the web UI instead and launch capture from the browser:

```bash
python3 pi_capture_web.py --host 0.0.0.0 --port 8080
```

#### 4) Move files offline and process

For PTP-based sessions, start with:

```bash
python3 offline_fuse_lidar_gps.py \
  --manifest-json /path/to/capture_manifest.json \
  --processing-mode slam_gps_anchor \
  --gps-time-column pi_time_ns \
  --output-dir /path/to/output
```

Why `pi_time_ns` is preferred here:

- the GPS rows are timestamped by the Pi clock
- the Ouster is supposed to lock to that same Pi-driven clock via PTP
- so `pi_time_ns` is the cleanest shared time basis for offline alignment

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

If your manifest contains Pi absolute paths (for example `/home/<pi-user>/...`) and you are running offline on another machine, remap paths with:

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

Guidance:
- for the original GPS logging path, `auto` is usually the right choice
- for `pi_capture_ptp.py` sessions, prefer `--gps-time-column pi_time_ns`

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
