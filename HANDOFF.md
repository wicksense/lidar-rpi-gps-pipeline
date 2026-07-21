# CivicScan Pipeline — Agent Handoff Document

Generated: 2026-03-30. This is the complete context transfer for continuing development
on a new machine. Read it fully before touching any code.

---

## 1. What This Project Is

**CivicScan Pipeline (Project A)** — An end-to-end LiDAR field capture and offline georeferencing
pipeline for urban planning. Captures raw 3D point clouds with a mobile rig, then georeferences
them offline into LAS files with real-world UTM coordinates.

**Project B** (analytics: people counting, classification) is entirely separate and not in this
repo. Do not conflate them.

**Hardware:**
- Raspberry Pi 5 (capture host)
- Ouster OS-1-128 LiDAR (128 beams, 1024x10 mode = 10 Hz, 1024 columns/frame)
- SparkFun u-blox ZED-F9P GPS receiver (RTK-capable, NMEA via serial at 9600 baud)
- Ouster configured with `TIME_FROM_SYNC_PULSE_IN` + `INPUT_NMEA_UART`:
  LiDAR scan timestamps are GPS UTC epoch nanoseconds (verified from sensor sidecar JSON)

---

## 2. Repository Layout

```
/home/r/Documents/LiDAR/UrbanPlanningLidar/       ← parent working dir (NOT the git repo)
├── .venv/                                          ← shared Python venv
├── docs/                                           ← internal planning docs (NOT in public repo)
│   ├── spec.md
│   ├── architecture.md
│   ├── roadmap.md
│   ├── memory.md                                   ← living decision log
│   └── gui_implementation_plan.md
├── SampleData/                                     ← 2-chunk session (2026-02-25)
│   └── mock_gps_for_test.csv
├── SampleData2/26/                                 ← 9-chunk session (2026-02-26)
└── lidar-rpi-gps-pipeline/                        ← PUBLIC git repo
    ├── pi_capture_raw.py                           ← Pi field capture script
    ├── offline_fuse_lidar_gps.py                  ← CLI wrapper (thin, no logic)
    ├── offline_gui.py                              ← PySide6 GUI launcher
    └── src/lidar_rpi_gps_pipeline/
        ├── jobs/
        │   ├── __init__.py                         ← exports: run_offline_job, build_arg_parser,
        │   │                                          run_from_args, main, JobCancelledError
        │   └── offline_fusion.py                  ← ALL backend logic (~2400+ lines)
        └── gui/
            ├── __init__.py
            └── app.py                              ← PySide6 GUI implementation
```

`docs/` and `SampleData*/` are NOT in the public git repo. The git repo root is
`lidar-rpi-gps-pipeline/`.

---

## 3. Python Environment

**Venv:** `/home/r/Documents/LiDAR/UrbanPlanningLidar/.venv`

**Key packages:** `ouster-sdk`, `PySide6`, `laspy`, `pyproj`, `pynmea2`, `numpy`, `pandas`

**Activate:** `source /home/r/Documents/LiDAR/UrbanPlanningLidar/.venv/bin/activate`

**CLI (from repo root):** `python offline_fuse_lidar_gps.py --help`

**GUI (from repo root):** `python offline_gui.py`

If PySide6 is missing, the GUI launcher fails with a clear install message; CLI and library still
work. The `src/` directory is added to `sys.path` by the launcher scripts; no install step needed.

---

## 4. Architecture: Two-Stage Pipeline

### Stage 1 — Field Capture (Pi)

`pi_capture_raw.py` runs on the Pi. Steps:
1. Opens GPS serial (`/dev/gps_ublox`, 9600 baud), starts `GpsLogger` background thread
2. Reads NMEA GGA sentences via `pynmea2`; logs one row per valid fix (gps_qual != 0)
3. GPS CSV columns: `pi_time_ns`, `gps_epoch_ns`, `hdop`, `latitude`, `longitude`,
   `altitude_m`, `easting`, `northing`
4. `altitude_m` = `msg.altitude` from GGA field 9 = **MSL altitude (above geoid)**, NOT HAE
   (u-blox applies embedded geoid model internally; result is approximately orthometric)
5. `easting`/`northing` = UTM meters via pyproj; default EPSG hardcoded as `UTM_EPSG = 32614`
6. Runs Ouster capture via `ouster-cli save_raw` subprocess → `.pcap` chunks + sidecar `.json`
7. Each chunk = 30 seconds of raw UDP packets
8. Writes `capture_manifest_<session_id>.json`
9. On exit, optionally zips everything into `session_<session_id>.zip`

### Stage 2 — Offline Processing (desktop/server)

`offline_fuse_lidar_gps.py` is a 17-line thin wrapper. All logic is in `offline_fusion.py`.
The GUI calls `run_offline_job(config, emit_event, should_cancel)` in-process from a `QThread`.

---

## 5. Processing Modes

### `gps_fusion` — Direct GPS interpolation, no SLAM

1. `load_gps_for_interpolation()` — load GPS CSV, drop NaN/non-finite easting/northing/altitude
2. Open each PCAP via `ouster.sdk.open_source`, iterate scan objects
3. `_extract_scan_arrays()` — range image → `XYZLut(info, use_extrinsics=True)` → XYZ;
   apply `destagger`; apply X/Y sign flip (`-xyz[:,:,0]`, `-xyz[:,:,1]`) to handle 180° yaw
4. `choose_time_mode()` — align clocks; always `unix_ns` for this hardware
5. `_trim_query_to_gps_range()` — trim columns outside GPS time range (log + skip, don't hard-fail)
6. `np.interp` easting/northing/altitude at each column's timestamp
7. `_rotate_local_xy_for_gps_fusion()` — rotate sensor-local XY into UTM frame:
   - `path_tangent` (default): heading from GPS trajectory tangent via `np.gradient(E/N)`,
     plus `sensor_yaw_deg` offset
   - `fixed_yaw`: constant angle = `sensor_yaw_deg`
8. `x_global = e_interp + x_local_rot`, `y_global = n_interp + y_local_rot`,
   `z_global = z_interp + z_local + sensor_offset_z`
9. Stream-write to single LAS 1.4 (no per-scan fan-out)
10. LAS header offsets derived from full GPS trajectory min − 500 m (not first scan only)

**GPS fusion requires the sensor to have a known, consistent mounting orientation relative
to direction of travel.** `path_tangent` is robust for most sessions; `fixed_yaw` is useful
when traveling in a fixed direction or when GPS trajectory is noisy.

### `slam_map` — Ouster SDK SLAM, no GPS

1. `SlamConfig` → `SlamEngine` (KISS-ICP backend via SDK)
2. Feed raw scans; SLAM produces per-column poses in local frame
3. `get_point_cloud()` → extract full map, voxel-filter
4. Write single merged LAS in SLAM local frame (no geographic coordinates)
5. Optional OSF export via `--save-osf`

### `slam_gps_anchor` — SLAM first, then GPS anchoring (recommended)

**Sub-backend `pose_optimizer` (default):**
1. Run SLAM on all raw files → build temporary OSF
2. Sample SLAM trajectory → `_trim_query_to_gps_range()` → interpolate GPS
3. Create `AbsolutePoseConstraint` objects spaced by `--poseopt-constraints-every-m`
4. Run `PoseOptimizer` (Ceres solver)
5. Replay raw scans, dewarp against optimized per-column poses → global XYZ
6. Write merged LAS + optional optimized OSF + `_qa.json`

**Sub-backend `rigid_fit`:**
1. Run SLAM → extract trajectory timestamps + XYZ
2. Trim to GPS range, interpolate GPS at SLAM timestamps
3. SVD rigid transform: `_fit_xy_rigid_transform(slam_xy, gps_xy)`;
   falls back to translation-only if motion span < `--anchor-min-motion-m` (default 5 m)
4. Apply rotation + translation to all SLAM points
5. Z: `anchor_z_mode=offset` → median GPS−SLAM Z offset; `none` → SLAM Z unchanged
6. Write merged LAS + `_qa.json` (written only AFTER `os.replace()` commits the LAS)

**No automatic fallback** between backends. If `pose_optimizer` fails, the job fails hard.
Select `rigid_fit` explicitly if needed.

---

## 6. Key Functions in `offline_fusion.py`

| Function | ~Line | Purpose |
|---|---|---|
| `choose_time_mode()` | 59 | Clock alignment; auto→unix_ns for this hardware |
| `load_gps_for_interpolation()` | 110 | Load GPS CSV; drops NaN/non-finite on all 3 coords |
| `_gps_in_range_mask()` | 145 | Boolean mask: queries within GPS time range |
| `_trim_query_to_gps_range()` | 153 | Trim to GPS range; fail if too few remain |
| `_compute_path_tangent_heading()` | 182 | GPS heading via np.gradient on E/N |
| `_rotate_local_xy_for_gps_fusion()` | 213 | Rotate sensor XY to UTM frame |
| `_estimate_las_header_offsets_from_gps()` | 249 | LAS offsets from full GPS traj min − 500 m |
| `_open_ouster_source()` | 642 | Open PCAP/OSF → (source, info, infos, XYZLut, ChanField, destagger) |
| `_extract_scan_arrays()` | 660 | Scan → (timestamp_ns, x_local, y_local, z_local) |
| `_fit_xy_rigid_transform()` | 742 | SVD 2D rigid transform fit |
| `_write_anchor_qa_report()` | 828 | Write `_qa.json`; pass = median≤5m AND P95≤10m |
| `fuse_raw_to_single_las()` | 1043 | GPS fusion main loop |
| `slam_raw_to_gps_anchored_las()` | 1424 | SLAM + rigid_fit anchoring |
| `slam_raw_to_gps_anchored_las_pose_optimizer()` | 1710 | SLAM + pose optimizer anchoring |
| `build_arg_parser()` | 2218 | argparse for all CLI flags |
| `run_from_args()` | 2433 | Dispatch from parsed args to correct processing fn |
| `run_offline_job()` | ~2550 | GUI-facing API: config dict + emit_event + should_cancel |

**In-process API:**
```python
from lidar_rpi_gps_pipeline.jobs import run_offline_job
summary = run_offline_job(config_dict, emit_event=fn, should_cancel=fn)
```
`run_offline_job` calls `parse_args([])` for defaults, then overrides with `config_dict` keys
(use underscore names, not hyphens: `processing_mode`, `manifest_json`, etc.).

---

## 7. GUI Architecture (`app.py`)

- `JobThread(QThread)` — wraps `run_offline_job()`, emits `event_signal`, `done_signal`,
  `error_signal`, `cancelled_signal`
- `_BaseModeWidget` — shared base; contains all form controls + Run/Stop logic
- `ManifestModeWidget` — manifest JSON path input; `build_config()` → `run_offline_job`
- `RawModeWidget` — raw folder + GPS CSV path input; same pattern
- Two tabs in main window: Manifest mode, Raw mode

**Controls:**
- Processing mode: SLAM+GPS Anchor (recommended) / SLAM Map / GPS Fusion
- Anchor: backend (`pose_optimizer`/`rigid_fit`), anchor-z-mode, constraints-every-m, map-voxel
- GPS Fusion (enabled only for gps_fusion): orientation (`path_tangent`/`fixed_yaw`), sensor-yaw-deg
- UTM EPSG spinbox (1000–999999, default 32614) — always enabled
- Save OSF checkbox (SLAM modes only)
- Run / Stop with cancellation + partial output cleanup
- "Open OSF in Viz" — enabled post-run when output_osf produced;
  launches `ouster-cli source <osf> viz` via `subprocess.Popen`

---

## 8. Output Behavior

- Single merged LAS by default (streaming scan-by-scan, memory-bounded)
- Auto-increment: `name.las` → `name_2.las` → `name_3.las` (never overwrites)
- Atomic commit: writes to `<output>.partial`, then `os.replace()` to final path
- `_qa.json` per `slam_gps_anchor` run: written after LAS commit; fields: `pass` bool,
  `xy_rmse_m`, `xy_median_m`, `xy_p95_m`, `xy_max_m`, `z_mean_m`, `z_rmse_m`, thresholds,
  backend, time_mode_used
- Optional OSF: `--save-osf` or `--output-osf` for SLAM modes
- Debug CSV fan-out: only with `--keep-converted`
- On cancel: `.partial` file removed; temp dirs cleaned up

---

## 9. Clock Alignment

For this hardware (`TIME_FROM_SYNC_PULSE_IN` + `INPUT_NMEA_UART`), `auto` always selects
`unix_ns`. Both LiDAR column timestamps and GPS `gps_epoch_ns` are in GPS UTC epoch nanoseconds.

`relative_start` mode: both LiDAR and GPS timestamps are offset by `lidar_start = lidar_ts_ns[0]`,
preserving the initial GPS-to-LiDAR time offset. (Effective dead code for this hardware but
kept for other configurations.)

---

## 10. Sensor Frame / Sign Convention

`lidar_to_sensor_transform` for this sensor = 180° yaw + 38.195 mm Z offset.

In `_extract_scan_arrays()`:
```python
x_local = -xyz[:,:,0]   # sign flip handles 180° yaw
y_local = -xyz[:,:,1]
z_local =  xyz[:,:,2]
```

`XYZLut(info, use_extrinsics=True)` is used everywhere for consistency
(extrinsic is identity for this sensor, so no practical difference).

After sign flips, `x_local` ≈ forward relative to sensor mount, `y_local` ≈ left.
`_rotate_local_xy_for_gps_fusion()` then rotates these into UTM E/N using path heading.

The 38.195 mm Z offset is not applied (negligible; use `--sensor-offset-z 0.038195` if needed).

---

## 11. GPS Altitude

`altitude_m` in the GPS CSV = NMEA GGA field 9 = **MSL altitude (above geoid)**.
The u-blox ZED-F9P applies an embedded geoid model to produce MSL; it is NOT raw HAE.

Residual error vs NAVD88: ~1–3 m from embedded geoid model. Acceptable for urban planning.
GPS vertical precision: ~3 cm with RTK fix, ~0.5–1 m float mode.

One pending metadata gap: LAS CRS uses 2D UTM EPSG only (no vertical datum tag).
Numbers are correct; the CRS label is just incomplete. Low priority fix.

---

## 12. All CLI Flags

```
--processing-mode         {gps_fusion,slam_map,slam_gps_anchor}      default: gps_fusion
--manifest-json           path to capture_manifest_*.json
--manifest-base-dir       remap Pi absolute paths on offline machine
--lidar-csv               one or more LiDAR CSV files (advanced)
--lidar-raw               .pcap/.osf files, directories, or glob patterns
--gps-csv                 path to GPS CSV
--converted-csv-dir       dir for debug per-scan CSVs
--keep-converted          enable per-scan CSV debug output
--output-las              final output LAS path
--output-osf              optional output OSF path
--save-osf                also save OSF alongside LAS (slam modes)
--output-prefix           output prefix
--output-dir              output directory
--time-mode               {auto,unix_ns,relative_start}               default: auto
--gps-time-column         {auto,gps_epoch_ns,pi_time_ns}             default: auto
--utm-epsg                horizontal CRS EPSG for output              default: 32614
--sensor-offset-x         sensor position offset X (meters)           default: 0.0
--sensor-offset-y         sensor position offset Y (meters)           default: 0.0
--sensor-offset-z         sensor position offset Z (meters)           default: 0.0
--gps-fusion-orientation  {fixed_yaw,path_tangent}                    default: path_tangent
--sensor-yaw-deg          yaw offset for gps_fusion orientation        default: 0.0
--slam-voxel-size         SLAM voxel size in meters (0=auto)          default: 0.0
--slam-min-range          SLAM min valid range (meters)               default: 1.0
--slam-max-range          SLAM max valid range (meters)               default: 150.0
--slam-deskew-method      {auto,none,constant_velocity,imu_deskew}    default: auto
--anchor-min-motion-m     min motion for XY rotation estimate         default: 5.0
--anchor-z-mode           {offset,none}                               default: offset
--anchor-backend          {pose_optimizer,rigid_fit}                  default: pose_optimizer
--poseopt-key-frame-distance
--poseopt-constraints-every-m
--poseopt-constraint-weights
--poseopt-max-iterations
--poseopt-map-voxel-size
```

---

## 13. Cancellation Pattern

```python
class JobCancelledError(RuntimeError): ...

def _check_cancel(should_cancel):
    if should_cancel is not None and should_cancel():
        raise JobCancelledError("Job cancelled by user.")
```

Called at top of each file loop and each SLAM scan iteration.
On cancel: `.partial` file deleted, temp dirs cleaned up, GUI shows "Cancelled".

---

## 14. Event Kinds (emit_event API)

| Kind | When |
|---|---|
| `job_started` | immediately on entry |
| `job_error` | on exception (before re-raise) |
| `job_cancelled` | on `JobCancelledError` |
| `job_complete` | successful finish |
| `status` | general log messages |
| `file_started` | start of each input file |
| `file_completed` | end of each input file |
| `scan_progress` | every 5 scans (GPS fusion) |

---

## 15. Sample Data

- `SampleData/` — 2-chunk session (2026-02-25)
- `SampleData/mock_gps_for_test.csv` — synthetic GPS for dry-run testing
- `SampleData2/26/` — 9-chunk session (2026-02-26); used for full-session validation
- `SampleData2/26/raw_lidar_20260226_101550_chunk0000_0.json` — sensor sidecar confirming
  `TIME_FROM_SYNC_PULSE_IN`, `INPUT_NMEA_UART`, `lidar_to_sensor_transform`

---

## 16. Current Development State (as of 2026-03-30)

**Complete:**
- Phase 0: living documentation
- Phase 1: backend extraction into package (`jobs/offline_fusion.py`)
- Phase 2: GUI MVP + SLAM modes + pose optimizer + OSF export + reliability hardening pass

**Last significant changes (2026-03-01):**
- Fixed 8 reliability bugs: NaN altitude in GPS loading; `relative_start` clock alignment;
  GPS-range trimming instead of silent extrapolation; `XYZLut(use_extrinsics=True)` consistency;
  LAS header offsets from full GPS trajectory; QA report written after LAS commit;
  OSF writer double-close; UTM EPSG exposed in GUI
- Added GPS fusion orientation modes (`path_tangent` default, `fixed_yaw`)

**Validation runs completed:**
- Single-chunk `gps_fusion`, `slam_map`, `slam_gps_anchor` (both backends): all pass
- Two-chunk `slam_gps_anchor` pose_optimizer: pass (one test: median ~1.77 m, P95 ~4.23 m)
- Full 9-chunk session pose_optimizer: ran and produced output; QA failed (median ~7.65 m) —
  correctly flagged; GPS/clock quality is the limiting factor, not pipeline logic

---

## 17. What Comes Next (Phase 3)

**Primary — packaging/release (not started):**
- PyInstaller builds for Linux, macOS, Windows
- GitHub Actions CI matrix: one job per platform, artifact upload
- This is the top priority

**Secondary (quality of life):**
- Disk-space precheck before long runs (hit `OSError: No space left on device` on 9-chunk test)
- GUI: custom output filename field
- GUI: stale-temp cleanup button
- GUI: one-click output folder open
- Optional trajectory debug CSV (SLAM local + GPS + aligned)
- Pose optimizer weight/iteration controls in GUI advanced panel
- GUI walkthrough screenshots in README
- Unit tests: NaN GPS altitude, relative_start offset, out-of-range timestamp trimming
- User docs: GPS coverage timing (GPS must cover full LiDAR session time range)
- Compound CRS (vertical datum tag) in LAS metadata (low priority)

---

## 18. Architecture Rules — Do Not Break

1. **Scripts are thin wrappers only.** `offline_fuse_lidar_gps.py` and `offline_gui.py`
   contain zero processing logic.

2. **GUI calls `run_offline_job()` in-process.** Never subprocess the CLI from GUI.

3. **No `ouster-cli` subprocess in the processing path.** All Ouster SDK work uses Python
   APIs. The only subprocess is the optional "Open in Viz" launch for visualization.

4. **Never overwrite output files.** Auto-increment is the contract.

5. **No silent failures.** GPS interpolation issues are either logged+trimmed (edge cases)
   or raised as hard errors. No silent clamping.

6. **QA JSON is written only after `os.replace()` commits the LAS.**

7. **Update `docs/memory.md`** when you make architectural decisions, change defaults,
   fix meaningful bugs, or add major features. That file is the living decision log.
