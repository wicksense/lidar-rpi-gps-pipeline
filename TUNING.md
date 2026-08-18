# Offline Processing: Verified Settings and Known Defects

Findings from processing a full vehicle-mounted session (2026-07-21, San Antonio
residential streets: 6 chunks, 176 s, 957 m, OS-1-128 at 1024x20 on the PTP timing
path, DGPS-quality GPS with no dropouts).

Everything below was measured on that session. Read
"[What transfers and what does not](#what-transfers-and-what-does-not)" before
reusing the numbers on a different route.

## Environment

The processing code requires the **ouster-sdk 0.16.x** mapping API. See
`requirements-offline.txt` for the verified combination.

On 0.15.0 every SLAM-based mode fails at runtime on missing APIs
(`SlamEngine.get_point_cloud`, `AbsolutePoseConstraint`,
`PoseOptimizer.initialize_trajectory_alignment`) while `gps_fusion` still runs, so
a wrong SDK version looks like broken code rather than a broken environment.

## Recommended invocation

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode slam_gps_anchor \
  --slam-voxel-size 0.25 \
  --slam-min-range 2.5 \
  --poseopt-map-voxel-size 0.1 \
  --poseopt-constraint-weights 1.0,1.0,1.0 \
  --gps-time-column pi_time_ns \
  --utm-epsg 32614 \
  --lidar-raw /path/to/session_dir \
  --gps-csv /path/to/raw_gps_<session>.csv \
  --output-dir /path/to/output
```

| Flag | Repo default | Use | Why |
|---|---|---|---|
| `--poseopt-constraint-weights` | `0.01,0.01,0.001` | `1.0,1.0,1.0` | The default is 1000x weaker than the SDK's internal trajectory weight (10), so the solver effectively ignores GPS. On this session it produced a 13.53 m median error and failed QA; balanced weights gave 0.55 m. Asymmetric weights such as `1,1,0.1` score even better on QA (0.41 m) but distort the poses, throwing 19% of points tens of metres above or below ground - see [QA does not inspect the cloud](#qa-does-not-inspect-the-point-cloud). |
| `--slam-voxel-size` | `0.0` (auto) | `0.25` | Auto-estimation chose 1.64 m here, above the 0.8-1.5 m the SDK itself documents for outdoor use. That coarse registration map is what produced smeared surfaces and concentric ring artefacts. Pure-SLAM runs at 0.25 m reached 1.9-2.1 cm surface thickness on all four chunks tested. |
| `--slam-min-range` | `1.0` | `2.5` | Returns from the vehicle body sit at 1.1-2.0 m on this rig (identified by their range being constant across frames, std 2.8 cm, while real scene returns at 2.0-2.5 m vary by over 1 m). The 1.0 m default removes essentially nothing. 2.5 m clears the body with margin at a cost of 1.3% of points. |
| `--poseopt-map-voxel-size` | `0.5` | `0.1` | Output thinning only; it does not affect accuracy. At 0.5 m the cloud is too sparse to even measure surface thickness, and the beam pattern on the ground survives thinning as visible rings. 0.1 m is 25x denser and still tractable (~52 M points, 1.5 GB for this session). |
| `--gps-time-column` | `auto` | `pi_time_ns` | Required whenever `--lidar-raw` is used without a manifest. `gps_epoch_ns` comes from NMEA and is truncated to whole seconds, which at 5 m/s is up to 5 m of along-track error, and `auto` prefers it. With `--manifest-json` the manifest's `timing_architecture` field makes `auto` pick `pi_time_ns` on its own. |

`--reflectivity-filter` is best left off. It drops returns with reflectivity 1,
which removed only 1% of points here and did not change the stray-point rate at
all, while discarding genuine weak returns from dark asphalt and dark vehicles.

## Two voxel sizes, two different jobs

These are easy to confuse:

- **`--slam-voxel-size`** is the resolution of the reference map that SLAM matches
  each incoming scan against. It determines **how accurately points are placed**.
  Coarse values blur the match, poses drift slightly, and surfaces thicken.
- **`--poseopt-map-voxel-size`** thins the finished cloud on export. It determines
  **file size and visual density only**, never accuracy.

More points is not automatically better: points are only as good as the poses that
placed them, and below the sensor's own precision extra points just resample the
same surface. Exporting this session with no thinning produced 61.6 M points per
29-second chunk at essentially the same measured surface thickness as 0.1 m
thinning.

## Acceptance checks

The QA report alone is not sufficient. On this session three of four configurations
passed QA while only one produced a usable cloud. Run all three checks:

1. **QA report** - `xy_median_m` and `xy_p95_m` against the thresholds in the
   report. Necessary but not sufficient.
2. **Coverage** - sample positions along the GPS track and confirm each has points
   nearby. `slam_map` and `rigid_fit` can return a cloud covering only the last
   part of the route while QA still reports pass; the QA report now also carries
   `map_to_trajectory_span_ratio` for this.
3. **Height distribution** - fraction of points more than 10 m below or 30 m above
   local ground. Under 1% is healthy; the accepted run here was 0.004%. A
   distorted-pose run reached 19%.

Only the point cloud itself can settle map quality. Load it and look for doubled
walls, stepped ground, or concentric rings, all of which QA is blind to.

### QA does not inspect the point cloud

QA compares the *trajectory* against GPS. It never looks at the exported points, so
it cannot see a truncated cloud or one whose poses were twisted by over-weighted
constraints. Treat a passing QA as "the trajectory is where GPS says it should be",
nothing more. It is also partly self-reporting for the `pose_optimizer` backend,
which minimises the very residual QA measures - raising the constraint weights
always improves the score, whether or not the map improves.

## Defects found

Fixed in this branch:

- **`gps_fusion` reflectivity filter never worked.** It indexed the full unmasked
  reflectivity image with a boolean mask sized for the already-masked point arrays,
  raising `IndexError` on every scan. A broad `except` swallowed it and miscounted
  the scan as lacking a reflectivity field, so `--reflectivity-filter` was silently
  inert.
- **Exported LAS carried only X/Y/Z.** `gps_time` and `intensity` were left zeroed
  even though point format 6 carries them and the source data has both. Without a
  per-point time there is no way to trim the unconverged startup period, drop
  stationary stretches, or trace a suspect point back to a capture moment.

Reported but not fixed, because the fix is a design decision:

- **`slam_map` and `rigid_fit` truncate long routes.** Both build their cloud from
  `SlamEngine.get_point_cloud()`, which returns the SLAM backend's rolling local
  map. On the six-chunk session `rigid_fit` reported `pass` with a 1.25 m median
  while its cloud spanned 288x263 m against a 563x446 m trajectory, centred on the
  end of the route. A full fix means replaying the raw scans against the SLAM poses,
  as the `pose_optimizer` backend already does. For now the mismatch is detected,
  warned about, and recorded in the QA report.

Minor: `--anchor-min-motion-m` is accepted and validated by the `pose_optimizer`
backend but never used, while the GUI offers the control for both backends.

## Capture-side notes

- **SLAM needs distance to converge.** The first ~20 s of this session carried
  roughly 3x the trajectory error of the rest (3.04 m median versus about 1 m),
  with the vehicle stationary for the first 6 s. Start recording before the route
  begins and trim the opening with `gps_time`.
- **Point density is inherently uneven.** Angular sampling means points per unit
  area falls off as 1/range², so ground directly beside the track is roughly 36x
  denser than ground 20-30 m away. This is real measurement density, not an
  artefact, but it does mean any density-dependent step (ground classification,
  segmentation, outlier removal, meshing) should run on a uniformly resampled copy.
- **The first chunk of a rolling split may not open in `ouster-cli`.** On this
  session `chunk0000.pcap` failed with "Duplicate lidar port/sn found in pcap" and
  needed an explicit `-m <metadata>.json`. The Python pipeline is unaffected because
  it always passes explicit metadata and `sensor_idx=0`.

## What transfers and what does not

The parameter values are specific to this rig, route and GPS quality. What
generalises is the method:

- Run the cheap `rigid_fit` backend as a control. It has only four degrees of
  freedom and cannot bend to fit GPS, so if it aligns well the SLAM trajectory is
  sound and any problem lies in the anchoring configuration rather than the data.
  (Check its coverage warning before trusting its cloud.)
- Use `ouster-cli source <chunk> slam --voxel-size <v> save out.las` as an
  independent reference. It shares the SDK but not the pipeline, so it separates
  data and SLAM problems from configuration problems.
- Isolate one variable at a time. Several plausible explanations for smeared output
  here - low-reflectivity noise, a tilted mount, attitude error, stopping at
  intersections - were each refuted by a controlled comparison before the coarse
  registration voxel turned out to be responsible.

Re-measure per session or per rig:

| | When it must change |
|---|---|
| `--slam-min-range` | New rig or new sensor mounting. Measure once by finding the range band whose returns stay constant across frames. |
| `--slam-voxel-size` | Different environment scale. Open suburban routes tolerate coarser values than dense residential streets. |
| `--poseopt-constraint-weights` | Materially different GPS quality. Lower them where multipath is severe. |
| `--poseopt-map-voxel-size` | Purely a downstream requirement. |
