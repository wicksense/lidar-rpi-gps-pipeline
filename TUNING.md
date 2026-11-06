# Offline Processing: Verified Settings and Known Defects

Findings from processing full vehicle-mounted sessions, both OS-1-128 at 1024x20 on
the PTP timing path with DGPS-quality GPS:

- **2026-07-21**, San Antonio residential streets: chunks 1-6, 176 s, 1035 m, one
  0.35 s dropout in chunk 1 (408 packets, 22:07:14.80-15.15 UTC; the cause of the
  pose spike - see [Pose spikes](#pose-spikes-after-a-dropout)). Source of the
  parameter values below.
- **2026-08-26**, downtown San Antonio: 13 chunks, 370 s, 1692 m, with four capture
  dropouts. Source of the capture-gap findings and of the output-field work. The
  raw capture was believed deleted when most of this file was written; complete
  originals were later recovered to `Data\20260826\Pi_original\`, so its figures
  can be re-measured. The short-route regression baseline stays on 2026-07-21
  chunk 1 - see [Regression baseline](#regression-baseline).
- **2026-10-28**, seven block recordings in San Antonio, 44 min in all, 0 of 3,381,248
  lidar packets lost. Source of the loop-closure findings - see
  [Removing drift where a route revisits itself](#removing-drift-where-a-route-revisits-itself).

**Every figure in this file dated before 2026-10-15 was measured with GPS fixes
stamped by `pi_time_ns`, which lags the fix by 0.3 s.** That inflated every
placement error and every "GPS noise" argument built on it. Figures re-measured
with `gps_epoch_ns` are marked as such; the rest are records of the old setting.
See [GPS time](#gps-time-use-the-fix-epoch-not-the-read-time).

This file is the current operating guidance. `HANDOFF.md` describes an earlier
hardware configuration and an earlier CLI and should not be used for parameter
choices.

Read "[What transfers and what does not](#what-transfers-and-what-does-not)" before
reusing the numbers on a different route.

## What was wrong with the early clouds, and what fixed it

The August 2026 clouds were blurred and misplaced. None of it was a fault in the SLAM
algorithm itself (the Ouster SDK's KISS-ICP backend, unmodified); it came from the
settings around it and from how the result was put onto GPS. In the order the fixes were
made:

| Problem | Cause | Fix | Effect | Section |
|---|---|---|---|---|
| Thick surfaces, rings on the ground | Auto-estimated SLAM voxel came out at 1.64 m, coarser than the SDK's own outdoor guidance | `--slam-voxel-size 0.25` | Pure-SLAM surface thickness 1.9-2.1 cm on all four chunks tested | flag table |
| Vehicle body in the cloud | Body returns at 1.1-2.0 m; the 1.0 m default removed almost none | `--slam-min-range 2.5` | Body gone, 1.3 % of points lost | flag table |
| Placement bent the cloud | `pose_optimizer` pulls the trajectory onto GPS every 10 m, writing GPS error into the shape | `--anchor-backend rigid_fit` (2026-09-10) | 5.9 cm to 0.9 cm thickness on a 2026-08-26 chunk; re-measured 2026-10-16 as 4.26 cm against 1.43 cm on the 1 km route | [Placing the cloud](#placing-the-cloud-without-distorting-it) |
| Cloud tilted | SLAM's initial "up" off by under 1 deg, 10.65 m of height ramp over 1 km | `--anchor-z-mode tilt` (2026-09-16) | Ramp removed; confirmed level against 3DEP | [Levelling](#levelling-the-cloud-the-vertical-tilt) |
| Placement 1-2 m off | GPS fixes stamped when the Pi read them, 0.31 s late | `gps_epoch_ns` (2026-10-15) | QA median 1.49 m to 0.35 m, p95 2.32 m to 0.90 m | [GPS time](#gps-time-use-the-fix-epoch-not-the-read-time) |
| 26 m ghost of part of a street | 41 ms of bad poses after a capture dropout | Automatic spike repair (2026-10-15) | QA max 27.4 m to 1.6 m; path length +5.8 % to +0.34 % | [Pose spikes](#pose-spikes-after-a-dropout) |
| SLAM broken by capture gaps | Pi's microSD card stalls and drops packets | Capture onto a USB SSD (2026-10-09) | 0 packets lost in a desk test, and 0 of 3,381,248 over the seven field recordings of 2026-10-28 | [Capture gaps](#capture-gaps-break-slam-and-no-amount-of-tuning-fixes-it) |
| A street driven twice appears twice | SLAM drifts with distance, 20 m by the second pass on a 14-minute recording; a rigid fit cannot bend the route back | Lidar-only loop closure, on by default (2026-11-01) | Passes at most 0.36 m apart on that recording, where they were about 20 m | [Loop closure](#removing-drift-where-a-route-revisits-itself) |

Where it stands, both 1 km routes, 0.1 m export voxel: surface thickness 1.43 cm
(2026-07-21) and 0.93 cm (2026-08-26 downtown); 0.54 cm on the residential route without
thinning. What remains is absolute position, which follows GPS: 0.8-2.4 m horizontally and
4-8 m low against USGS 3DEP. See
[Absolute height](#absolute-height-is-only-as-good-as-gps-altitude-which-is-off-by-metres).
The 2026-10-28 recordings are longer and do less well on the acceptance thresholds: see
[where they stand](#where-the-2026-10-28-clouds-stand). Placed by GPS, they sit 1.3-3.4 m off
3DEP horizontally and are left tilted by GPS altitude's error, by up to 1.9 deg. Six of the
seven were then placed on 3DEP, by scripts not yet in the pipeline.

## Environment

The processing code requires the **ouster-sdk 0.16.x** mapping API. See
`requirements-offline.txt` for the verified combination.

On 0.15.0 every SLAM-based mode fails at runtime on missing APIs
(`SlamEngine.get_point_cloud`, `AbsolutePoseConstraint`,
`PoseOptimizer.initialize_trajectory_alignment`) while `gps_fusion` still runs, so
a wrong SDK version looks like broken code rather than a broken environment.

Loop closure also needs `map-closures`, which has to be installed separately with
`--no-deps`; the reason and the command are in `requirements-offline.txt`. Without it a
run stops at the start and says so, rather than after SLAM has run for an hour.

## Recommended invocation

```bash
python3 offline_fuse_lidar_gps.py \
  --processing-mode slam_gps_anchor \
  --anchor-backend rigid_fit \
  --slam-voxel-size 0.25 \
  --slam-min-range 2.5 \
  --poseopt-map-voxel-size 0.1 \
  --utm-epsg 32614 \
  --lidar-raw /path/to/gap_free_stretch \
  --gps-csv /path/to/raw_gps_<session>.csv \
  --output-dir /path/to/output
```

`rigid_fit` replaced `pose_optimizer` as the recommendation on 2026-09-10. See
[Placing the cloud without distorting it](#placing-the-cloud-without-distorting-it)
for the measurements behind that; in short, per-node optimisation against GPS
makes surfaces three times thicker on the 1 km delivery route for a placement gain
that an independent survey cannot confirm (re-measured 2026-10-16 with the correct
GPS time). Keep `pose_optimizer` for routes long enough that SLAM drift exceeds GPS
error, and use the settings in that section.

| Flag | Repo default | Use | Why |
|---|---|---|---|
| `--anchor-backend` | `pose_optimizer` | `rigid_fit` | Places the cloud without reshaping it. Same surface quality as pure SLAM. Re-checked 2026-10-16 with the correct GPS time: `pose_optimizer` scores better on QA but is no better against an independent survey and makes surfaces three times thicker (4.26 cm against 1.43 cm). Only switch back for routes where SLAM drift exceeds GPS error. |
| `--poseopt-constraint-weights` | `0.01,0.01,0.001` | `0.3,0.3,0.1` *(pose_optimizer only)* | The default is 1000x weaker than the SDK's internal trajectory weight (10). On the 957 m 2026-07-21 session that left GPS effectively ignored, at 13.53 m median error. On a 165 m chunk the same default passes at 0.69 m, because SLAM barely drifts over that distance - so the defect is route-length dependent, not absolute. Its real failure is vertical: a Z weight of 0.001 leaves height unanchored, measured at 3.83 m RMSE. Balanced `1,1,1` anchors well but smears surfaces sixfold; `0.3,0.3,0.1` is the best compromise found. |
| `--slam-voxel-size` | `0.0` (auto) | `0.25` | Auto-estimation chose 1.64 m here, above the 0.8-1.5 m the SDK itself documents for outdoor use. That coarse registration map is what produced smeared surfaces and concentric ring artefacts. Pure-SLAM runs at 0.25 m reached 1.9-2.1 cm surface thickness on all four chunks tested. |
| `--slam-min-range` | `1.0` | `2.5` | Returns from the vehicle body sit at 1.1-2.0 m on this rig (identified by their range being constant across frames, std 2.8 cm, while real scene returns at 2.0-2.5 m vary by over 1 m). The 1.0 m default removes essentially nothing. 2.5 m clears the body with margin at a cost of 1.3% of points. |
| `--poseopt-map-voxel-size` | `0.5` | `0.1` | Output thinning only; it does not affect accuracy. At 0.5 m the cloud is too sparse to even measure surface thickness, and the beam pattern on the ground survives thinning as visible rings. 0.1 m is 25x denser and still tractable (~52 M points, 1.5 GB for this session). |
| `--gps-time-column` | `auto` | `auto` (= `gps_epoch_ns`) | The receiver's fix epoch. Until 2026-10-15 this row recommended `pi_time_ns`, on the belief that `gps_epoch_ns` was truncated to whole seconds; it is not, and `pi_time_ns` lags the fix by 0.3 s. See [GPS time](#gps-time-use-the-fix-epoch-not-the-read-time). |
| `--slam-deskew-method` | `auto` | `auto` | Resolves to the SDK's IMU deskew (`InertialIntegrationImuDeskewMethod` in the log). Measured on 2026-07-21 chunk 4, the one with a 131 deg turn, at a 0.1 m export voxel: IMU 1.49 cm surface thickness, `constant_velocity` 3.54 cm, `none` diverged (17.5 m QA median). If the log shows it fell back to constant velocity, expect surfaces about twice as thick. |
| `--loop-closure` | on | on | `rigid_fit` only. Nothing to decide per session: a route with no revisit comes out as it would without it. See [Loop closure](#removing-drift-where-a-route-revisits-itself). |
| `--loop-rotation-weight` | `100` | `100` | How strongly the solve keeps SLAM's rotation between key frames. 10 put 2.4x as much twist into the 14-minute recording and left surfaces thicker (3.83 cm against 3.56 cm). Past 100 the stretch grows quickly: 0.26 mm/m at 100, 4.18 mm/m at 1000. Chosen on one recording. |

`--slam-min-range 2.5` was re-measured on the 2026-08-26 rig and still holds. Taking
300 frames from a moving chunk and looking for pixels that return something almost
every frame at an almost constant range puts the rig's own returns at 1.22-1.76 m
(592 pixels, per-pixel range standard deviation below 0.05 m, 0.58 % of all
returns). The 1.0-1.5 m band has a median standard deviation of 0.016 m; from 1.5 m
outward nothing stays constant. So 2.5 m clears the body with 0.7 m of margin.

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

The export voxel does not change accuracy, but it does change **measured** surface
thickness, so hold it fixed when comparing configurations. The same chunk from the
same run measures 5.9 cm exported at 0.05 m and 8.7 cm at 0.10 m: a coarser voxel
makes each point an average over a larger cube and spreads the neighbourhood the
measurement uses. Reading that difference as an accuracy difference would be wrong.

Below about 0.05 m there is little left to gain. Point spacing in a mobile scan
grows with range - roughly `range x 0.006` for a 128-beam sensor, so 10 cm at 16 m
and 30 cm at 50 m - which means a fine voxel only bites in the near field, where
the data is already dense. Going 0.10 m to 0.05 m on a 165 m chunk multiplied
points by 2.3, not by 8.

## Placing the cloud without distorting it

Anchoring a SLAM cloud to GPS is a trade, and both sides of it have to be
measured.

- **QA residual** says how closely the trajectory follows GPS. Denser and stronger
  constraints always improve it.
- **Surface thickness** says how sharp the cloud is. Denser and stronger constraints
  always make it worse, because every constraint writes a share of GPS's error into
  the shape of the trajectory.

### Re-measured 2026-10-16 with the correct GPS time

The argument below was first made with `pi_time_ns`, which made GPS look accurate
to about a metre when most of that metre was the 0.3 s timing lag. So it was
re-measured on the full delivery route, 2026-07-21 chunks 1-6, with
`gps_epoch_ns`, 10 m constraint spacing, `--poseopt-map-voxel-size 0.1`, everything
else as recommended. The 3DEP rows are independent of GPS; see
[Absolute height](#absolute-height-is-only-as-good-as-gps-altitude-which-is-off-by-metres)
for the reference.

| | `rigid_fit` | `pose_optimizer` `0.3,0.3,0.1` | `pose_optimizer` `1,1,1` |
|---|---|---|---|
| QA median / p95 | 0.35 / 0.90 m | 0.13 / 0.85 m | **0.06 / 0.68 m** |
| QA max | **1.58 m** | 20.6 m | 26.3 m |
| Z RMSE against GPS | 0.93 m | 1.54 m | 0.06 m |
| Surface thickness p50, 0.1 m voxel | **1.43 cm** | 4.26 cm | 4.26 cm |
| SLAM / GPS path length | **+0.34 %** | +5.6 % | +6.7 % |
| Constraints (expected ~104) | - | 111 | 111 |
| 3DEP: shift to match building outlines | 0.81 m | 0.61 m | 0.82 m |
| 3DEP: ICP horizontal shift | 1.33 m | 0.88 m | 1.00 m |
| 3DEP: road minus bare earth, median / MAD | -4.08 / 0.62 m | -4.55 / 0.34 m | -4.55 / 0.35 m |
| 3DEP: sensor height above ground, range of 100 m bins | -1.05 to -4.68 m | +0.98 to -3.40 m | -1.27 to -3.33 m |

**`rigid_fit` stays the recommendation.** Against the independent survey the three
sit within each other's measurement error horizontally - the wall method resolves
about 0.5 m - so the optimiser's 3-6x better QA score buys no placement that can be
demonstrated. What it does buy is a cloud three times thicker, 4.26 cm against
1.43 cm, which is the property classification depends on. The two weightings
thicken the cloud equally on this route; the earlier finding that lower weights
smear less came from a 165 m chunk with `pi_time_ns` and does not hold here.

Two things the optimiser does better, both worth knowing:

- **Its height follows the ground more closely.** The road-minus-3DEP spread is
  0.34 m against 0.62 m, and the sensor-height bins wander over about 2 m instead of
  3.6 m. Constraining height at every node pulls out part of SLAM's own vertical
  drift, even towards a GPS altitude that is itself off - the drift it removes is
  larger than the GPS error it adds at that scale. A rigid fit cannot do this.
- Nothing else. The large QA maxima and path lengths are the unrepaired pose spike:
  spike repair is wired into `rigid_fit` only.

`1,1,1` matching GPS altitude to 0.06 m while sitting 4.5 m below 3DEP ground is the
clearest example yet of why QA cannot be read alone: it is fitting GPS exactly,
GPS's errors included.

### The original comparison, 2026-08-26, `pi_time_ns`

Measured on one 29 s / 165 m chunk of the 2026-08-26 session, everything else
fixed at `--slam-voxel-size 0.25`, `--slam-min-range 2.5`,
`--poseopt-map-voxel-size 0.05`:

| Backend | Spacing | Weights | Constraints | Thickness p50 | QA median | QA max | Z RMSE |
|---|---|---|---|---|---|---|---|
| pure SLAM, no GPS | - | - | - | **1.0 cm** | - | - | - |
| pose_optimizer | 10 m | `1,1,1` | 17 | 5.9 cm | **0.28 m** | 1.1 m | 0.05 m |
| pose_optimizer | 30 m | `1,1,1` | 6 | 5.0 cm | 0.31 m | 1.2 m | 0.12 m |
| pose_optimizer | 100 m | `1,1,1` | 2 | 1.0 cm | 42.35 m | 121.5 m | 2.09 m |
| pose_optimizer | 10 m | `0.3,0.3,0.1` | 17 | 3.6 cm | 0.41 m | 1.4 m | 0.15 m |
| pose_optimizer | 30 m | `0.3,0.3,0.1` | 6 | 2.9 cm | 0.44 m | 1.5 m | 0.16 m |
| pose_optimizer | 10 m | `0.1,0.1,0.1` | 17 | 3.2 cm | 0.55 m | 1.5 m | 0.11 m |
| pose_optimizer | 10 m | `0.01,0.01,0.001` | 17 | 0.9 cm | 0.69 m | 1.21 m | **3.83 m** |
| **rigid_fit** | - | - | - | **0.9 cm** | **0.88 m** | 1.39 m | 0.93 m |

What the table says:

**Weight matters more than spacing.** Loosening spacing alone (10 m to 30 m) barely
moved thickness, 5.9 to 5.0 cm. Lowering weight alone took it from 5.9 to 3.2 cm.
The damage comes from each constraint pulling hard, not from there being many.

**Two constraints is not the same as a rigid fit.** The 100 m row has the thickness
of pure SLAM and a 42 m median error. The pose optimizer treats absolute
constraints as suggestions competing with a prior that keeps the SLAM shape, and
two weak suggestions lose; the trajectory stays near where SLAM left it. Fitting
four parameters over the whole trajectory is a different operation from asking an
optimiser to honour two soft constraints.

**A better QA score can mean a worse cloud.** The `1,1,1` row scores 0.28 m against
GPS that is itself accurate to about a metre. It is fitting noise, and the cloud is
six times thicker for it.

**`rigid_fit` wins on this length of route.** One rotation, two translations and a
height offset, least-squares fitted over the whole trajectory, applied to every
point. Nothing internal can move, so thickness stays at what SLAM produced, and
because all 585 trajectory samples feed the fit, GPS noise averages out rather than
being written into the shape. Positioning lands at 0.88 m, which is where GPS's own
accuracy puts the floor.

The supporting number: the fitted trajectory measures 167.19 m against GPS's
166.64 m, 0.3 % apart. Over this distance SLAM neither drifts nor mis-scales enough
to need correcting, so bending it can only add error.

### When to go back to pose_optimizer

Rigid placement assumes SLAM drift across the segment is small compared to GPS
error. That holds for a few hundred metres and stops holding at some point beyond
it - on a kilometres-long route the drift accumulates and one transform will leave
an end of the route misplaced. The check is to look at QA residual along the route
rather than as a single median: flat means one rigid fit is enough, a bow shape
means the segment needs subdividing or the optimizer.

### Merging multiple chunks

Run SLAM continuously across a whole gap-free stretch and fit once at the end.
Do not anchor each chunk separately and concatenate: chunk boundaries are just the
320 MB point at which the recorder rolled over, they carry no physical meaning, and
each independently anchored piece brings its own ~0.9 m GPS error. Where two of
them meet, a wall can appear twice a metre apart, which downstream classification
reads as a real object. A uniform offset across the whole cloud is harmless by
comparison - classification works on local shape and height above ground.

## Removing drift where a route revisits itself

A rigid fit moves the whole route as one piece, so it cannot bring two passes of the same
street together once SLAM has drifted between them. Measured on 2026-10-28: SLAM's drift
by the second pass reached about 20 m on the 14-minute recording, 3.7 m on the 5-minute
`101705`, and 0.34-0.71 m on the others that revisit. Loop closure measures that drift from
the lidar and removes it before the rigid fit.

**GPS plays no part in it.** GPS has its own error and is unstable, and once SLAM has
drifted, neither SLAM's nor GPS's position says reliably that two passes are on the same
street. GPS is used only at the end, to place the finished cloud.

### What a run does

1. SLAM runs as usual. Every 4th scan is kept, thinned, for what follows.
2. **Find revisits.** MapClosures (Gupta et al., "Effectively Detecting Loop Closures using
   Point Cloud Density Maps", ICRA 2024; `map-closures` 2.2.0, MIT licence) compares a 50 m
   local map started every 10 m of path. Closures can miss a short overlap, so key frames
   within 10 m on SLAM's poses, at least 50 m apart along the path, are also tried.
3. **Measure each revisit.** Point-to-plane ICP between submaps of the two visits, from 18
   starts, because one start can settle 1 m off. A pair is accepted at fitness of at least
   0.3 and an error of at most 0.15 m; a rejected pair is ignored and logged.
4. **Solve.** The SDK's pose graph is solved with SLAM's odometry and the accepted pairs.
   Distance on the corrected poses can then find further revisits, up to 3 rounds.
5. **Apply as a correction.** At each key frame, the change from SLAM's pose to the solved
   pose is interpolated onto SLAM's own column poses. Taking the solver's column poses
   instead thickened surfaces by a fifth, because it replaces SLAM's per-column motion.
6. `rigid_fit` then places the corrected route on GPS, and the raw scans are replayed.

A run costs more with it: 36 minutes to about 5 and a half hours depending on length and on
what else was running, and 15-30 GB of memory on the longer recordings.

### Traps found

These cost the most time and neither shows up as an error:

- **`PoseOptimizer` silently ignores a constraint on the last key frame.** The solve moves
  nothing and reports no failure; the second-last key frame works and so does the first.
  So the last key frame is never paired. This affected routes ending where they began.
- **A pose spike can create a false revisit.** A spike's out-and-back added 55 m to the
  distance along the path, so on 2026-07-21 chunk 1, a straight street, key frames 0.05 s
  apart counted as 50 m apart and bent the route by up to 3 m. Steps over 10 m between key
  frames now do not count as distance. The regression baseline is what caught this.
- **MapClosures depends on where a map starts.** Moving the start by 5-30 m changed 101705
  from 4 closures to 2-3 and 7-10 pairs. That is why the local maps overlap.
- **Do not run loop closure on an exported LAS** (`scripts/loop_closure.py`). It does not
  work reliably; the pipeline's version works on SLAM's scans.

### Results on 2026-10-28

0.1 m export voxel, thickness p50 of all planar surfaces. Pairs are accepted / tried.

| Session | Pairs | SLAM's drift at revisits | Thickness without / with | QA median / p95 / max with |
|---|---|---|---|---|
| `101153` | 1 / 1 | 0.34 m | 1.18 / 1.13 cm | 0.84 / 1.40 / 1.62 m |
| `101705` | 9 / 9 | up to 3.7 m | 1.51 / 1.60 cm | 1.09 / 3.49 / 4.48 m |
| `102349` | 38 / 39 | about 20 m | 2.93 / 3.54 cm | 2.45 / 3.75 / 4.19 m |
| `103912` | 1 / 1 | 0.36 m | 1.89 / 1.82 cm | 1.59 / 4.60 / 4.83 m |
| `104427` | 1 / 1 | 0.71 m | 2.18 / 2.23 cm | 0.60 / 1.68 / 2.02 m |
| `105117` | 0 | none found | 1.90 cm, no change | 0.53 / 1.15 / 1.33 m |
| `105703` | 0 | none found | 1.08 cm, no change | 2.25 / 9.97 / 20.11 m |

On `102349`, the only recording with a large drift, passes that were about 20 m apart end
at most 0.36 m apart. The two with no revisit match a run without loop closure to within
3 mm at the QA median, which is the evidence that it leaves such routes alone.

What this has not shown:

- **Why surfaces are thicker with it,** `102349` 3.54 cm against 2.93 cm and `101705` 1.60
  against 1.51 cm. Unknown. On both, the version whose revisits were started from GPS had the
  same excess, so it does not come from how the revisits are found.
- **That the correction is accurate, as opposed to consistent.** The pass overlay and the
  residuals reuse the scans that were fitted. Only `101153` and `104427` were checked on
  key frames left out of the fit (within 2.3 cm).
- **That its thresholds hold on other routes.** No false pair remains on these seven
  recordings or the baseline, but the acceptance limits were set on this data. Look at
  a new route's `loop_*` figures and, where it has revisits, at the cloud.

SLAM does not repeat exactly between runs: 1 key frame out of about 1660 and median 2.4 mm,
at most 9.6 cm, on `102349`. Small differences between two runs are not evidence of anything.

### Reading a run

In `extra` of the QA report: `loop_map_closures`, `loop_pairs_found`,
`loop_constraints_added`, `loop_constraints_failed`, `loop_drift_median_m`,
`loop_drift_max_m`, `loop_key_frame_shift_max_m` and `loop_solved_residual_max_m`. The last
should be small; a large shift with a large residual means the constraints disagree.
`--loop-debug-dir` keeps each pair's submaps for inspection.

### Where the 2026-10-28 clouds stand

The QA report's own `pass` is true on all seven, but it only applies its loose limits, 5 m
median and 10 m p95. Against the four checks in `VALIDATION.md` only `105117` passes
everything. These are the figures outside a threshold, with the threshold in the header:

| Session | Median < 0.6 m | p95 < 1.2 m | Max < 3 m | Z RMSE < 1.5 m | Time offset within 0.1 s | Thickness <= 2 cm |
|---|---|---|---|---|---|---|
| `101153` | 0.84 | 1.40 | | | | |
| `101705` | 1.09 | 3.49 | 4.48 | | | |
| `102349` | 2.45 | 3.75 | 4.19 | 2.31 | -0.34 | 3.54 |
| `103912` | 1.59 | 4.60 | 4.83 | 3.88 | -0.26 | |
| `104427` | 0.60, on the limit | 1.68 | | | | 2.23 |
| `105117` | | | | | | |
| `105703` | 2.25 | 9.97 | 20.11 | 2.52 | 0.26 | |

Coverage and height distribution pass on all seven (100 %, at most 0.70 % out of band).
`VALIDATION.md` treats a breach as a reason to look, and its limits were set on one route
with good GPS. The horizontal and height figures measure agreement with GPS, whose error
on these routes has not been measured; for `105703` the evidence that its 20 m is GPS's
error is in that cloud's README. Why three sessions show a best time offset of about 0.3 s
has not been investigated. Fitted tilts are 1.35-4.32 deg, far above the "fraction of a
degree" under [Levelling](#levelling-the-cloud-the-vertical-tilt) and close to the 5 deg
limit; why SLAM's "up" is that far off is not explained.

**Checked against 3DEP on 2026-11-01** (TX Central B2 2017, class 2 ground, 2.5 m radius).
The sensor is bolted to the vehicle, so its height above ground should be constant. A plane
in (x, y) was fitted to that height along the trajectory, and likewise to GPS altitude above
the same ground. "Left" is the standard deviation after removing the plane.

| Session | Cloud: plane | Cloud: left | GPS: plane | GPS: left | Z RMSE against GPS | Road minus 3DEP ground | Wall shift, whole route |
|---|---|---|---|---|---|---|---|
| `101153` | 1.20 deg | 0.12 m | 1.20 deg | 0.40 m | 0.36 m | -4.69 m | 3.41 m |
| `101705` | 0.25 deg | 0.41 m | 0.28 deg | 1.13 m | 1.04 m | -3.85 m | 2.98 m |
| `102349` | 0.81 deg | 0.51 m | 0.80 deg | 2.28 m | 2.31 m | -3.16 m | 2.28 m |
| `103912` | 1.88 deg | 0.33 m | 2.02 deg | 3.66 m | 3.88 m | +4.02 m | 1.80 m |
| `104427` | 1.02 deg | 0.12 m | 1.07 deg | 1.24 m | 1.19 m | +0.12 m | 1.30 m |
| `105117` | 0.29 deg | 0.20 m | 0.26 deg | 1.38 m | 1.19 m | -1.33 m | 2.61 m |
| `105703` | 0.17 deg | 2.25 m | 0.26 deg | 1.42 m | 2.52 m | -2.59 m | 2.33 m |

What this shows:

- **On six of the seven, SLAM's height is better than GPS's.** Once one plane is removed the
  cloud follows 3DEP ground to 0.12-0.51 m, GPS to 0.40-3.66 m. `105703` is the exception:
  2.25 m against 1.42 m, cause not established.
- **The tilt left in the cloud is GPS altitude's.** The cloud's plane matches GPS's on every
  session, as expected when the tilt is fitted to GPS. So the fit passes on any tilt in GPS's
  own error, which is why heights wander by up to 8.8 m along `103912`. A linear trend
  along the path misses this on a block loop, because the route comes back.
- **`z_rmse_m` is mostly GPS altitude's own noise.** On six of the seven it is within about
  0.2 m of GPS's scatter about the same plane. It says little about the cloud.
- **The height offset differs by session,** from -4.69 m to +4.02 m at the road, so these
  clouds do not meet in height where they adjoin.
- **The horizontal offset is 1.3-3.4 m and has a pattern.** Easting -1.1 to -2.6 m on all
  seven, northing mostly positive. Not explained. The wall method resolves about 0.5 m and
  carries an eaves bias; translation-only ICP gives 1.4-2.4 m. 2026-07-21 reached 0.81 m.

3DEP was flown in 2017, so ground rebuilt since then is wrong in it. Files: `checks\` of each
cloud; the plane fits are `G:\Lidar\Data\20261028\Processed\height_source_1028.json`.

**Placed on 3DEP on 2026-11-01.** One rigid transform per cloud after `rigid_fit`: a plane
through road minus 3DEP ground removes tilt and height, and a shift onto building footprint
edges sets the horizontal position. On areas left out of the fit, road minus 3DEP drops to
0.12-0.78 m on six sessions (from 1.33-4.61 m). Horizontal is about 1 m and not proved.
`105703` is not fixed: it leans 1.3-2.7 deg across the route in every 300 m window, so the
lean turns with the route and one rigid transform cannot remove it. Walls, which use neither
3DEP nor GPS, and the road against 3DEP agree on this; why SLAM leans is not established:
constant-velocity deskew, dropping the stationary first chunk and a 0.5 m SLAM voxel each
left it at 1.56-1.79 deg (1.72 before), and constant velocity thickened surfaces to 4.11 cm.
The accelerometer shows the same lean as the walls, and the cloud bends with distance at about
3.4 deg/km; `105703` is the only route ending far from its start. Levelling on the accelerometer,
tested on finished clouds, took its sensor height above 3DEP from a std of 2.24 m to 0.08 m and
added 0.25-0.50 cm of thickness on all seven: `G:\Lidar\Data\20261028\Processed\imu_gravity_check\README.md`. A
whole-route tilt check averages this away on a route that turns, so check per window. The
scripts are not in the pipeline yet. Details:
`G:\Lidar\Data\20261028\Processed\3dep_align_trial\README.md`.

The clouds and which to use are listed in `G:\Lidar\Data\20261028\README.md`. They come from
successive versions of the code: `101705`, `102349`, `103912` from the version before the
last two trap fixes, `101153` and `104427` with the last-key-frame fix, `105117` and
`105703` from before loop closure was added (rerun with it, they found no revisits). The
last fix only changes routes with a pose spike or a key-frame step over 10 m, and none of
these has one.

## After fusion: placing, aligning, and streets covered twice

Since 2026-11-05 every fused cloud (`rigid_fit`, `--gravity-level`) goes through four more
steps. GPS only gives a first position; the position delivered comes from airborne lidar.

```bash
python3 place_on_airborne.py --las <fused.las> --trajectory <fused>_trajectory.csv \
  --tiles <StratMap 2021 tiles> --epoch 2026.82 --output-dir placed/      # one per drive
python3 align_drives.py --las placed/*.las --output-dir aligned_raw/      # all drives together
python3 flag_overlap.py --las aligned_raw/*.las --output-dir aligned/
python3 check_drives.py --las aligned/*.las --placement-dir placed/ --output-dir checks/
```

- **Placement** (`jobs/airborne_placement.py`): one rigid shift per drive onto StratMap 2021
  ground and building edges. Height on held-out tiles 0.03-0.13 m on 2026-10-28.
- **Alignment** (`jobs/drive_alignment.py`): where drives share streets, an east/north shift
  every 50 m along each route, solved over all drives together. Squares that contradict the
  rest are weighted down (`--robust-m 0.3`). Weighting all squares alike left 101705 and
  102349, which meet only at a bare junction, 1.1-1.6 m apart; with it, 0.40 m. Median over
  all held-out squares 0.13 m. The squares measured at the overpass junction of 104427 and
  105703 stay 1.5-2 m apart and are set aside.
- **Streets covered twice** (`jobs/overlap_flags.py`). Within one drive (it comes back) and
  between drives alike: in every 2 m square, the pass that saw it from closest (smallest mean
  `range_m`) is kept, and every other pass's points there get the LAS overlap flag. A pass is
  one drive's points in the square with no time gap over 60 s. Nothing is deleted.
  Classification skips flagged points, so each place is classified from one copy and a tree is
  not counted twice. Where the kept pass changes from one drive to another, a tree on the seam
  can be split between two files: when totalling trees, merge trees of different drives
  closer than 1 m. Never remove overlap before alignment; the overlap is what aligns the drives.
- **Checks**: limits in `VALIDATION.md`.

## Regression baseline

Changes to the anchoring code are checked against a short single chunk before
anything is run over a full route, because a short chunk finishes in minutes and
most defects show up in the QA numbers rather than in the cloud.

The baseline was one 29 s / 165 m chunk of the 2026-08-26 session while that
capture was thought lost. It is now **2026-07-21 `chunk0000-1`**. Expected values,
measured 2026-10-15 with `gps_epoch_ns` and pose-spike repair:

| | 2026-07-21 chunk0000-1 |
|---|---|
| Length, GPS / SLAM | 120.17 m / 120.21 m |
| `rigid_fit` QA median | 0.26 m |
| `rigid_fit` QA p95 / max | 0.73 m / 0.96 m |
| `rigid_fit` Z RMSE | 1.12 m (tilt is skipped: the chunk is straight) |
| Time offset check | best at -0.01 s |
| Pose spikes repaired | 1, at 22:07:15.155 UTC, 832 column poses, 27.6 m |
| Loop closure pairs | 0 (added 2026-11-01); a straight street has no revisit, so any pair here is false |

The figures this table replaced - 0.884 m median, "~170 m" length - came from
`pi_time_ns` and from a trajectory still carrying the 27 m spike; the spike's out
and back alone added 55 m, which is why the length was misread as 170 m.

Run it with the production settings and `--poseopt-map-voxel-size 0.5`: the QA
numbers are computed from the trajectory before any export happens, so the export
voxel cannot change them and a coarse one makes the check fast.

**What this baseline cannot check.** The first 350 m of the 2026-07-21 route is a
straight northward street, so a single chunk of it never turns. Anything whose
behaviour depends on the route covering two directions - the vertical tilt fit
below is the example - will take its own straight-route path here and has to be
validated on the full L-shaped route instead. A baseline that passes is evidence of
no harm, not evidence that a new correction works.

**A short baseline cannot stand alone, because some errors scale with route
length.** The vertical tilt is the worked example: it is 2.6 m of height ramp over
165 m and 10.65 m over 1031 m, so a short chunk reports a healthy figure for a
defect that makes a full route unusable. Any change to anchoring has to clear a
route of **delivery length** before it is believed, not just the short baseline.
The short one exists to fail fast, not to sign anything off.

## Levelling the cloud: the vertical tilt

SLAM takes "up" from the IMU at the start of a run, and that direction carries a
small fixed error. A `rigid_fit` placement solves four numbers - yaw, easting,
northing, a height offset - and **none of them is a rotation about a horizontal
axis**, so nothing in it can correct that error. It appears instead as a height
residual that grows linearly across the site.

Measured on 2026-07-21 chunks 1-6, 1031 m:

| | Four parameters | |
|---|---|---|
| Z residual at the start of the route | -6.84 m | |
| Z residual at the end | +3.82 m | |
| Span | **10.65 m** | over ground that itself moved only 7.00 m |
| Z RMSE | 3.58 m | |
| Fitted tilt | **0.915 deg** | |
| Z RMSE after removing that tilt | **0.911 m** | |

Over the 165 m chunk the method was first measured on, the same tilt produces
2.6 m of ramp and the height offset absorbs most of it, which is why it went
unseen: the defect scales with route length and the original test route was too
short to show it.

`--anchor-z-mode tilt` fits it. The correction is applied as a **rotation**, not as
a height correction shaped like the fitted plane - adding the plane back would be a
shear, which changes distances; a rotation cannot. The result is still exactly as
rigid as before, which is the whole reason to use this backend.

Two things it deliberately refuses to do:

- **A straight route gets no tilt.** Down one street only the along-track gradient
  is observable and the cross-track one would be fitted out of noise, so the fit is
  skipped unless the trajectory spans `--anchor-tilt-min-span-m` (default 50 m) in
  both easting and northing.
- **An implausible tilt is rejected, not applied.** A real IMU misalignment is a
  fraction of a degree. Past `--anchor-tilt-max-deg` (default 5) the likely causes
  are a broken altitude series or a route on a hillside that the fit has mistaken
  for tilt, and silently rotating the cloud would hide either.

`tilt` is the default. `pose_optimizer` quietly falls back to `offset` under it,
because that backend already solves height at every node and has no single tilt to
correct.

## Absolute height is only as good as GPS altitude, which is off by metres

Everything above judges height against GPS altitude, the same series the fit uses.
Checked on 2026-10-15 against an independent survey, USGS 3DEP "TX Central B2 2017"
airborne lidar (tested 0.03 m vertical RMSE; files and datum conversion in
`Data\USGS_3DEP_TX_Central_B2_2017\`), the height is the weak axis by far:

- **GPS altitude is biased low and wanders.** The roof antenna should read about
  2 m above the 3DEP ground under it. It reads 0.4 to 3.3 m *below*, and the offset
  drifts by 3.7 m over the eight-minute session. A geoid-model difference would be a
  constant, so this is the receiver's vertical solution, not a datum mismatch.
- **The tilt is real.** With `offset`, sensor height above 3DEP ground runs from
  -7.2 m to +1.7 m along the 1089 m route, 8.9 m/km. With `tilt` the trend falls to
  -1.9 m/km. So the 0.92 deg correction is doing the right thing.
- **What remains is waviness and an offset.** After `tilt`, sensor height above
  ground still varies between -1.2 and -4.6 m in 100 m bins, and the whole cloud
  sits about 4 m low: our road surface is 4.09 m below 3DEP bare earth (median of
  15,717 one-metre cells, MAD 0.62 m). A rigid transform can only add a plane, so
  the waviness is SLAM's own vertical drift; the offset is GPS altitude.

None of this touches height above local ground, which is what classification uses,
and none of it changes with `--gps-time-column`. It does mean **absolute heights from
this pipeline are good to a few metres, not to the 0.9 m `z_rmse_m` suggests** -
that figure measures agreement with a GPS altitude that is itself several metres
out. Anything compared against an external elevation product needs a vertical
reference other than GPS.

## Capture gaps break SLAM, and no amount of tuning fixes it

A dropout during capture leaves a hole in the LiDAR stream that scan-to-map
registration cannot bridge. At 5 m/s a 1.5 s hole is 7.5 m of unobserved motion:
the first scan after it registers against a map that no longer overlaps, the
resulting pose is wrong, and once that pose is folded into the SLAM map the
trajectory never recovers. Everything after the gap is unusable.

Observed on the 2026-08-26 downtown session (session `20260826_094253`, 13 chunks,
370 s, 1692 m). Four dropouts totalling 6.7 s fell inside a fifteen-second window:

| Time (UTC) | Gap | Chunk |
|---|---|---|
| 14:44:54 | 1.51 s | `chunk0000-3` |
| 14:44:59 | 0.40 s | `chunk0000-4` |
| 14:45:04 | 2.34 s | `chunk0000-4` |
| 14:45:09 | 2.46 s | `chunk0000-4` |

SLAM diverged at 14:44:50 and stayed diverged for the remaining 4 min 20 s, 70 % of
the session. The same data processed as gap-free stretches is fine: a single clean
chunk reaches 0.23 m median / 0.71 m p95.

### How it shows up

Not in the QA report, which only compares the finished trajectory against GPS and
has no idea the trajectory was built across a break. It shows up in the **number of
GPS constraints**. Constraints are placed every `--poseopt-constraints-every-m` of
distance travelled *along the SLAM trajectory*, so a diverged trajectory generates
them without limit - the counter clears the spacing again every few columns:

| | Constraints |
|---|---|
| Expected for a 1692 m track at 10 m spacing | ~170 |
| Healthy 8-chunk run | 309 |
| Run spanning the gaps | 218,600 and still climbing |

Left unbounded that turns a run that should fail in minutes into hours of solving a
hopeless problem. The pipeline now aborts once the count exceeds twenty times what
the GPS track can justify, and names the time at which it happened.

### What to do about it

Split the session at the gaps and process each continuous stretch as its own run.
SLAM only needs continuity *within* a run; GPS supplies the absolute frame, so the
outputs land in the same CRS and merge by concatenation with no further
registration. The cost is that each stretch carries its own independent error, so
where two of them cover the same ground the surfaces may disagree by a metre or
two.

For the 2026-08-26 session that means:

| Segment | Chunks | Window | Length |
|---|---|---|---|
| A | `chunk0000` … `chunk0000-3` | 14:42:58 - 14:44:57 | 119 s |
| dropped | `chunk0000-4` | 14:44:57 - 14:45:31 | 34 s, holds three of the four gaps |
| B | `chunk0000-5` … `chunk0000-12` | 14:45:31 - 14:49:08 | 217 s |

Gaps are now detected while scans are read, warned about individually and in
summary, and recorded under `extra.capture_gaps` in the QA report. The threshold is
`CAPTURE_GAP_THRESHOLD_S`, 0.5 s.

Gaps are packet loss at capture time, not a transfer problem: the timestamps come
from the sensor clock. They are invisible in file sizes and in anything that only
checks that a pcap parses.

### Pose spikes after a dropout

A gap short enough for SLAM to survive can still leave a mark. 2026-07-21 chunk 1
lost 408 packets between 22:07:14.80 and 22:07:15.15 UTC - six whole frames plus
two partial ones, about 0.35 s. For the next 41 ms, 832 column poses, the SLAM
output fell back to the run's **starting pose**, 27.6 m behind the vehicle, and
then resumed where the vehicle actually was. Every point measured in those 41 ms
was replayed through the wrong pose and landed as a displaced ghost at plausible
heights. It is the "transient pose spike at t = 7.4 s" earlier deliverables
documented as unexplained.

`rigid_fit` now finds such excursions and repairs them before the replay
(`_find_transient_pose_spikes`, `_repair_pose_spikes`). A jump counts only if it is
both large (over 2 m between consecutive poses) and impossibly fast (over 40 m/s),
and only if the trajectory comes back within 2 s; the poses inside are replaced by
interpolation between the good poses either side, linear in position and slerp in
rotation. The points are kept, because they are genuine measurements with wrong
poses. A jump that never comes back is reported as a pose step and left alone.
Each repair is logged and listed under `extra.pose_spikes_repaired` in the QA
report.

On the 1-6 route: QA maximum 27.7 m to 1.58 m, SLAM path length 1093.3 m to
1038.3 m against GPS's 1034.9 m (+0.34 %). For the points recorded in the window,
the distance from the true sensor position (interpolated from outside the window)
now matches each point's measured range to a median 0.01 m, against 8.2 m before,
when 58 % were more than 5 m out. Surface thickness and every other check unchanged.

## GPS time: use the fix epoch, not the read time

The GPS CSV carries two timestamps per fix:

- `gps_epoch_ns` - the instant the receiver computed the position, from the NMEA
  sentence. At 1 Hz the receiver fixes on whole GPS seconds, which is why the values
  look truncated; they are not. `ublox_i2c_chrony_bridge.py` keeps any fractional
  seconds the sentence carries.
- `pi_time_ns` - when the Pi finished reading that sentence over I2C.

The second lags the first by 0.31 s on average (0.11-0.36 s) on 2026-07-21 and by
0.25-0.28 s on both 2026-08-26 sessions. At the route's 6-9 m/s that puts each fix
about 2 m behind where the vehicle was. Because the Pi clock follows GPS PPS through
chrony, the LiDAR's PTP time and `gps_epoch_ns` are both UTC, so `gps_epoch_ns` is
the one that matches.

Measured on 2026-07-21 chunks 1-6, `rigid_fit`, everything else fixed:

| GPS time column | QA median | QA p95 | Surface thickness (0.1 m voxel) |
|---|---|---|---|
| `pi_time_ns` (recommended here until 2026-10-15) | 1.49 m | 2.32 m | 1.42 cm |
| `gps_epoch_ns` | **0.34 m** | **0.91 m** | 1.41 cm |

A sweep confirms it from the data alone: refitting the same trajectory with the GPS
times shifted in 0.05 s steps puts the minimum residual at 0 to -0.05 s from
`gps_epoch_ns`. On single chunks the improvement is larger - chunk 4, 0.11 m median.
Independently of GPS, aligning against 3DEP building outlines needs a 1.42 m shift
for the `pi_time_ns` cloud and 0.81 m for the `gps_epoch_ns` one.

`rigid_fit` now runs that sweep on every run and records the best shift in the QA
report as `gps_time_offset_best_s`, warning if it is more than 0.1 s from zero, or
if no shift gets the residual below 2 m, which points to a broken trajectory
rather than a clock.

## What the exported LAS carries

The `pose_optimizer` backend writes LAS 1.4 **point format 8**. The other two
backends still write format 6 with X/Y/Z only, which is one more reason to prefer
`pose_optimizer`.

| Field | Contents |
|---|---|
| `X` `Y` `Z` | UTM easting/northing in metres, orthometric-ish height from GPS MSL |
| `gps_time` | Capture time, Unix seconds |
| `intensity` | Sensor reflectivity, 0-255 |
| `nir` | Ambient near-infrared |
| `range_m` | Sensor-to-point range in metres (extra dimension) |
| `user_data` | Beam row, 0-127 |
| `classification` | Unused, reserved for downstream labelling |

`nir` is passive solar radiance near 865 nm rather than a return from the sensor's
own laser, so vegetation is far brighter in it than asphalt or concrete. The SDK
reports the sensor's 8-bit measurement already shifted left by four bits; values are
therefore multiples of 16 running 0-4080, and are stored unscaled to stay lossless
and to match what other Ouster tooling displays. It depends on sun angle and
shadowing, so it is comparable within a session but not across sessions.

`range_m` and `user_data` exist because point density falls off as 1/range² and
varies per beam. Any density-dependent step needs the viewing geometry, not just
the point.

**These are voxel averages.** `voxel_downsample` averages attributes over the points
that share a voxel, which is why no discrete identifier such as a frame number is
carried: averaging one is meaningless. `user_data` is exact only when downsampling
is disabled with `--poseopt-map-voxel-size 0`.

### The trajectory CSV

Each `pose_optimizer` run also writes `<output>_trajectory.csv`: `time_ns`,
`time_s`, `x`, `y`, `z`, `roll_deg`, `pitch_deg`, `yaw_deg`.

The point cloud records where each point is but not where it was seen from, and
that viewing geometry is what incidence angle, occlusion reasoning and density
normalisation all need. Carrying a sensor position on every point would be
wasteful, so the trajectory is written once and a point is matched to it by
`gps_time`. Rows are strided down from the per-column poses to at most 50,000,
which stays better than per-scan resolution for sessions up to roughly forty
minutes.

## Acceptance checks

These live in `VALIDATION.md`, with the thresholds, the reasoning behind the two
that are easy to misread, and the scripts that measure them. Do not keep a second
copy here: one of them has already drifted from the other.

The short version, because it decides whether a run was worth doing: the QA report
is necessary and not sufficient. It compares the **trajectory** against GPS and
never inspects the exported points, so it cannot see a truncated cloud or one whose
poses were twisted. For `pose_optimizer` it is also partly self-reporting, since
the optimiser minimises the very residual QA measures - raising constraint weights
always improves the score whether or not the map improves. Measured: the
configuration scoring best on QA, 0.28 m, produced the worst cloud
in the comparison at 5.9 cm surface thickness, against 0.9 cm for the configuration
scoring 0.88 m.

Only the cloud itself settles map quality. Load it and look for doubled walls,
stepped ground, or concentric rings. QA is blind to all three.

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

Fixed while processing the 2026-08-26 session:

- **Nothing bounded the GPS constraint count.** A diverged SLAM trajectory made the
  spacing rule fire on almost every column; one run reached 218,600 constraints and
  was still growing after 70 minutes, having stopped being salvageable an hour
  earlier. The count is now capped against the GPS track length and the failure
  names the time of divergence.
- **Capture gaps were neither detected nor reported.** See
  [Capture gaps break SLAM](#capture-gaps-break-slam-and-no-amount-of-tuning-fixes-it).
- **The near-infrared channel was discarded.** The sensor profile is
  `RNG15_RFL8_NIR8` and the pipeline read only range and reflectivity.
- **Range and beam row were discarded**, leaving no way to reconstruct viewing
  geometry, and no sensor position was written anywhere, so incidence angle and
  occlusion were not recoverable from the output at all.
- **The CLI was silent through the long phases.** `run_from_args` emits progress
  throughout, but `main` passed no emitter, so a full session printed nothing
  between the first file and the final QA line - over an hour with no way to
  distinguish a working run from a hung one.
- **A zero-byte metadata sidecar aborted the run.** A failed transfer can leave one
  beside a perfectly good pcap, and `open_source` then fails with "empty metadata
  passed in" rather than looking further. Every chunk of a session carries a
  byte-identical sensor configuration, so any non-empty `*_0.json` in the same
  directory now serves as a fallback.

Reported but not fixed:

- **The replay accumulates the whole cloud in memory.** Incremental voxel
  downsampling bounds it to the thinned size rather than the raw point count, but a
  217 s segment at `--poseopt-map-voxel-size 0.1` still peaked around 8 GB on a
  17 GB machine. A ten-minute session will not fit. The fix is to write the LAS
  incrementally or tile spatially; until then, raise the export voxel size on long
  runs and re-export a fine-voxel copy of just the area of interest.

Fixed on 2026-09-10:

- **`rigid_fit` truncated long routes and threw away every field but position.**
  Both symptoms had one cause: it built its cloud from
  `SlamEngine.get_point_cloud()`, the SLAM backend's rolling local map. That map
  prunes older voxels as the sensor moves away, which is why a six-chunk run
  reported `pass` at a 1.25 m median while its cloud spanned 288x263 m against a
  563x446 m trajectory, centred on the end of the route. It also stores nothing but
  coordinates, because registration has no use for reflectivity - so the
  attributes were never available to write, rather than being dropped at export.
  `rigid_fit` now applies its transform to the SLAM poses and replays the raw scans
  through the same path `pose_optimizer` uses. Transforming poses rather than points
  keeps the result exactly as rigid as before. On the reference chunk the output
  went from 8.3 M points with X/Y/Z only to 16.8 M points carrying the full field
  set, with thickness unchanged at 0.9 cm and QA unchanged at 0.88 m.
  `--reflectivity-filter` and `--poseopt-map-voxel-size` now apply to this backend
  too.

Fixed on 2026-10-15:

- **The GPS fixes were stamped with the time the Pi read them, not the time they
  were taken.** `pi_time_ns` lags by 0.3 s, and both this file and the manifest
  preference in the code chose it. The manifest preference and `auto` now pick
  `gps_epoch_ns`, as do the defaults of `check_coverage_height.py`,
  `crop_convergence.py` and `plot_session_mode_stability.py`. See
  [GPS time](#gps-time-use-the-fix-epoch-not-the-read-time).
- **Pose spikes after a dropout were left in the cloud.** `rigid_fit` now repairs
  them. See [Pose spikes](#pose-spikes-after-a-dropout).
- **Nothing checked the two clocks against each other.** `rigid_fit` now records
  the best-fitting GPS time shift in the QA report.

Fixed on 2026-11-01:

- **Streets driven twice appeared twice.** Added lidar-only loop closure, including the
  fixes for the last-key-frame and pose-spike traps. See
  [Loop closure](#removing-drift-where-a-route-revisits-itself).

Not fixed in the pipeline:

- **`pose_optimizer` does not repair pose spikes**: it takes its column poses from
  the optimiser rather than from the SLAM scans, and the repair has only been wired
  into `rigid_fit`.
- **Absolute height follows GPS altitude**, which 3DEP shows to be several metres
  out. See [Absolute height](#absolute-height-is-only-as-good-as-gps-altitude-which-is-off-by-metres).

Reported but not fixed, because the fix is a design decision:

- **`slam_map` still builds its cloud from `SlamEngine.get_point_cloud()`** and so
  keeps both of the problems `rigid_fit` just shed. It is a diagnostic mode rather
  than an output path - it is what pure-SLAM thickness is measured on - so the
  truncation matters less, but the same replay would fix it.

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
- **Packet loss during capture is the failure that costs the most.** A few seconds
  of dropped UDP is a rounding error as a fraction of a session but severs the SLAM
  chain, and everything after the first gap is lost rather than degraded. On
  2026-08-26 that turned 6.7 s of loss into 4 min 20 s of unusable poses. It is
  worth watching the capture host's network and disk headroom for this reason
  alone; nothing in offline processing can recover it.

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
