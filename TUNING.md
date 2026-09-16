# Offline Processing: Verified Settings and Known Defects

Findings from processing full vehicle-mounted sessions, both OS-1-128 at 1024x20 on
the PTP timing path with DGPS-quality GPS:

- **2026-07-21**, San Antonio residential streets: 6 chunks, 176 s, 957 m, no
  dropouts. Source of the parameter values below.
- **2026-08-26**, downtown San Antonio: 13 chunks, 370 s, 1692 m, with four capture
  dropouts. Source of the capture-gap findings and of the output-field work.
  **The raw capture for this session no longer exists**, so every figure below that
  came from it is a record, not something that can be re-measured. The short-route
  regression baseline has moved to 2026-07-21 chunk 1 - see
  [Regression baseline](#regression-baseline).

This file is the current operating guidance. `HANDOFF.md` describes an earlier
hardware configuration and an earlier CLI and should not be used for parameter
choices.

Read "[What transfers and what does not](#what-transfers-and-what-does-not)" before
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
  --anchor-backend rigid_fit \
  --slam-voxel-size 0.25 \
  --slam-min-range 2.5 \
  --poseopt-map-voxel-size 0.1 \
  --gps-time-column pi_time_ns \
  --utm-epsg 32614 \
  --lidar-raw /path/to/gap_free_stretch \
  --gps-csv /path/to/raw_gps_<session>.csv \
  --output-dir /path/to/output
```

`rigid_fit` replaced `pose_optimizer` as the recommendation on 2026-09-10. See
[Placing the cloud without distorting it](#placing-the-cloud-without-distorting-it)
for the measurements behind that; in short, per-node optimisation against
metre-accurate GPS smears surfaces by a factor of six for an accuracy gain that is
inside GPS's own noise. Keep `pose_optimizer` for routes long enough that SLAM
drift exceeds GPS error, and use the settings in that section.

| Flag | Repo default | Use | Why |
|---|---|---|---|
| `--anchor-backend` | `pose_optimizer` | `rigid_fit` | Places the cloud without reshaping it. Same surface quality as pure SLAM, positioning inside GPS's own error. Only switch back for routes where SLAM drift exceeds GPS error. |
| `--poseopt-constraint-weights` | `0.01,0.01,0.001` | `0.3,0.3,0.1` *(pose_optimizer only)* | The default is 1000x weaker than the SDK's internal trajectory weight (10). On the 957 m 2026-07-21 session that left GPS effectively ignored, at 13.53 m median error. On a 165 m chunk the same default passes at 0.69 m, because SLAM barely drifts over that distance - so the defect is route-length dependent, not absolute. Its real failure is vertical: a Z weight of 0.001 leaves height unanchored, measured at 3.83 m RMSE. Balanced `1,1,1` anchors well but smears surfaces sixfold; `0.3,0.3,0.1` is the best compromise found. |
| `--slam-voxel-size` | `0.0` (auto) | `0.25` | Auto-estimation chose 1.64 m here, above the 0.8-1.5 m the SDK itself documents for outdoor use. That coarse registration map is what produced smeared surfaces and concentric ring artefacts. Pure-SLAM runs at 0.25 m reached 1.9-2.1 cm surface thickness on all four chunks tested. |
| `--slam-min-range` | `1.0` | `2.5` | Returns from the vehicle body sit at 1.1-2.0 m on this rig (identified by their range being constant across frames, std 2.8 cm, while real scene returns at 2.0-2.5 m vary by over 1 m). The 1.0 m default removes essentially nothing. 2.5 m clears the body with margin at a cost of 1.3% of points. |
| `--poseopt-map-voxel-size` | `0.5` | `0.1` | Output thinning only; it does not affect accuracy. At 0.5 m the cloud is too sparse to even measure surface thickness, and the beam pattern on the ground survives thinning as visible rings. 0.1 m is 25x denser and still tractable (~52 M points, 1.5 GB for this session). |
| `--gps-time-column` | `auto` | `pi_time_ns` | Required whenever `--lidar-raw` is used without a manifest. `gps_epoch_ns` comes from NMEA and is truncated to whole seconds, which at 5 m/s is up to 5 m of along-track error, and `auto` prefers it. With `--manifest-json` the manifest's `timing_architecture` field makes `auto` pick `pi_time_ns` on its own. |

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
  always make it worse, because GPS is accurate to about a metre and every
  constraint writes a share of that noise into the shape of the trajectory.

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

## Regression baseline

Changes to the anchoring code are checked against a short single chunk before
anything is run over a full route, because a short chunk finishes in minutes and
most defects show up in the QA numbers rather than in the cloud.

The baseline was one 29 s / 165 m chunk of the 2026-08-26 session until that
capture was deleted. It is now **2026-07-21 `chunk0000-1`**, which is close enough
that the two are interchangeable for this purpose:

| | 2026-08-26 chunk0000-5 (gone) | 2026-07-21 chunk0000-1 |
|---|---|---|
| Length | 165 m | ~170 m |
| `rigid_fit` QA median | 0.88 m | 0.884 m |
| `rigid_fit` Z RMSE | 0.93 m | 1.117 m |

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
