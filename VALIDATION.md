# Validation: What "This Run Is Usable" Means

`TUNING.md` records which settings to use and why. This file records how a finished
run is judged, so that a result can be accepted or rejected without rerunning the
argument each time.

Every threshold below is an **operating threshold drawn from a small number of
verified runs**, not a property of the hardware or of the method. They are the
levels at which, on this rig and these routes, a run either has been usable or has
been shown not to be. Treat a breach as a reason to look, not as proof of failure,
and read
"[What transfers and what does not](TUNING.md#what-transfers-and-what-does-not)"
before applying them to a different route.

The figures below are from the 2026-07-21 chunks 1-6 deliverable of 2026-09-13.

---

## Why one number is not enough

The QA report the fusion job writes compares the **trajectory** against GPS. It
never inspects the exported points. That leaves it blind to two failures that
matter, and worse, for the `pose_optimizer` backend it is partly self-reporting:
the optimiser minimises the very residual QA measures, so raising constraint
weights always improves the score whether or not the map improves.

Measured on one 165 m chunk of the 2026-08-26 session, holding everything else
fixed. That capture has since been deleted, so this table is a record rather
than something that can be re-measured:

| Configuration | Surface thickness | QA median | Verdict |
|---|---|---|---|
| `pose_optimizer`, 10 m, `1,1,1` | 5.9 cm | **0.28 m** | best QA score, worst cloud |
| `pose_optimizer`, 100 m, `1,1,1` | **1.0 cm** | 42.35 m | best thickness, cloud in the wrong place |
| `rigid_fit` | **0.9 cm** | 0.88 m | accepted |

The first two rows are each other's counterexample. A cloud can follow GPS
perfectly and be smeared; it can be internally sharp and sit tens of metres off.
Neither number alone separates a usable run from an unusable one, which is why the
checks below are run as a set and reported as a set.

---

## The four checks

Run all four. A run is accepted only if all four pass.

### 1. QA residual

From the `_qa.json` written beside the LAS.

| Metric | Threshold | This deliverable |
|---|---|---|
| `xy_median_m` | below 2.0 m, and **not far below 0.3 m** | 1.492 m |
| `xy_p95_m` | below 2.5 m | 2.324 m |
| `z_rmse_m` | below 1.5 m for `rigid_fit` | 0.924 m |

The 2.0 m figure comes from the reference, not from the cloud. The reference is a
1 Hz non-RTK GNSS series accurate to about a metre, so the threshold is set at
roughly twice that. **This check measures absolute registration and varies with
route geometry**; it is listed separately from the internal-geometry checks for
that reason, and a run can be excellent on one and poor on the other.

The lower bound is the unusual part and it is deliberate. GPS on this rig is
accurate to roughly a metre. A trajectory that matches it to 0.28 m is not more
accurate than GPS; it is **fitting GPS noise**, and the cost of doing so is written
into the shape of the cloud. A median well under 0.3 m is a reason to check surface
thickness before believing the run, not a reason to celebrate.

`z_rmse_m` behaves differently under `rigid_fit` depending on the anchoring mode,
and the threshold assumes `--anchor-z-mode tilt`. With the four-parameter placement
(`offset`) the figure grows with route length rather than staying put, because the
SLAM frame's vertical is tilted by a fraction of a degree and a constant height
offset cannot correct it: 0.93 m over 165 m, 3.58 m over 1031 m, all of it a linear
ramp. See
"[Levelling the cloud](TUNING.md#levelling-the-cloud-the-vertical-tilt)".

**So a passing `z_rmse_m` on a short route says nothing about a long one.** Check it
against the route length, and prefer `tilt` on anything long enough to turn a
corner. Around 0.9 m is what remains once the tilt is removed, and it is acceptable
for classification work, which uses height above local ground. Comparison against an
external elevation product needs separate handling.

### 2. Coverage

Sample positions along the GPS track and confirm each has points nearby. The QA
report carries `map_to_trajectory_span_ratio` for the coarse version of this.

| Metric | Threshold | This deliverable |
|---|---|---|
| Fraction of GPS track with points within 10 m | 100% | 100% (104/104) |
| `map_to_trajectory_span_ratio` | above 1.0 | 1.381 |

This check exists because the `slam_map` and pre-fix `rigid_fit` backends could
return a cloud covering only the last part of a route while QA still reported a
pass. Since `rigid_fit` was changed to replay raw scans through anchored poses, a
coverage shortfall would indicate a pose-lookup problem rather than map truncation,
but the check stays: it is the only one that would catch a cloud that is simply
missing.

### 3. Height distribution

Fraction of points more than 10 m below or 30 m above local ground.

| Metric | Threshold | This deliverable |
|---|---|---|
| Out-of-band point fraction | below 1% | 0.0017% (534 of 31,576,695) |

Reference points: an accepted run reached 0.004%; a run with distorted poses
reached 19%. This check is also what catches a transient pose spike, which
deposits a small cluster of points far from any real surface.

### 4. Surface thickness

The spread perpendicular to locally flat surfaces: the RMS distance from a point's
20 nearest neighbours to the least-squares plane through them, restricted to the
most planar quartile of neighbourhoods. Measure it with `measure_surface_thickness.py`.

**The threshold depends on the export voxel and is meaningless without it.** The
export voxel cannot change accuracy - every point's position is already fixed
before export - but it does change the measured thickness, because a coarser grid
averages each point over a wider area and spreads the neighbourhood the
measurement runs on. Measured on the 2026-07-21 `rigid_fit` cloud:

| Export voxel | All planar | Ground | Wall |
|---|---|---|---|
| none (raw points) | **0.54 cm** | 0.49 cm | 0.91 cm |
| 0.10 m | 1.42 cm | 0.82 cm | 3.39 cm |
| 0.50 m | 19.33 cm | 15.93 cm | 19.71 cm |

The first row is the physical answer; the others are that answer plus the grid the
cloud was thinned onto. At 0.50 m the measurement is reporting the grid and nothing
else.

**So never quote a thickness from a quick-validation export.** Thinning averages
each voxel's points to one position, which divides the real spread by roughly the
square root of the points per voxel before anything is measured. A figure from a
0.5 m export is a property of the voxel grid. The honest number needs
`--poseopt-map-voxel-size 0` on a single chunk, which takes about ten minutes and
is worth doing once per configuration rather than once per run.

| Metric | Threshold | This deliverable |
|---|---|---|
| Thickness p50 at 0.10 m export voxel | at or below 2 cm | 1.42 cm |
| Thickness p50 with no thinning | at or below 1 cm | 0.54 cm |

Always state the export voxel alongside the number. A thickness quoted without one
cannot be compared to anything.

**Measure in local coordinates, never raw UTM.** A northing here is around 3.25e6.
Stored as float32 that value has a representable step of roughly 0.4 m, hundreds of
times coarser than the spread being measured, and every neighbourhood collapses
onto a degenerate plane that reports a thickness of exactly zero. A run of exact
zeros is the signature of this mistake, not of a perfect cloud.
`measure_surface_thickness.py` subtracts the cloud's own corner before converting;
anything else measuring these clouds has to do the same.

---

## Reference item: constraint count

Not a pass/fail check, but the cheapest way to catch a run that is about to waste
an hour. Applies to `pose_optimizer` only; `rigid_fit` adds no constraints and
reports none.

Expected constraints ≈ route length ÷ `--poseopt-constraints-every-m`. A count far
above that means constraints are being generated per sample rather than per
distance, and the run will not produce anything usable. The code caps this, and the
cap was originally set at 20x expected - loose enough that an 11x overrun ran to
completion over 65 minutes before failing. **The cap belongs at 3-5x expected.**

Report the observed count against the expected one, not on its own.

---

## Reference item: trajectory length ratio

`estimated_path_length_m` against `reference_path_length_m` is an attractive drift
indicator and a treacherous one. Two separate things inflate it, and neither is
drift.

**A single transient pose excursion dominates it.** Path length is a sum of
consecutive distances, so an excursion is counted twice - once out, once back. The
2026-07-21 route carries one such event at t = 7.4 s, a 26 m jump lasting 53 ms,
and it alone contributes **55.9 m** to a 1031 m route. That is 5.4% of apparent
drift from 53 milliseconds of bad poses. Check for a spike before reading the ratio
as anything else: sort the trajectory by implied speed between consecutive samples
and look at the top of the list.

**Sampling density adds about 1% more.** The two series are not sampled at the same
rate, and summing point-to-point over a denser one accumulates jitter the sparser
one never sees. It is a real effect but a small one.

Measured on the 2026-07-21 chunks 1-6 trajectory, against GPS's 1030.70 m:

| Resampled to | With the spike | Spike removed |
|---|---|---|
| native (284 Hz in the CSV) | **+6.47%** | +1.12% |
| 20 Hz | +0.68% | +0.68% |
| 5 Hz | +0.62% | +0.62% |
| 1 Hz, matching GPS | +0.07% | +0.07% |

The two columns are the whole point. Coarse resampling does not remove a sampling
bias, it steps over the spike - which is why the left column looked like a 6%
sampling artefact until the spike was subtracted separately.

**So: find and quantify any pose spike first, then allow about 1%, and treat what
is left as drift.** A route with no spike should sit within about 1% at any
sampling rate.

### What is actually being compared

The two backends do not sample this metric the same way, which matters when
comparing runs:

- `rigid_fit` computes it per scan, about 20 Hz. A 176 s route gives 3,515 samples.
- `pose_optimizer` computes it per lidar column, about 20 kHz. The same route gives
  3,594,880 samples.

The trajectory CSV is a third rate again: poses arrive at roughly 20,000 per second
of capture and the writer strides them down to at most 50,000 rows, so its rate
depends on how long the run was - 284 Hz for a 176 s route, 1,688 Hz for a 30 s
one. Any analysis of the CSV has to account for that, and a figure read off one run
cannot be compared to another without checking the stride.

### Worked example: 2026-07-21 chunks 0-6

The same session including chunk 0 reported `estimated_path_length_m` 1396.96 m
against `reference_path_length_m` 1121.16 m, **+24.6%**. Allowing about 1% of
sampling bias and the 5% the known spike contributes still leaves roughly **18% of
real deviation**, and the rest of that run's numbers agree that this one is real:
QA maximum 99.25 m against 26.18 m for chunks 1-6, optimiser cost 1209 against 82,
and 137 constraints against 111. Chunk 0 was excluded and the deliverable was built
from chunks 1-6.

The lesson is the shape of the reasoning rather than the number: read the ratio
only after accounting for spikes and sampling, and only alongside the QA maximum
and the optimiser cost, which have neither problem.

---

## Input-side check: capture gaps

Run before processing, not after. A dropout during capture leaves a hole that
scan-to-map registration cannot bridge, and no downstream setting recovers it.
Scan the packet stream for inter-packet intervals and apply:

| Gap length | Ruling | Basis |
|---|---|---|
| up to 0.1 s | harmless | many 0.03-0.04 s jitters across sessions, no visible effect |
| 0.44 s | survivable | a verified run at 0.55 m median error contained one |
| 0.44 - 0.6 s | **no case law** | worth a dedicated experiment if a gap ever lands here |
| 0.6 s and above | breaks SLAM | 0.6-0.76 s gaps produced trajectory jumps; 1.5-1.7 s always broke |

`CAPTURE_GAP_THRESHOLD_S` is 0.5 s, which sits inside the untested band and is
therefore in the right place: it accepts what has been shown to work and rejects
what has been shown to fail.

An earlier ruling that 0.35 s was already enough to break SLAM has been
**withdrawn**, disproved by the successful 0.44 s run.

### Pose optimisation is global, so bad segments contaminate good ones

A divergent stretch late in a route pulls the trajectory off by hundreds of metres
90 seconds *before* the gap that caused it. Processing a whole session and cropping
the good part out of the result does not work. Clean stretches must be selected on
the input side and processed on their own.

---

## Convergence crop

SLAM needs a short stretch of motion before registration settles, and the points
placed before it are misplaced by a margin that has nothing to do with GPS.
`crop_convergence.py` finds that moment from the trajectory residual and removes
the points recorded before it.

**Run the crop before the four checks, and run the checks on the cropped file
only.** The uncropped numbers describe a cloud that is not being delivered.

Detection is coarse on purpose: bin the run in time, take the median residual per
bin, and cut at the first bin followed by settled bins. A bin median is unmoved by
a handful of outlier columns, so a transient spike is not mistaken for a
convergence boundary. Two guards keep the answer honest - only the start of the run
is searched, so a mid-route GPS excursion can never be selected, and
`--expect-seconds` refuses to write a file when detection lands far from the
expected answer.

**On the backend being delivered, this route needs no crop.** Measured on the
2026-07-21 chunks 1-6 `rigid_fit` run, the first three bins are 1.175, 0.863 and
0.882 m against a steady state of 1.657 m - the *lowest* stretch of the whole
route, because SLAM has not yet had distance to drift. Detection returns 0 s.

The earlier readings that suggested a bad first 10 to 30 s all came from
`pose_optimizer` output, and that is not a measurement of convergence: the
optimiser pulls the trajectory onto GPS from its first constraint, including
through the unconverged stretch, so its residual shows the optimiser disagreeing
with GPS rather than SLAM settling. **Run this check on `rigid_fit` output only.**

| Field | This deliverable |
|---|---|
| Detected convergence | 0.0 s |
| Seconds cropped | none - no bin exceeded the threshold |
| Points removed | none |

---

## Keeping what the numbers rest on

Every threshold in this file was set by measuring a specific capture. When that
capture is gone the number stays in the document but stops being checkable, and
nobody reading it later can tell the difference. That has already happened once:
the 2026-08-26 downtown session was the source of the capture-gap rulings, the
anchoring comparison and the original regression baseline, and its raw capture was
deleted. Those figures are now a record rather than something that can be
re-measured, and the regression baseline had to move to 2026-07-21 chunk 1.

So:

- **Keep the raw capture of any session a published number came from.** That means
  the `.pcap` files, the `raw_gps_*.csv`, and the manifest. Processed output is not
  a substitute - it cannot be reprocessed with different settings, which is the
  whole reason a baseline exists.
- **Before deleting a session, check this file and `TUNING.md` for figures that
  came from it.** If any are load-bearing, either keep the capture or re-derive the
  figure on a session that is being kept, and say in the document which session it
  now comes from.
- **Mark orphaned figures where they appear.** A number whose capture is gone
  should say so inline, so a later reader does not plan a re-measurement that
  cannot happen.
- **A regression baseline needs its capture kept indefinitely.** It is the one
  thing that gets re-run on every code change.
- **Anything cited in a paper needs its capture archived**, not merely kept on the
  working machine: the sample data going to Zenodo covers the demonstration route,
  not the sessions the thresholds came from.

### Current baseline

| | |
|---|---|
| Short-route regression | 2026-07-21 `chunk0000-1` (~170 m) - **keep indefinitely** |
| Full-route deliverable | 2026-07-21 `chunk0000-1` through `-6` (1031 m) - **keep indefinitely** |
| 2026-08-26 downtown | deleted; its figures are records only |

## Delivery checklist

A run is not delivered until all of these are true.

- [ ] All four acceptance checks pass on the **cropped** file, each reported with
      its number rather than as a bare "pass".
- [ ] Export voxel stated alongside the thickness figure.
- [ ] Field completeness confirmed: `gps_time`, `intensity`, `nir` and `range_m`
      present and non-empty. A cloud carrying only X/Y/Z cannot be classified, and
      this was silently the case for `rigid_fit` output before 2026-09-10.
- [ ] CRS written into the LAS header and correct for the site.
- [ ] Trajectory CSV and `_qa.json` retained beside the LAS. They are the only
      record of how the cloud was placed.
- [ ] The processing command retained verbatim, including the backend and both
      voxel sizes.
- [ ] Input-side gap scan recorded, including gaps that were judged survivable.

## Deliverable record

| Field | Value |
|---|---|
| Session | 2026-07-21 170623, San Antonio residential |
| Chunks | 1-6 |
| Route length | 1032.99 m (GPS reference) |
| Duration | 175.8 s |
| Backend | `rigid_fit`, `--anchor-z-mode tilt` |
| SLAM voxel | 0.25 m |
| Export voxel | 0.10 m |
| CRS | EPSG:32614 |
| Points delivered | 31,576,695 |
| Processing time | ~60 min |
