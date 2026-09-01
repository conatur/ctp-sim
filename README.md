# Cone Track Path Simulator

Recovers a driveable racing line from unordered, noisy cone positions and simulates a vehicle on it

![Animation of car driving along the racing line](figures/lap1.gif)

## Problem
Formula SAE driverless requires the vehicle to be operated autonomously on a course marked by colored cones, blue on one side and yellow on the other. The layout is not known to the car in advance, so it must build a driveable centerline line in real time from cone positions and then track it.

This project recovers the centerline and tracks it on synthetic cone data, with perception assumed. Cone data is unordered, noisy, and some cones are missing to simulate data inaccuracy during real races. 

## Pipeline

`track.py` -> Track generation: builds a closed-loop centerline as a sum of sinusoidal harmonics in polar form, resamples it at a uniform arc length (5.0 m), and places blue and yellow cones 1.75 meters on each side along the normal. The output is then corrupted to mimic imperfect perception: 5 cm Gaussian position noise, 7% cones missed, and shuffled so that cones are unordered. The true centerline is saved for evaluation purposes (never given to the path builder).  

`centerline.py` -> Centerline recovery: Runs Delaunay triangulation over all the cones and extracts the unique edges. We then apply a filter -> only blue-yellow edges are kept (same color edges run along the boundary). Edges over 8.0 m are also rejected (caused by deleted cones) so that the line is not skewed by outliers (see `edgelengths.png` and `midpoints_no_outliers.png` vs `midpoints_w_outliers.png`). After we take the midpoints of these remaining edges and order them with a greedy nearest-neighbor starting from the car's start position. 

`path.py` -> Path fitting + speed profile: We fit a periodic cubic spline through the ordered midpoints with a smoothing constant `s = 1.0`. Then the spline is resampled to 3000 points at a uniform arc length. Curvature is calculated from the spline's first and second derivatives, and it's then used to build a speed profile over the 3000 points with the speed capped at `sqrt(a_lat/|κ|)` at every point. A backward pass applies braking limits and a forward pass on the profile applies the acceleration limits. Each pass is done twice to take care of the seam (end and start of the loop).

`sim.py` -> Vehicle simulation: a kinematic bicycle model with a 1.55 m wheelbase is simulated in 0.02 second steps over the path returned by `path.py`. Steering comes from pure pursuit: the controller aims at a point on the path a lookahead distance away. This distance scales with speed, and the steering angle is clamped to ±25°. The car accelerates or brakes in proportion to how far it is from the speed profile's target, clamped by the acceleration and braking limits used to build the profile. All states are recorded and are used to construct the animation and the cross-track error measurement. 

## Findings

1. **Edge-length filtering removes outliers without affecting median accuracy** 

Blue - yellow edge lengths are bimodal, with straight edges across the track clustered at the track width (3.5 m) and diagonal edges from cone *i* to cone *i* + 1 at √(width² + spacing²) ≈ 6.1 m. However, there are some outliers where a dropped cone forces Delaunay to bridge a large gap which produces midpoints far off the centerline. 

![Measure of edge lengths between blue to yellow cones](figures/edge_lengths.png)

Rejecting edges over 8.0 m removes 0.73% of midpoints across 50 tracks (5,055 edges). Although typical accuracy is mostly unchanged (median per track offset 0.0825 m -> 0.0810 m), the tail was truncated: number of tracks containing an outlier more than 0.5 m off center fell from 11 to 0. Additionally, for the 24 out of 50 tracks the filter fired on, it improves the maximum on 16 of those of which median maximum falls from 0.565m to 0.243m.

2. **Spline smoothing must balance position accuracy and curvature accuracy**

The smoothing parameter has opposite optima for the two quantities the spline must provide: position and curvature. Minumum position error obviously occurs at `s = 0` where the curve interpolates every midpoints exactly. But doing this amplifies the 5 cm cone noise as curvature is a second derivative: the curvature oscillates between ±0.25 m⁻¹ against a true peak of 0.063 m⁻¹, roughly 4 times sharper than the real curvature. At `s = 5`, this noise is gone but a small reversal in curvature at 70 m is smoothed away entirely.

`s = 1` was chosen as the best as curvature stays within 15% of ground truth and position error is within 30% of `s = 0` maxima. 

| s | mean (m) | p95 (m) | max (m) |
|---|----------|---------|---------|
| 0 | 0.083 | 0.214 | 0.376 |
| **1** | **0.112** | **0.261** | **0.322** |
| 5 | 0.220 | 0.520 | 0.646 |
| 20 | 0.442 | 0.898 | 1.018 |

3. **Greedy ordering requires a large search radius**



## Limitations


## Next Steps


## References

