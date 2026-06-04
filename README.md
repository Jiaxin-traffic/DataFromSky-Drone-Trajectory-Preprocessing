# DataFromSky-Drone-Trajectory-Preprocessing

This script converts raw vehicle trajectory data into Frenet-coordinate features along a fitted reference line. It is designed for semicolon-separated trajectory files where each vehicle trajectory is stored as one long row.

## What The Code Does

The pipeline performs the following steps:

1. Reads raw trajectory data from a semicolon-separated CSV file.
2. Detects valid vehicle trajectories by `Track ID` and `Type`.
3. Skips metadata columns before the trajectory sequence.
4. Keeps only vehicles whose first-frame `y` coordinate falls within a specified range.
5. Resamples trajectory frames by a configurable time interval.
6. Builds a polynomial reference line.
7. Projects vehicle positions onto the reference line and computes Frenet variables.
8. Optionally applies one-dimensional Kalman smoothing.
9. Converts pixel-based values to metric units.
10. Exports selected Frenet features to CSV.
