# DataFromSky-Drone-Trajectory-Preprocessing

This script converts raw vehicle trajectory data into Frenet-coordinate features along a fitted reference line. It is designed for semicolon-separated trajectory files where each vehicle trajectory is stored as one long row.

## 🔧 What The Code Does?

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

## 📥 Input Data Format

The input CSV file is expected to be exported from **DataFromSky** and separated by semicolons (`;`).

```text
Track ID; Type; Entry Gate; Entry Time [s]; Exit Gate; Exit Time [s]; Traveled Dist. [px]; Avg. Speed [kpx/h]; Trajectory(...)
```

### 🧾 Vehicle Metadata

The first eight columns contain vehicle-level metadata, including vehicle ID, vehicle type, entry and exit information, traveled distance, and average speed.

| Column                | Description                                    |
| --------------------- | ---------------------------------------------- |
| `Track ID`            | Unique vehicle trajectory ID                   |
| `Type`                | Vehicle type                                   |
| `Entry Gate`          | Entry position or gate                         |
| `Entry Time [s]`      | Time when the vehicle enters the analysis area |
| `Exit Gate`           | Exit position or gate                          |
| `Exit Time [s]`       | Time when the vehicle leaves the analysis area |
| `Traveled Dist. [px]` | Traveled distance in image pixels              |
| `Avg. Speed [kpx/h]`  | Average speed in image-based units             |


### 📍 Trajectory Sequence

The trajectory sequence starts from the **9th column**. Each trajectory frame contains five values:

```text
x [px]; y [px]; speed; acceleration; time [s]
```

Therefore, each vehicle trajectory row is interpreted as:

```text
Track ID; Type; metadata...; x1; y1; v1; a1; t1; x2; y2; v2; a2; t2; ...
```

### 🛣️ Variables Used for Frenet Processing

In the current preprocessing procedure, only the image coordinates and time values are used for Frenet coordinate transformation:

```text
x [px]; y [px]; time [s]
```

The speed and acceleration values exported by DataFromSky are not directly used in the Frenet transformation step.

### ✅ Output Purpose

After preprocessing, the trajectory data are converted into a cleaner and more structured format for highway traffic flow analysis, vehicle interaction analysis, and traffic safety evaluation.
