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
The Frenet coordinate transformation is implemented based on the method proposed by Werling et al. in [*Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*](https://www.semanticscholar.org/paper/Optimal-trajectory-generation-for-dynamic-street-in-Werling-Ziegler/6bda8fc13bda8cffb3bb426a73ce5c12cc0a1760)


## ⚙️ Main Function

The main function for trajectory preprocessing is `process_raw_to_frenet_csv()`.

This function reads the raw semicolon-separated trajectory CSV exported from **DataFromSky**, filters and smooths the trajectory data, transforms image coordinates into highway Frenet coordinates, and saves the processed results as a new CSV file.

```python
process_raw_to_frenet_csv(
    input_file,
    output_file,
    max_rows=None,
    y_min=1333,
    y_max=1422,
    time_interval=0.1,
    time_atol=1e-6,
    reference_line=REFERENCE_LINE,
    reference_x_min=REFERENCE_X_MIN,
    reference_x_max=REFERENCE_X_MAX,
    reference_num=REFERENCE_NUM,
    reverse_s=False,
    flip_d=False,
    smooth=True,
    kalman_r=KALMAN_R,
    kalman_q=KALMAN_Q,
)
```

## 📥 Function Inputs

| Parameter         | Description                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------------------ |
| `input_file`      | Path to the raw semicolon-separated trajectory CSV exported from DataFromSky.                    |
| `output_file`     | Path where the processed Frenet-coordinate CSV will be saved.                                    |
| `max_rows`        | Optional row limit for testing. Use `None` to process the full file.                             |
| `y_min`           | Lower bound for filtering vehicles based on the first-frame image `y` coordinate.                |
| `y_max`           | Upper bound for filtering vehicles based on the first-frame image `y` coordinate.                |
| `time_interval`   | Sampling interval in seconds. Use `None` or `<= 0` to keep all frames.                           |
| `time_atol`       | Absolute tolerance used when matching the desired sampling interval.                             |
| `reference_line`  | Polynomial coefficients of the highway reference line, ordered for `numpy.polyval`.              |
| `reference_x_min` | Minimum `x` value used to sample the reference line.                                             |
| `reference_x_max` | Maximum `x` value used to sample the reference line.                                             |
| `reference_num`   | Number of sampled points used to construct the reference line.                                   |
| `reverse_s`       | If `True`, reverses the longitudinal Frenet coordinate direction.                                |
| `flip_d`          | If `True`, flips the sign of the lateral Frenet coordinate.                                      |
| `smooth`          | If `True`, applies Kalman smoothing to selected trajectory variables.                            |
| `kalman_r`        | Measurement noise parameter of the Kalman filter. A larger value produces stronger smoothing.    |
| `kalman_q`        | Process noise parameter of the Kalman filter. A larger value allows faster motion-state changes. |

## 🛣️ Reference Line Hyperparameters

The reference line is used to define the highway Frenet coordinate system. The polynomial coefficients are stored in `REFERENCE_LINE`, while the sampling range and resolution are controlled by the following hyperparameters:

```python
REFERENCE_X_MIN = 0.0
REFERENCE_X_MAX = 4000.0
REFERENCE_NUM = 4000
```

These hyperparameters define the longitudinal sampling range and resolution of the highway reference line. They can be adjusted according to the length and geometry of different highway sections.

## 📉 Kalman Smoothing Hyperparameters

Kalman smoothing is controlled by two hyperparameters:

```python
KALMAN_R = 0.001
KALMAN_Q = 0.05
```

`KALMAN_R` represents the measurement noise, which controls the reliability of the observed trajectory data. A larger value means the filter trusts the raw observations less and produces stronger smoothing.

`KALMAN_Q` represents the process noise, which controls the allowed variation of vehicle motion states. A larger value allows the trajectory state to change more quickly.

## ✅ Output

The function generates a processed CSV file containing standardized highway vehicle trajectory data in Frenet coordinates. The output can be used for traffic flow analysis, vehicle interaction analysis, lane-changing behavior analysis, and traffic safety evaluation.
