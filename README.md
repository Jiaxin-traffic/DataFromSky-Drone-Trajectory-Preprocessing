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
    y_min=Y_MIN,
    y_max=Y_MAX,
    time_interval=TIME_INTERVAL,
    time_atol=1e-6,
    reference_degree=REFERENCE_DEGREE,
    reference_coefficients=REFERENCE_COEFFICIENTS,
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
| `reference_degree` | Degree of the polynomial reference line.                                                        |
| `reference_coefficients` | Polynomial coefficients of the reference line, ordered for `numpy.polyval`. The length must be `reference_degree + 1`. |
| `reference_x_min` | Starting `x` coordinate of the reference line segment in the image coordinate system.            |
| `reference_x_max` | Ending `x` coordinate of the reference line segment in the image coordinate system.              |
| `reference_num`   | Number of points sampled along the reference line segment.                                       |
| `reverse_s`       | If `True`, reverses the longitudinal Frenet coordinate direction.                                |
| `flip_d`          | If `True`, flips the sign of the lateral Frenet coordinate.                                      |
| `smooth`          | If `True`, applies Kalman smoothing to selected trajectory variables.                            |
| `kalman_r`        | Measurement noise parameter of the Kalman filter. A larger value produces stronger smoothing.    |
| `kalman_q`        | Process noise parameter of the Kalman filter. A larger value allows faster motion-state changes. |

## 🛣️ Reference Line Hyperparameters

The reference line is used to define the highway Frenet coordinate system. The polynomial coefficients are stored in `REFERENCE_LINE`, while the sampling range and resolution are controlled by the following hyperparameters:

```python
REFERENCE_X_MIN = X_MIN
REFERENCE_X_MAX = X_MAX
REFERENCE_NUM = NUM_POINTS
```

These hyperparameters define the longitudinal sampling range and resolution of the highway reference line. They can be adjusted according to the length and geometry of different highway sections.

## 📉 Kalman Smoothing Hyperparameters

Kalman smoothing is controlled by two hyperparameters:

```python
KALMAN_R = KALMAN_R
KALMAN_Q = KALMAN_Q
```

`KALMAN_R` represents the measurement noise, which controls the reliability of the observed trajectory data. A larger value means the filter trusts the raw observations less and produces stronger smoothing.
`KALMAN_Q` represents the process noise, which controls the allowed variation of vehicle motion states. A larger value allows the trajectory state to change more quickly.

## 🛣️ Reference Line

The reference line is used to define the Frenet coordinate system for the highway section. In this program, the road boundary or lane reference line is represented by a polynomial:

```python
y = np.polyval(reference_line, x)
```

The polynomial coefficients are stored in `REFERENCE_LINE` and can be replaced for different work zones or road sections. The reference line can be fitted using a polynomial of any order. Higher-order polynomials can better describe curved or irregular road boundaries, while lower-order polynomials are suitable for straight or slightly curved highway sections.

The coefficients should follow the input format of `numpy.polyval`, ordered from the highest-order term to the constant term:

```python
REFERENCE_LINE = [
    coefficient_n,
    coefficient_n_minus_1,
    ...,
    coefficient_1,
    coefficient_0,
]
```

After defining the polynomial reference line, the program samples points along the selected reference line segment and uses them to transform vehicle positions from image coordinates into Frenet coordinates.

## 📤 Output Columns

The output CSV file contains the processed highway vehicle trajectory data in Frenet coordinates. Each row represents one vehicle at one timestamp.

| Output Column | Description |
| ------------- | ----------- |
| `Track ID` | Unique vehicle trajectory ID. |
| `Type` | Vehicle type exported from DataFromSky. |
| `Time [s]` | Timestamp of the trajectory point, in seconds. |
| `s` | Longitudinal Frenet position along the reference line, converted from pixels to meters. |
| `s_dot` | Longitudinal velocity in the Frenet coordinate system, in meters per second. |
| `s_ddot` | Longitudinal acceleration in the Frenet coordinate system, in meters per second squared. |
| `d` | Lateral Frenet offset from the reference line, converted from pixels to meters. |
| `d_dot` | Lateral velocity in the Frenet coordinate system, in meters per second. |
| `d_ddot` | Lateral acceleration in the Frenet coordinate system, in meters per second squared. |
| `d_theta` | Heading angle difference between the vehicle movement direction and the tangent direction of the reference line, in radians. |
| `dt_ds` | Approximate lateral slope with respect to the longitudinal Frenet distance. |

### 📏 Unit Correction

The Frenet position, velocity, and acceleration variables are converted from image-based pixel units into real-world metric units. The trajectory data exported from DataFromSky are initially represented in image-based pixel units. To convert them into real-world metric units, a pixel-to-meter scale factor is calculated using known highway geometric dimensions. In the software, two lane marking lines are selected as calibration reference. The pixel distance between these two lines is measured from the image. Since the actual lane width can be determined according to highway design specifications, the real-world distance between the selected lane lines can be calculated.

## 📄 Example Output

The processed output CSV contains one row for each vehicle at each timestamp.  
An example output is shown below:

| Track ID | Type | Time [s] | s | s_dot | s_ddot | d | d_dot | d_ddot | d_theta | dt_ds |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | Car | 0.6 | 74.710 | 26.159 | 0.487 | 1.994 | 0.035 | 0.122 | 0.00132 | 0.00132 |
| 10 | Car | 0.7 | 77.354 | 26.208 | 0.491 | 1.997 | 0.035 | 0.006 | 0.00134 | 0.00134 |
| 10 | Car | 0.8 | 79.965 | 26.342 | 1.338 | 2.000 | 0.038 | 0.024 | 0.00142 | 0.00143 |

In this example, the same `Track ID` appears at multiple timestamps, representing the continuous trajectory of the same vehicle in the Frenet coordinate system.
