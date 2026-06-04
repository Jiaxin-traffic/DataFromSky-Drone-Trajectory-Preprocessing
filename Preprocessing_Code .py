import csv
import math
import re
from typing import List, Any, Optional
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# Vehicle types exported by DataFromSky software
ALLOWED_TYPES = {"car", "medium vehicle", "bus", "heavy vehicle"}

# Example parameter values. Users should modify them according to their own data and processing needs.
TRAJECTORY_START_COL = 8
GROUP_SIZE = 5
PX_PER_METER = 8.2
Y_MIN = 1333
Y_MAX = 1422
TIME_INTERVAL = 0.1
TIME_ATOL = 1e-6
REFERENCE_DEGREE = 4
REFERENCE_COEFFICIENTS = [-1.587e-15, 2.942e-10, -1.048e-5, -0.01445, 1336.061798]
REFERENCE_X_MIN = 0
REFERENCE_X_MAX = 4000
REFERENCE_NUM = 4000
KALMAN_R = 0.001
KALMAN_Q = 0.05
TRIM_EDGE_SECONDS = 0.0

def is_integer_token(s: str) -> bool:
    if s is None:
        return False
    return re.fullmatch(r"\d+", s.strip()) is not None


def normalize_type(s: str) -> str:
    if s is None:
        return ""
    return " ".join(s.split()).lower()


def is_number(s: str) -> bool:
    if s is None:
        return False
    try:
        float(str(s).strip())
        return True
    except ValueError:
        return False


def clean_frames(data: List[Any], frame_len: int = 5) -> List[Any]:
    valid_len = len(data) - len(data) % frame_len
    return data[:valid_len]


def should_keep_track(data: List[Any], y_min: float, y_max: float) -> bool:
    if len(data) < 2:
        return False
    try:
        first_y = float(data[1])
    except ValueError:
        return False
    return y_min < first_y < y_max


def is_sample_time(t: float, time_interval: Optional[float], atol: float = 1e-6) -> bool:
    if time_interval is None or time_interval <= 0:
        return True
    nearest = round(t / time_interval) * time_interval
    return abs(t - nearest) <= atol


def build_reference_line(reference_coefficients: Any, reference_degree: int, x_min: float, x_max: float, num: int):
    x_curve = np.linspace(x_min, x_max, num)
    if not isinstance(reference_coefficients, (list, tuple, np.ndarray)):
        raise TypeError("reference_coefficients must be polynomial coefficients.")
    if len(reference_coefficients) != reference_degree + 1:
        raise ValueError("reference_coefficients length must be reference_degree + 1.")

    y_curve = np.polyval(reference_coefficients, x_curve)

    ref_pts = np.column_stack((x_curve, y_curve))
    segments = np.diff(ref_pts, axis=0)
    seg_lens = np.linalg.norm(segments, axis=1)
    s_array = np.insert(np.cumsum(seg_lens), 0, 0)

    dy = np.gradient(y_curve, x_curve)
    d2y = np.gradient(dy, x_curve)
    curvature = d2y / (1 + dy ** 2) ** 1.5
    dcurv_ds = np.gradient(curvature, s_array)

    return ref_pts, s_array, curvature, dcurv_ds, cKDTree(ref_pts)


def kalman_1d(data, r: float, q: float):
    values = np.asarray(data, dtype=float)
    if len(values) == 0:
        return values

    filtered = np.zeros(len(values))
    state = np.array([[values[0]], [0.0]])
    covariance = np.eye(2) * 1000.0
    transition = np.array([[1.0, 1.0], [0.0, 1.0]])
    measurement = np.array([[1.0, 0.0]])
    process_noise = q * np.eye(2)
    measurement_noise = np.array([[r]])

    for i, z in enumerate(values):
        state = transition @ state
        covariance = transition @ covariance @ transition.T + process_noise

        innovation = np.array([[z]]) - measurement @ state
        innovation_covariance = measurement @ covariance @ measurement.T + measurement_noise
        kalman_gain = covariance @ measurement.T @ np.linalg.inv(innovation_covariance)

        state = state + kalman_gain @ innovation
        covariance = (np.eye(2) - kalman_gain @ measurement) @ covariance
        filtered[i] = state[0, 0]
    return filtered


def apply_kalman_to_selected_cols(group, smooth_cols, r: float, q: float):
    for col in smooth_cols:
        if col in group.columns:
            group[col] = kalman_1d(group[col].values, r=r, q=q)
    return group


def parse_raw_tracks_to_long_rows(
        input_file: str,
        max_rows: Optional[int] = None,
        y_min: float = Y_MIN,
        y_max: float = Y_MAX,
        time_interval: Optional[float] = TIME_INTERVAL,
        time_atol: float = TIME_ATOL):
    out_rows = []
    current_track_data: List[Any] = []
    prev_track_id = None
    prev_type = None

    def flush_track():
        if prev_track_id is None:
            return

        cleaned_data = clean_frames(current_track_data, GROUP_SIZE)
        if not should_keep_track(cleaned_data, y_min, y_max):
            return

        numeric_tokens = [float(tok) for tok in cleaned_data if is_number(tok)]
        for i in range(0, len(numeric_tokens) - GROUP_SIZE + 1, GROUP_SIZE):
            x, y, _, _, t = numeric_tokens[i:i + GROUP_SIZE]
            if is_sample_time(t, time_interval, time_atol):
                out_rows.append([prev_track_id, prev_type, t, x, y])

    with open(input_file, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.reader(f, delimiter=';')

        for line_no, raw_row in enumerate(reader, start=1):
            if max_rows is not None and line_no > max_rows:
                break

            row = [str(c).strip() for c in raw_row if str(c).strip() != ""]
            if not row:
                continue

            col0 = row[0]
            col1 = row[1] if len(row) > 1 else ""

            is_track_start_row = is_integer_token(col0) and normalize_type(col1) in ALLOWED_TYPES
            if is_track_start_row:
                flush_track()
                prev_track_id = col0
                prev_type = " ".join(col1.split())
                current_track_data = row[TRAJECTORY_START_COL:]
            elif prev_track_id is not None:
                current_track_data.extend(row)

        flush_track()

    return out_rows


def add_frenet_columns(
        df: pd.DataFrame,
        reference_coefficients: Any,
        reference_degree: int,
        reference_x_min: float,
        reference_x_max: float,
        reference_num: int,
        reverse_s: bool,
        flip_d: bool,
        smooth: bool,
        kalman_r: float,
        kalman_q: float,
        trim_edge_seconds: float,
        max_speed_px_s: float = 300.0,
        max_abs_s_ddot: float = 80.0,
        trim_frames: int = 4,
        px_per_meter: float = PX_PER_METER):
    if df.empty:
        return df

    df = df.sort_values(["Track ID", "Time [s]"]).reset_index(drop=True)
    if trim_edge_seconds > 0:
        group = df.groupby("Track ID", sort=False)["Time [s]"]
        start_time = group.transform("min")
        end_time = group.transform("max")
        keep_mask = (
            (df["Time [s]"] >= start_time + trim_edge_seconds)
            & (df["Time [s]"] <= end_time - trim_edge_seconds)
        )
        df = df[keep_mask].reset_index(drop=True)
        if df.empty:
            return df

    group = df.groupby("Track ID", sort=False)
    dx = group["x [px]"].diff()
    dy = group["y [px]"].diff()
    dt = group["Time [s]"].diff().replace(0, np.nan)
    df["V (px/s)"] = np.sqrt(dx ** 2 + dy ** 2) / dt
    df["Heading"] = np.arctan2(dy, dx)
    df["Ax (px/s)"] = group["V (px/s)"].diff() / dt
    df = df[df["V (px/s)"].notna()]
    df = df[df["V (px/s)"] <= max_speed_px_s].reset_index(drop=True)
    if df.empty:
        return df

    ref_pts, s_array, curv_ref, dcurv_ds, tree = build_reference_line(
        reference_coefficients=reference_coefficients,
        reference_degree=reference_degree,
        x_min=reference_x_min,
        x_max=reference_x_max,
        num=reference_num,
    )

    results = []
    for _, row in df.iterrows():
        x = row["x [px]"]
        y = row["y [px]"]
        vx = row["V (px/s)"]
        heading = row["Heading"]
        ax = row["Ax (px/s)"]

        _, i = tree.query([x, y])
        i = min(i, len(ref_pts) - 2)
        p0, p1 = ref_pts[i], ref_pts[i + 1]
        vec = p1 - p0
        length = np.linalg.norm(vec)
        tangent = vec / length if length > 0 else np.array([1.0, 0.0])
        normal = np.array([-tangent[1], tangent[0]])

        point_vec = np.array([x, y]) - p0
        d = np.dot(point_vec, normal)
        proj = np.clip(np.dot(point_vec, tangent), 0, length)
        s = s_array[i] + proj

        theta_r = math.atan2(tangent[1], tangent[0])
        d_theta = math.atan2(math.sin(heading - theta_r), math.cos(heading - theta_r))
        kappa = curv_ref[i]
        kappa_p = dcurv_ds[i]
        denom = 1 - kappa * d
        if abs(denom) < 1e-6:
            denom = 1e-6 if denom >= 0 else -1e-6

        s_dot = vx * math.cos(d_theta) / denom
        d_dot = vx * math.sin(d_theta)
        s_ddot = (ax * math.cos(d_theta) - s_dot ** 2 * (kappa + kappa_p * d)) / denom
        d_ddot = ax * math.sin(d_theta) + vx ** 2 * math.cos(d_theta) * (kappa + kappa_p * d)
        dt_ds = denom * math.tan(d_theta)
        results.append([s, s_dot, s_ddot, d, d_dot, d_ddot, d_theta, dt_ds])

    f_cols = ["s", "s_dot", "s_ddot", "d", "d_dot", "d_ddot", "d_theta", "dt_ds"]
    df = pd.concat([df.reset_index(drop=True), pd.DataFrame(results, columns=f_cols)], axis=1)

    dt_series = df.groupby("Track ID")["Time [s]"].diff().replace(0, np.nan)
    df["s_ddot"] = df.groupby("Track ID")["s_dot"].diff() / dt_series
    df["d_ddot"] = df.groupby("Track ID")["d_dot"].diff() / dt_series

    if flip_d:
        df["d"] = -df["d"]
        df["d_dot"] = -df["d_dot"]
        df["d_ddot"] = -df["d_ddot"]

    if reverse_s:
        max_s = df["s"].max()
        df["s"] = max_s - df["s"]
        df["s_dot"] = -df["s_dot"]
        df["s_ddot"] = -df["s_ddot"]

    df = df[df["s_ddot"].abs() < max_abs_s_ddot]
    df = df.drop(columns=["Heading", "Ax (px/s)"])

    if trim_frames > 0:
        group = df.groupby("Track ID", sort=False)
        group_size = group["Track ID"].transform("size")
        frame_index = group.cumcount()
        keep_mask = (
            (group_size > 3 * trim_frames)
            & (frame_index >= trim_frames)
            & (frame_index < group_size - trim_frames)
        )
        df = df[keep_mask].reset_index(drop=True)

    if smooth and not df.empty:
        smooth_cols = ["V (px/s)", "s_dot", "s_ddot", "d_dot", "d_ddot", "d_theta"]
        smoothed_groups = []
        for _, group_df in df.groupby("Track ID", sort=False):
            smoothed_groups.append(
                apply_kalman_to_selected_cols(
                    group_df.copy(),
                    smooth_cols,
                    r=kalman_r,
                    q=kalman_q,
                )
            )
        df = pd.concat(smoothed_groups, ignore_index=True)

    columns_to_divide = [
        "s", "s_dot", "s_ddot", "d", "d_dot", "d_ddot"
    ]
    df[columns_to_divide] = df[columns_to_divide] / px_per_meter
    df = df.round(5)
    cols_to_round = [
        "s", "s_dot", "s_ddot", "d", "d_dot", "d_ddot"
    ]
    df[cols_to_round] = df[cols_to_round].round(3)

    return df


def process_raw_to_frenet_csv(
        input_file: str,
        output_file: str,
        max_rows: Optional[int] = None,
        y_min: float = Y_MIN,
        y_max: float = Y_MAX,
        time_interval: Optional[float] = TIME_INTERVAL,
        time_atol: float = TIME_ATOL,
        reference_degree: int = REFERENCE_DEGREE,
        reference_coefficients: Any = REFERENCE_COEFFICIENTS,
        reference_x_min: float = REFERENCE_X_MIN,
        reference_x_max: float = REFERENCE_X_MAX,
        reference_num: int = REFERENCE_NUM,
        reverse_s: bool = False,
        flip_d: bool = False,
        smooth: bool = True,
        kalman_r: float = KALMAN_R,
        kalman_q: float = KALMAN_Q,
        trim_edge_seconds: float = TRIM_EDGE_SECONDS):
    print(f"Reading file and computing Frenet coordinates: {input_file}")
    rows = parse_raw_tracks_to_long_rows(
        input_file=input_file,
        max_rows=max_rows,
        y_min=y_min,
        y_max=y_max,
        time_interval=time_interval,
        time_atol=time_atol,
    )
    if not rows:
        print("No valid trajectory data was detected.")
        return

    df = pd.DataFrame(rows, columns=[
        "Track ID", "Type", "Time [s]", "x [px]", "y [px]"
    ])
    df = add_frenet_columns(
        df,
        reference_coefficients=reference_coefficients,
        reference_degree=reference_degree,
        reference_x_min=reference_x_min,
        reference_x_max=reference_x_max,
        reference_num=reference_num,
        reverse_s=reverse_s,
        flip_d=flip_d,
        smooth=smooth,
        kalman_r=kalman_r,
        kalman_q=kalman_q,
        trim_edge_seconds=trim_edge_seconds,
    )
    output_columns = [
        "Track ID", "Type", "Time [s]",
        "s", "s_dot", "s_ddot",
        "d", "d_dot", "d_ddot",
        "d_theta", "dt_ds",
    ]
    df = df[output_columns]
    df.to_csv(output_file, index=False, encoding="utf-8")
    print(f"Frenet processing complete. Wrote {len(df)} rows to: {output_file}")


if __name__ == "__main__":


    process_raw_to_frenet_csv(
        "input_file.csv",
        "Output_file.csv",
        time_interval=TIME_INTERVAL,
        y_min=Y_MIN,
        y_max=Y_MAX,
        reference_degree=REFERENCE_DEGREE,
        reference_coefficients=REFERENCE_COEFFICIENTS,
        reference_x_min=REFERENCE_X_MIN,
        reference_x_max=REFERENCE_X_MAX,
        reference_num=REFERENCE_NUM,
        smooth=True,
        kalman_r=KALMAN_R,
        kalman_q=KALMAN_Q,
        trim_edge_seconds=TRIM_EDGE_SECONDS,
    )
