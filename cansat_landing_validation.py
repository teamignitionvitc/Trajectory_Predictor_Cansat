#!/usr/bin/env python3
"""
============================================================================
 IGNITIA CanSat 2026 — Experiment 2: Post-Flight Landing Prediction Validation
 Team Ignition

 cansat_landing_validation.py
 Standalone post-flight analysis script that evaluates the accuracy of
 the onboard real-time landing zone prediction against actual landing GPS.

 Usage:
   python cansat_landing_validation.py --csv log.csv
   python cansat_landing_validation.py --csv log.csv --actual-lat 12.9716 --actual-lon 80.2209
   python cansat_landing_validation.py --simulate

 Dependencies: numpy, scipy, matplotlib, pandas, argparse
============================================================================
"""

import argparse
import math
import os
import sys

# ─── Fix Windows console encoding for Unicode output (emojis, symbols) ───────
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

import matplotlib
matplotlib.use("Agg")  # non-interactive backend for headless rendering

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Circle
import numpy as np
import pandas as pd
import re

# ─── Global style constants ──────────────────────────────────────────────────
COLOR_TRACK       = "#1a5fa8"
COLOR_PREDICTED   = "#b5321a"
COLOR_LANDING     = "#1e7e45"
COLOR_UNCERTAINTY = "#c87e14"

FONT_FAMILY       = "DejaVu Serif"
DPI               = 300
GRID_ALPHA        = 0.28
OUTPUT_DIR        = "./cansat_landing_results"


# ─── Haversine distance (metres) ─────────────────────────────────────────────
def haversine(lat1, lon1, lat2, lon2):
    """
    Compute the great-circle distance in metres between two points
    on Earth given their latitude/longitude in decimal degrees.
    """
    R = 6371000.0  # Earth radius in metres
    phi1    = math.radians(lat1)
    phi2    = math.radians(lat2)
    dphi    = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def haversine_vec(lat1, lon1, lat2, lon2):
    """Vectorised haversine for numpy arrays."""
    R = 6371000.0
    phi1    = np.radians(lat1)
    phi2    = np.radians(lat2)
    dphi    = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)

    a = (np.sin(dphi / 2.0) ** 2 +
         np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2.0) ** 2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return R * c


# ─── Smart column mapper ─────────────────────────────────────────────────────
def auto_map_columns(df):
    """
    Automatically detect and map CSV columns to required fields based on
    column name patterns. Returns a dict with standardized column names.
    """
    columns = df.columns.tolist()
    col_lower = [c.lower() for c in columns]
    
    mapping = {}
    
    # Define search patterns for each required field
    patterns = {
        'altitude_m': [r'.*baro.*alt.*', r'.*alt.*baro.*', r'.*altitude.*', r'.*alt_m.*'],
        'gps_lat': [r'.*lat.*', r'.*latitude.*'],
        'gps_lon': [r'.*lon.*', r'.*longitude.*'],
        'gps_alt_m': [r'.*gps.*alt.*', r'.*alt.*gps.*'],
        'fix_type': [r'.*fix.*type.*', r'.*fix.*qual.*', r'.*gps.*fix.*', r'.*quality.*'],
        'hdop': [r'.*hdop.*', r'.*dop.*'],
        'gps_satellites': [r'.*sat.*', r'.*satellite.*'],
        'pred_lat': [r'.*pred.*lat.*', r'.*predict.*lat.*'],
        'pred_lon': [r'.*pred.*lon.*', r'.*predict.*lon.*'],
        'uncertainty_m': [r'.*uncertain.*', r'.*error.*radius.*', r'.*unc_m.*'],
        't_to_land_s': [r'.*time.*land.*', r'.*ttl.*', r'.*t_to_land.*'],
        'gps_fallback': [r'.*fallback.*', r'.*gps.*fb.*'],
        'temperature': [r'.*temp.*'],
        'pressure': [r'.*press.*'],
        'accel_x': [r'.*accel.*x.*', r'.*acc_x.*', r'.*ax.*'],
        'accel_y': [r'.*accel.*y.*', r'.*acc_y.*', r'.*ay.*'],
        'accel_z': [r'.*accel.*z.*', r'.*acc_z.*', r'.*az.*'],
        'gyro_x': [r'.*gyro.*x.*', r'.*gyr_x.*', r'.*gx.*'],
        'gyro_y': [r'.*gyro.*y.*', r'.*gyr_y.*', r'.*gy.*'],
        'gyro_z': [r'.*gyro.*z.*', r'.*gyr_z.*', r'.*gz.*'],
        'velocity': [r'.*velocity.*', r'.*vel.*'],
    }
    
    # Try to match each field
    for field, pattern_list in patterns.items():
        matched = False
        for pattern in pattern_list:
            for i, col in enumerate(col_lower):
                if re.match(pattern, col):
                    mapping[field] = columns[i]
                    matched = True
                    break
            if matched:
                break
    
    return mapping


def validate_and_prepare_dataframe(df, column_mapping):
    """
    Validate that required columns exist and prepare a standardized DataFrame.
    Returns a new DataFrame with standardized column names.
    """
    required_fields = ['altitude_m', 'gps_lat', 'gps_lon']
    optional_fields = ['gps_alt_m', 'fix_type', 'hdop', 'gps_satellites', 
                      'uncertainty_m', 't_to_land_s', 'gps_fallback',
                      'temperature', 'pressure', 'velocity', 'pred_lat', 'pred_lon']
    
    # Check required fields
    missing_required = [f for f in required_fields if f not in column_mapping]
    if missing_required:
        print(f"\n  ✗ ERROR: Could not auto-detect required columns: {missing_required}")
        print(f"\n  Available columns: {list(df.columns)}")
        print(f"\n  Detected mappings: {column_mapping}")
        print(f"\n  💡 TIP: Required columns must contain these keywords:")
        print(f"     - altitude: 'altitude', 'baro', 'alt_m'")
        print(f"     - latitude: 'lat', 'latitude'")
        print(f"     - longitude: 'lon', 'longitude'")
        raise ValueError(f"Missing required columns: {missing_required}")
    
    # Report optional fields that were not found
    missing_optional = [f for f in optional_fields if f not in column_mapping]
    if missing_optional:
        print(f"\n  ⚠ Optional columns not found (will calculate/use defaults): {', '.join(missing_optional)}")
    
    # Create new DataFrame with standardized names
    df_std = pd.DataFrame()
    
    for std_name, orig_name in column_mapping.items():
        df_std[std_name] = df[orig_name]
    
    # Add default values for missing optional fields
    if 'fix_type' not in df_std.columns:
        df_std['fix_type'] = 3  # Assume good fix
    if 'hdop' not in df_std.columns:
        df_std['hdop'] = 1.0  # Assume good HDOP
    if 'gps_satellites' not in df_std.columns:
        df_std['gps_satellites'] = 8  # Assume good satellite count
    if 'gps_fallback' not in df_std.columns:
        df_std['gps_fallback'] = 0
    if 'gps_alt_m' not in df_std.columns:
        df_std['gps_alt_m'] = df_std['altitude_m']
    
    # Calculate vertical velocity if not present
    if 'velocity' not in df_std.columns:
        # Compute vertical velocity from altitude differences
        df_std['velocity'] = df_std['altitude_m'].diff().fillna(0.0)
    
    # Calculate Time to Land iteratively with physical constraints
    print(f"\n  🧮 Calculating Time to Land at each timestamp...")
    df_std['t_to_land_s'] = calculate_time_to_land(df_std['altitude_m'].values, 
                                                     df_std['velocity'].values)
    
    # Calculate dynamic predicted landing coordinates
    print(f"\n  🎯 Calculating dynamic landing predictions...")
    pred_lat, pred_lon, uncertainty = calculate_dynamic_landing_prediction(
        df_std['gps_lat'].values,
        df_std['gps_lon'].values,
        df_std['t_to_land_s'].values,
        df_std['altitude_m'].values
    )
    
    df_std['pred_lat'] = pred_lat
    df_std['pred_lon'] = pred_lon
    df_std['uncertainty_m'] = uncertainty
    
    return df_std


def calculate_dynamic_landing_prediction(latitudes, longitudes, ttl_values, altitudes):
    """
    Calculate predicted landing coordinates dynamically at each timestamp.
    
    Uses iterative horizontal velocity calculation:
    1. Calculate horizontal drift velocity from GPS position changes
    2. Project landing point: pred = current_pos + (drift_velocity * time_to_land)
    3. As TTL → 0, prediction converges to actual GPS position
    
    Args:
        latitudes: array of GPS latitudes (degrees)
        longitudes: array of GPS longitudes (degrees)
        ttl_values: array of time to land values (seconds)
        altitudes: array of altitude values (meters)
    
    Returns:
        tuple: (pred_lat, pred_lon, uncertainty) arrays
    """
    n = len(latitudes)
    pred_lat = np.zeros(n)
    pred_lon = np.zeros(n)
    uncertainty = np.zeros(n)
    
    # Constants
    DEG_TO_M = 111320.0  # meters per degree latitude
    
    # Initialize first point (no previous data)
    pred_lat[0] = latitudes[0]
    pred_lon[0] = longitudes[0]
    uncertainty[0] = min(200.0, max(5.0, altitudes[0] * 0.1))
    
    # Smoothed velocity estimates (exponential moving average)
    v_north_smooth = 0.0
    v_east_smooth = 0.0
    alpha = 0.3  # smoothing factor
    
    print(f"\n  📊 Dynamic Prediction Reasoning Trace:")
    print(f"  " + "=" * 70)
    
    for i in range(1, n):
        dt = 1.0  # assume 1 second between samples
        
        # Calculate instantaneous horizontal velocities (m/s)
        dlat = latitudes[i] - latitudes[i-1]
        dlon = longitudes[i] - longitudes[i-1]
        
        cos_lat = np.cos(np.radians(latitudes[i]))
        if abs(cos_lat) < 0.001:
            cos_lat = 0.001
        
        v_north_inst = dlat * DEG_TO_M / dt
        v_east_inst = dlon * DEG_TO_M * cos_lat / dt
        
        # Apply exponential smoothing to reduce noise
        v_north_smooth = alpha * v_north_inst + (1 - alpha) * v_north_smooth
        v_east_smooth = alpha * v_east_inst + (1 - alpha) * v_east_smooth
        
        # Get current time to land
        ttl = ttl_values[i]
        
        # Calculate predicted landing coordinates
        if np.isnan(ttl) or ttl <= 0:
            # If not descending or already landed, prediction = current position
            pred_lat[i] = latitudes[i]
            pred_lon[i] = longitudes[i]
        else:
            # Project landing point using current drift and time to land
            # pred = current_pos + (drift_velocity * time_to_land)
            pred_lat[i] = latitudes[i] + (v_north_smooth * ttl) / DEG_TO_M
            pred_lon[i] = longitudes[i] + (v_east_smooth * ttl) / (DEG_TO_M * cos_lat)
        
        # Calculate uncertainty (decreases as altitude decreases)
        uncertainty[i] = min(200.0, max(5.0, altitudes[i] * 0.1 + abs(v_north_smooth + v_east_smooth) * 0.5))
        
        # Print trace for key points
        if i == 1 or i == n // 2 or i >= n - 3:
            print(f"\n  Index {i}:")
            print(f"    Current GPS:     ({latitudes[i]:.7f}°, {longitudes[i]:.7f}°)")
            print(f"    Altitude:        {altitudes[i]:.2f} m")
            print(f"    Drift velocity:  N={v_north_smooth:.3f} m/s, E={v_east_smooth:.3f} m/s")
            print(f"    Time to land:    {ttl:.2f} s" if not np.isnan(ttl) else "    Time to land:    N/A")
            if not np.isnan(ttl) and ttl > 0:
                print(f"    Projection:      ({latitudes[i]:.7f}, {longitudes[i]:.7f}) + ")
                print(f"                     ({v_north_smooth:.3f}, {v_east_smooth:.3f}) * {ttl:.2f}s")
            print(f"    Predicted:       ({pred_lat[i]:.7f}°, {pred_lon[i]:.7f}°)")
            print(f"    Uncertainty:     {uncertainty[i]:.1f} m")
    
    print(f"  " + "=" * 70)
    
    return pred_lat, pred_lon, uncertainty


def calculate_time_to_land(altitudes, velocities):
    """
    Calculate Time to Land at each timestamp using vectorized operations.
    
    Physical constraints:
    - If altitude = 0, TTL = 0
    - If velocity >= 0 (climbing/hovering), TTL = NaN
    - If velocity < 0 (descending), TTL = altitude / |velocity|
    
    Args:
        altitudes: numpy array of altitude values (m)
        velocities: numpy array of vertical velocity values (m/s, negative = descending)
    
    Returns:
        numpy array of time to land values (seconds)
    """
    # Convert to numpy arrays
    alt = np.array(altitudes, dtype=float)
    vel = np.array(velocities, dtype=float)
    
    # Initialize TTL array with NaN
    ttl = np.full_like(alt, np.nan, dtype=float)
    
    # Apply constraints using vectorized operations
    # Constraint 1: If altitude is 0, TTL = 0
    ttl[alt <= 0.0] = 0.0
    
    # Constraint 2: If descending (velocity < 0) and altitude > 0, calculate TTL
    descending_mask = (vel < 0.0) & (alt > 0.0)
    ttl[descending_mask] = alt[descending_mask] / np.abs(vel[descending_mask])
    
    # Constraint 3: If climbing or hovering (velocity >= 0) and altitude > 0, TTL = NaN
    # (already initialized to NaN, so no action needed)
    
    # Print reasoning trace for key points
    print_reasoning_trace(alt, vel, ttl)
    
    return ttl


def print_reasoning_trace(altitudes, velocities, ttl_values):
    """
    Print reasoning trace for start, midpoint, and near-landing points.
    """
    n = len(altitudes)
    if n == 0:
        return
    
    # Select indices: start, midpoint, near-landing
    start_idx = 0
    mid_idx = n // 2
    
    # Find near-landing: last point with altitude > 5m
    near_landing_idx = n - 1
    for i in range(n - 1, -1, -1):
        if altitudes[i] > 5.0:
            near_landing_idx = i
            break
    
    print(f"\n  📊 Reasoning Trace for Time to Land Calculation:")
    print(f"  " + "=" * 70)
    
    for label, idx in [("START", start_idx), ("MIDPOINT", mid_idx), ("NEAR LANDING", near_landing_idx)]:
        alt = altitudes[idx]
        vel = velocities[idx]
        ttl = ttl_values[idx]
        
        print(f"\n  {label} (index {idx}):")
        print(f"    Altitude:  {alt:8.2f} m")
        print(f"    Velocity:  {vel:8.2f} m/s")
        
        if alt <= 0.0:
            print(f"    Math:      altitude = 0 → TTL = 0")
            print(f"    TTL:       {ttl:8.2f} s")
        elif vel >= 0.0:
            print(f"    Math:      velocity ≥ 0 (not descending) → TTL = N/A")
            print(f"    TTL:       {'NaN':>8s}")
        else:
            print(f"    Math:      TTL = altitude / |velocity|")
            print(f"               TTL = {alt:.2f} / {abs(vel):.2f}")
            print(f"               TTL = {ttl:.2f} s")
            print(f"    TTL:       {ttl:8.2f} s")
    
    print(f"  " + "=" * 70)


# ─── Style helper ────────────────────────────────────────────────────────────
def style_axes(ax, xlabel="", ylabel="", title=""):
    """Apply the IGNITIA house style to an axes object."""
    ax.set_xlabel(xlabel, fontfamily=FONT_FAMILY, fontsize=10)
    ax.set_ylabel(ylabel, fontfamily=FONT_FAMILY, fontsize=10)
    if title:
        ax.set_title(title, fontfamily=FONT_FAMILY, fontsize=12, fontweight="bold")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, linestyle="--", alpha=GRID_ALPHA)
    ax.tick_params(labelsize=8)
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontfamily(FONT_FAMILY)


# ─── Simulation: generate synthetic descent data ─────────────────────────────
def generate_simulated_data():
    """
    Simulate a 350 m descent with constant horizontal drift.
    v_north = 1.2 m/s, v_east = 0.8 m/s, GPS valid throughout.
    Runs the full onboard prediction pipeline in Python.
    """
    dt = 1.0
    alt_start = 350.0
    v_descent = -3.5       # m/s (negative = descending)
    v_north_true = 1.2     # m/s northward drift
    v_east_true  = 0.8     # m/s eastward drift

    launch_lat = 12.971600
    launch_lon = 80.220900

    n_steps = int(abs(alt_start / v_descent)) + 5  # a few extra after landing

    rows = []
    lat = launch_lat
    lon = launch_lon
    alt = alt_start

    # ── Onboard filter state (mirrors C implementation) ──
    alt_prev       = alt
    vz_filt        = 0.0
    lat_prev       = lat
    lon_prev       = lon
    v_north_filt   = 0.0
    v_east_filt    = 0.0
    last_valid_lat = lat
    last_valid_lon = lon
    first_call     = True

    for i in range(n_steps):
        # True physics
        if alt > 0:
            alt += v_descent * dt
            lat += v_north_true * dt / 111320.0
            lon += v_east_true * dt / (111320.0 * math.cos(math.radians(lat)))
        alt = max(alt, 0.0)

        # Sensor noise
        noise_lat = np.random.normal(0, 0.000002)
        noise_lon = np.random.normal(0, 0.000002)
        noise_alt = np.random.normal(0, 0.3)

        s_alt  = alt + noise_alt
        s_lat  = lat + noise_lat
        s_lon  = lon + noise_lon
        s_fix  = 3
        s_hdop = 1.2

        # ── Onboard prediction algorithm (Python replica) ──
        if first_call:
            alt_prev       = s_alt
            vz_filt        = 0.0
            lat_prev       = s_lat
            lon_prev       = s_lon
            v_north_filt   = 0.0
            v_east_filt    = 0.0
            last_valid_lat = s_lat
            last_valid_lon = s_lon
            first_call     = False

            pred_lat       = s_lat
            pred_lon       = s_lon
            uncertainty_m  = 200.0
            t_to_land      = 120.0
            gps_fallback   = 0
        else:
            # Step 1: vertical velocity
            vz_raw  = (s_alt - alt_prev) / dt
            vz_filt = 0.3 * vz_raw + 0.7 * vz_filt
            alt_prev = s_alt

            if abs(vz_filt) < 0.05:
                t_to_land = 120.0
            else:
                t_to_land = s_alt / abs(vz_filt)
                t_to_land = max(1.0, min(120.0, t_to_land))

            # Step 2: horizontal drift
            gps_valid = (s_fix >= 2) and (s_hdop <= 5.0)
            if gps_valid:
                vn = (s_lat - lat_prev) * 111320.0 / dt
                ve = (s_lon - lon_prev) * 111320.0 * math.cos(math.radians(s_lat)) / dt
                v_north_filt = 0.3 * vn + 0.7 * v_north_filt
                v_east_filt  = 0.3 * ve + 0.7 * v_east_filt
                lat_prev       = s_lat
                lon_prev       = s_lon
                last_valid_lat = s_lat
                last_valid_lon = s_lon
                gps_fallback   = 0
            else:
                gps_fallback = 1

            # Step 3: project
            origin_lat = last_valid_lat if gps_fallback else s_lat
            origin_lon = last_valid_lon if gps_fallback else s_lon
            cos_lat = math.cos(math.radians(origin_lat))
            if abs(cos_lat) < 0.001:
                cos_lat = 0.001

            pred_lat = origin_lat + (v_north_filt * t_to_land) / 111320.0
            pred_lon = origin_lon + (v_east_filt  * t_to_land) / (111320.0 * cos_lat)

            # Step 4: uncertainty
            uncertainty_m = s_alt * 0.1 + abs(vz_filt) * 2.0
            uncertainty_m = max(5.0, min(200.0, uncertainty_m))

        rows.append({
            "altitude_m":     round(s_alt, 2),
            "gps_lat":        round(s_lat, 7),
            "gps_lon":        round(s_lon, 7),
            "gps_alt_m":      round(s_alt, 2),
            "fix_type":       s_fix,
            "hdop":           s_hdop,
            "pred_lat":       round(pred_lat, 7),
            "pred_lon":       round(pred_lon, 7),
            "uncertainty_m":  round(uncertainty_m, 1),
            "t_to_land_s":    round(t_to_land, 1),
            "gps_fallback":   gps_fallback,
        })

        if alt <= 0:
            break

    df = pd.DataFrame(rows)
    return df


# ─── Figure 1: Prediction Track Map ─────────────────────────────────────────
def plot_prediction_track(df, actual_lat, actual_lon, launch_lat, launch_lon):
    fig, ax = plt.subplots(figsize=(8, 7))

    # Actual GPS track (blue line)
    ax.plot(df["gps_lon"], df["gps_lat"], color=COLOR_TRACK,
            linewidth=1.8, label="Actual GPS track", zorder=3)

    # Predicted landing dots — alpha fades with altitude
    alt_max = df["altitude_m"].max()
    alt_min = df["altitude_m"].min()
    alt_range = max(alt_max - alt_min, 1.0)
    alphas = 0.15 + 0.85 * (1.0 - (df["altitude_m"] - alt_min) / alt_range)

    for _, row in df.iterrows():
        a = 0.15 + 0.85 * (1.0 - (row["altitude_m"] - alt_min) / alt_range)
        ax.scatter(row["pred_lon"], row["pred_lat"], color=COLOR_PREDICTED,
                   s=18, alpha=a, zorder=4, edgecolors="none")
    # Dummy for legend
    ax.scatter([], [], color=COLOR_PREDICTED, s=18, label="Predicted landing", zorder=4)

    # Actual landing (green star)
    ax.scatter(actual_lon, actual_lat, color=COLOR_LANDING, s=200,
               marker="*", zorder=6, label="Actual landing", edgecolors="k", linewidths=0.5)

    # Launch site (green star outline)
    if launch_lat is not None and launch_lon is not None:
        ax.scatter(launch_lon, launch_lat, color=COLOR_LANDING, s=160,
                   marker="^", zorder=6, label="Launch site", edgecolors="k", linewidths=0.5)

    # Uncertainty circles at selected altitudes
    for target_alt in [300, 200, 100, 50]:
        closest_idx = (df["altitude_m"] - target_alt).abs().idxmin()
        row = df.loc[closest_idx]
        if abs(row["altitude_m"] - target_alt) < 30:
            r_deg = row["uncertainty_m"] / 111320.0
            circle = Circle((row["pred_lon"], row["pred_lat"]), r_deg,
                            fill=True, facecolor=COLOR_UNCERTAINTY, alpha=0.2,
                            edgecolor=COLOR_UNCERTAINTY, linewidth=1.0,
                            linestyle="--", zorder=2)
            ax.add_patch(circle)
            ax.annotate(f'{int(row["altitude_m"])}m', (row["pred_lon"], row["pred_lat"]),
                        fontsize=7, fontfamily=FONT_FAMILY, color=COLOR_UNCERTAINTY,
                        ha="center", va="bottom", zorder=7)

    ax.set_aspect("equal")
    style_axes(ax, xlabel="Longitude (°)", ylabel="Latitude (°)",
               title="Landing Prediction Track")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9,
              prop={"family": FONT_FAMILY})

    # Auto-zoom with padding
    all_lons = np.concatenate([df["gps_lon"].values, df["pred_lon"].values, [actual_lon]])
    all_lats = np.concatenate([df["gps_lat"].values, df["pred_lat"].values, [actual_lat]])
    pad = max((all_lons.max() - all_lons.min()), (all_lats.max() - all_lats.min())) * 0.15
    pad = max(pad, 0.0005)
    ax.set_xlim(all_lons.min() - pad, all_lons.max() + pad)
    ax.set_ylim(all_lats.min() - pad, all_lats.max() + pad)

    fig.tight_layout()
    path = os.path.join(OUTPUT_DIR, "fig1_prediction_track.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ Saved {path}")


# ─── Figure 2: Error vs Altitude ────────────────────────────────────────────
def plot_error_vs_altitude(df, errors):
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(df["altitude_m"], errors, color=COLOR_PREDICTED, linewidth=1.5,
            label="Prediction error", zorder=3)

    # Shaded uncertainty band
    ax.fill_between(df["altitude_m"], 0, df["uncertainty_m"],
                    color=COLOR_UNCERTAINTY, alpha=0.2, label="Uncertainty band",
                    zorder=2)

    # Dashed line at final error
    final_err = errors.iloc[-1]
    ax.axhline(final_err, color=COLOR_LANDING, linestyle="--", linewidth=1.2,
               label=f"Final error: {final_err:.1f} m", zorder=4)

    # Invert x-axis: high altitude left → low altitude right
    ax.invert_xaxis()

    style_axes(ax, xlabel="Altitude (m)", ylabel="Prediction Error (m)",
               title="Prediction Error vs Altitude")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9,
              prop={"family": FONT_FAMILY})

    fig.tight_layout()
    path = os.path.join(OUTPUT_DIR, "fig2_error_vs_altitude.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ Saved {path}")


# ─── Figure 3: Time-to-Land vs Altitude ─────────────────────────────────────
def plot_ttl_vs_altitude(df):
    fig, ax = plt.subplots(figsize=(8, 5))

    ttl = df["t_to_land_s"].values
    alt = df["altitude_m"].values

    # Filter out NaN values for plotting
    valid_mask = ~np.isnan(ttl)
    ttl_valid = ttl[valid_mask]
    alt_valid = alt[valid_mask]
    
    if len(ttl_valid) == 0:
        print("  ⚠ Warning: No valid TTL values to plot")
        plt.close(fig)
        return

    # Detect non-monotonic segments (TTL should decrease as altitude drops)
    mono_mask = np.ones(len(ttl_valid), dtype=bool)
    for i in range(1, len(ttl_valid)):
        if ttl_valid[i] > ttl_valid[i - 1] + 0.5:  # tolerance 0.5 s
            mono_mask[i] = False

    # Plot valid TTL values
    ax.plot(alt_valid, ttl_valid, color=COLOR_TRACK, linewidth=1.5, 
            label="Time to land", zorder=3)

    # Highlight non-monotonic points
    non_mono_idx = np.where(~mono_mask)[0]
    if len(non_mono_idx) > 0:
        ax.scatter(alt_valid[non_mono_idx], ttl_valid[non_mono_idx], 
                   color="red", s=25, zorder=5, 
                   label=f"Non-monotonic ({len(non_mono_idx)} pts)",
                   edgecolors="darkred", linewidths=0.5)
    
    # Show NaN regions if any
    nan_count = np.sum(~valid_mask)
    if nan_count > 0:
        ax.text(0.02, 0.98, f"⚠ {nan_count} points with N/A TTL\n(climbing/hovering)", 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax.invert_xaxis()

    style_axes(ax, xlabel="Altitude (m)", ylabel="Time to Land (s)",
               title="Estimated Time to Landing vs Altitude")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9,
              prop={"family": FONT_FAMILY})

    fig.tight_layout()
    path = os.path.join(OUTPUT_DIR, "fig3_ttl_vs_altitude.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ Saved {path}")


# ─── Figure 4: Summary Panel (2×2) ──────────────────────────────────────────
def plot_summary_panel(df, errors, actual_lat, actual_lon, launch_lat, launch_lon):
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))

    # ── Panel (0,0): Track ──
    ax = axes[0, 0]
    ax.plot(df["gps_lon"], df["gps_lat"], color=COLOR_TRACK, linewidth=1.4, label="GPS track")
    ax.scatter(df["pred_lon"], df["pred_lat"], color=COLOR_PREDICTED, s=10, alpha=0.5, label="Predictions")
    ax.scatter(actual_lon, actual_lat, color=COLOR_LANDING, s=120, marker="*", zorder=6,
               label="Actual landing", edgecolors="k", linewidths=0.4)
    if launch_lat and launch_lon:
        ax.scatter(launch_lon, launch_lat, color=COLOR_LANDING, s=80, marker="^", zorder=6,
                   label="Launch", edgecolors="k", linewidths=0.4)
    ax.set_aspect("equal")
    style_axes(ax, xlabel="Lon (°)", ylabel="Lat (°)", title="Track & Predictions")
    ax.legend(fontsize=6, framealpha=0.8, prop={"family": FONT_FAMILY})

    # ── Panel (0,1): Error vs Altitude ──
    ax = axes[0, 1]
    ax.plot(df["altitude_m"], errors, color=COLOR_PREDICTED, linewidth=1.4)
    ax.fill_between(df["altitude_m"], 0, df["uncertainty_m"], color=COLOR_UNCERTAINTY, alpha=0.18)
    ax.axhline(errors.iloc[-1], color=COLOR_LANDING, linestyle="--", linewidth=1.0,
               label=f"Final: {errors.iloc[-1]:.1f} m")
    ax.invert_xaxis()
    style_axes(ax, xlabel="Altitude (m)", ylabel="Error (m)", title="Error vs Altitude")
    ax.legend(fontsize=7, framealpha=0.8, prop={"family": FONT_FAMILY})

    # ── Panel (1,0): TTL vs Altitude ──
    ax = axes[1, 0]
    ttl = df["t_to_land_s"].values
    alt = df["altitude_m"].values
    
    # Filter out NaN values
    valid_mask = ~np.isnan(ttl)
    ttl_valid = ttl[valid_mask]
    alt_valid = alt[valid_mask]
    
    if len(ttl_valid) > 0:
        ax.plot(alt_valid, ttl_valid, color=COLOR_TRACK, linewidth=1.4)
        # Check for non-monotonic points
        for i in range(1, len(ttl_valid)):
            if ttl_valid[i] > ttl_valid[i - 1] + 0.5:
                ax.scatter(alt_valid[i], ttl_valid[i], color="red", s=20, zorder=5)
    
    ax.invert_xaxis()
    style_axes(ax, xlabel="Altitude (m)", ylabel="TTL (s)", title="Time to Land vs Altitude")

    # ── Panel (1,1): Error Histogram ──
    ax = axes[1, 1]
    ax.hist(errors, bins=25, color=COLOR_PREDICTED, alpha=0.7, edgecolor="white", linewidth=0.5)
    ax.axvline(errors.iloc[-1], color=COLOR_LANDING, linestyle="--", linewidth=1.5,
               label=f"Final: {errors.iloc[-1]:.1f} m")
    ax.axvline(errors.mean(), color=COLOR_UNCERTAINTY, linestyle="-.", linewidth=1.5,
               label=f"Mean: {errors.mean():.1f} m")
    style_axes(ax, xlabel="Error (m)", ylabel="Count", title="Error Distribution")
    ax.legend(fontsize=7, framealpha=0.8, prop={"family": FONT_FAMILY})

    fig.suptitle("IGNITIA CanSat 2026 — Landing Prediction Summary",
                 fontfamily=FONT_FAMILY, fontsize=14, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(OUTPUT_DIR, "fig4_summary_panel.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ Saved {path}")


# ─── CSV summary output ─────────────────────────────────────────────────────
def write_summary_csv(df, errors, actual_lat, actual_lon):
    final_pred_lat = df["pred_lat"].iloc[-1]
    final_pred_lon = df["pred_lon"].iloc[-1]
    fallback_frac  = df["gps_fallback"].mean()

    summary = pd.DataFrame([{
        "final_error_m":        round(errors.iloc[-1], 2),
        "mean_error_m":         round(errors.mean(), 2),
        "min_error_m":          round(errors.min(), 2),
        "max_error_m":          round(errors.max(), 2),
        "final_pred_lat":       round(final_pred_lat, 7),
        "final_pred_lon":       round(final_pred_lon, 7),
        "actual_lat":           round(actual_lat, 7),
        "actual_lon":           round(actual_lon, 7),
        "gps_fallback_fraction": round(fallback_frac, 4),
    }])

    path = os.path.join(OUTPUT_DIR, "landing_accuracy_summary.csv")
    summary.to_csv(path, index=False)
    print(f"  ✓ Saved {path}")
    return summary


# ─── Console summary ────────────────────────────────────────────────────────
def print_console_summary(df, errors, actual_lat, actual_lon):
    final_err = errors.iloc[-1]
    mean_err  = errors.mean()
    min_err   = errors.min()
    max_err   = errors.max()
    fb_frac   = df["gps_fallback"].mean() * 100.0
    target_met = final_err < 10.0

    print()
    print("=" * 64)
    print("  IGNITIA CanSat 2026 — Landing Prediction Validation Report")
    print("=" * 64)
    print()
    print(f"  Actual landing:        {actual_lat:.7f}°, {actual_lon:.7f}°")
    print(f"  Final predicted:       {df['pred_lat'].iloc[-1]:.7f}°, {df['pred_lon'].iloc[-1]:.7f}°")
    print()
    print(f"  Final error:           {final_err:>8.2f} m")
    print(f"  Mean error:            {mean_err:>8.2f} m")
    print(f"  Min error:             {min_err:>8.2f} m")
    print(f"  Max error:             {max_err:>8.2f} m")
    print()
    print(f"  GPS fallback fraction: {fb_frac:>7.1f} %")
    print(f"  Total descent samples: {len(df)}")
    print()
    print(f"  Target (<10 m):        {'✓ YES — TARGET MET' if target_met else '✗ NO — TARGET NOT MET'}")
    print()
    print("=" * 64)
    print()


# ─── Determine actual landing coordinates ───────────────────────────────────
def find_actual_landing(df, cli_lat=None, cli_lon=None):
    """
    Determine actual landing coordinates.
    Priority: CLI args > last row with altitude < 5m and fix_type >= 2.
    """
    if cli_lat is not None and cli_lon is not None:
        return cli_lat, cli_lon

    # Find last row where altitude_m < 5 and fix_type >= 2
    mask = (df["altitude_m"] < 5.0) & (df["fix_type"] >= 2)
    if mask.any():
        last_valid = df.loc[mask].iloc[-1]
        return last_valid["gps_lat"], last_valid["gps_lon"]

    # Fallback: last row with valid GPS fix
    mask2 = df["fix_type"] >= 2
    if mask2.any():
        last_valid = df.loc[mask2].iloc[-1]
        print("  ⚠ No row with altitude < 5 m found; using last valid GPS fix.")
        return last_valid["gps_lat"], last_valid["gps_lon"]

    # Last resort: last row
    print("  ⚠ No valid GPS fix found; using last row as actual landing.")
    return df["gps_lat"].iloc[-1], df["gps_lon"].iloc[-1]


# ─── Main pipeline ──────────────────────────────────────────────────────────
def run_validation(df, actual_lat, actual_lon, launch_lat=None, launch_lon=None):
    """Run the full validation pipeline on a DataFrame."""

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Compute per-row prediction error
    errors = haversine_vec(
        df["pred_lat"].values, df["pred_lon"].values,
        actual_lat, actual_lon
    )
    errors = pd.Series(errors, index=df.index)

    # Derive launch site from first row if not given
    if launch_lat is None:
        launch_lat = df["gps_lat"].iloc[0]
    if launch_lon is None:
        launch_lon = df["gps_lon"].iloc[0]

    print("\n📊 Generating figures...")
    plot_prediction_track(df, actual_lat, actual_lon, launch_lat, launch_lon)
    plot_error_vs_altitude(df, errors)
    plot_ttl_vs_altitude(df)
    plot_summary_panel(df, errors, actual_lat, actual_lon, launch_lat, launch_lon)

    print("\n📄 Writing summary CSV...")
    write_summary_csv(df, errors, actual_lat, actual_lon)

    print_console_summary(df, errors, actual_lat, actual_lon)


# ─── CLI ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="IGNITIA CanSat 2026 — Post-Flight Landing Prediction Validation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python cansat_landing_validation.py --csv log.csv
  python cansat_landing_validation.py --csv log.csv --actual-lat 12.9716 --actual-lon 80.2209
  python cansat_landing_validation.py --simulate
        """
    )
    parser.add_argument("--csv", type=str, default=None,
                        help="Path to SD card CSV log file")
    parser.add_argument("--actual-lat", type=float, default=None,
                        help="Actual landing latitude (decimal degrees)")
    parser.add_argument("--actual-lon", type=float, default=None,
                        help="Actual landing longitude (decimal degrees)")
    parser.add_argument("--simulate", action="store_true",
                        help="Run simulation mode (350 m descent, pre-flight verification)")

    args = parser.parse_args()

    if args.simulate:
        print("🚀 IGNITIA CanSat 2026 — Simulation Mode")
        print("   Generating synthetic 350 m descent data...")
        print("   v_north=1.2 m/s, v_east=0.8 m/s, GPS valid throughout\n")

        df = generate_simulated_data()

        # Save simulated CSV for reference
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        sim_csv_path = os.path.join(OUTPUT_DIR, "simulated_flight_log.csv")
        df.to_csv(sim_csv_path, index=False)
        print(f"  ✓ Saved simulated log: {sim_csv_path}")

        actual_lat, actual_lon = find_actual_landing(df)
        run_validation(df, actual_lat, actual_lon)

    elif args.csv:
        print(f"📂 Loading CSV: {args.csv}")
        if not os.path.isfile(args.csv):
            print(f"  ✗ File not found: {args.csv}")
            sys.exit(1)

        df_raw = pd.read_csv(args.csv)
        
        print(f"  ✓ Loaded {len(df_raw)} rows")
        print(f"\n🔍 Auto-detecting column mappings...")
        
        # Auto-detect column mappings
        column_mapping = auto_map_columns(df_raw)
        
        if column_mapping:
            print(f"  ✓ Detected mappings:")
            for std_name, orig_name in sorted(column_mapping.items()):
                print(f"    {std_name:20s} <- {orig_name}")
        
        # Validate and prepare standardized DataFrame
        try:
            df = validate_and_prepare_dataframe(df_raw, column_mapping)
            print(f"\n  ✓ Successfully mapped {len(column_mapping)} columns")
        except ValueError as e:
            print(f"\n  ✗ Column mapping failed: {e}")
            sys.exit(1)

        # Determine actual landing
        actual_lat, actual_lon = find_actual_landing(df, args.actual_lat, args.actual_lon)
        run_validation(df, actual_lat, actual_lon)

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
