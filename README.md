# IGNITIA CanSat 2026 — Landing Prediction System

**Team Ignition | Experiment 2: Real-Time Landing Zone Prediction**

A complete landing prediction system for CanSat competitions, featuring onboard real-time prediction firmware and post-flight validation analysis.

## Overview

This system predicts where a descending CanSat will land using kinematic modeling based on GPS drift velocity and barometric altitude. The prediction runs in real-time on the CanSat during descent and is validated post-flight against actual landing coordinates.

**Target Accuracy:** <10 meters final prediction error

## System Components

### 1. Firmware (C++ for RP2040)

Real-time prediction algorithm that runs onboard during descent.

**Files:**
- `firmware/landing_prediction.h` - API header
- `firmware/landing_prediction.cpp` - Core algorithm
- `firmware/integration_snippet.ino` - Integration example

**Performance:**
- Execution: <5 ms per cycle
- Update rate: 1 Hz
- Platform: RP2040-Zero @ 125 MHz

**Algorithm:**
Uses only GPS data from your TelemetrySnapshot:
1. Vertical velocity from barometric altitude changes
2. Horizontal drift from GPS position changes (lat/lon)
3. Projects landing point using time-to-land estimate
4. Validates GPS quality (satellites ≥4, fix_quality ≥1)

**Required from TelemetrySnapshot:**
- `baro_altitude` - For descent rate
- `latitude`, `longitude` - For horizontal drift
- `gps_satellites`, `gps_fix_quality` - For GPS validation

Other fields (temperature, accel, etc.) are not used for prediction.

### 2. Validation Script (Python)

Post-flight analysis tool that evaluates prediction accuracy against actual landing coordinates.

**File:** `cansat_landing_validation.py`

**Features:**
- Processes real flight logs from SD card
- Generates simulated descent data for pre-flight testing
- Produces comprehensive visualizations and metrics
- Validates against <10m accuracy target

## Installation

### Python Dependencies

```bash
pip install numpy scipy matplotlib pandas
```

Or using conda:
```bash
conda install numpy scipy matplotlib pandas
```

### Firmware Integration

1. Copy `landing_prediction.h` and `landing_prediction.cpp` to your Arduino project
2. Include the header in your main sketch:
   ```cpp
   #include "landing_prediction.h"
   ```
3. Call `resetLandingPrediction()` when entering DESCENT state
4. Call `computeLandingPrediction(&sensorData)` once per telemetry cycle
5. Append prediction results to telemetry packet and SD log

See `firmware/integration_snippet.ino` for complete integration example.

## Usage

### Simulation Mode (Pre-Flight Testing)

Generate synthetic descent data to verify the analysis pipeline:

```bash
python cansat_landing_validation.py --simulate
```

This simulates a 350m descent with constant horizontal drift and generates all output figures.

### Real Flight Data Analysis

Process actual SD card logs from your CanSat flight:

```bash
python cansat_landing_validation.py --csv flight_log.csv
```

The script automatically detects actual landing coordinates from the log (last GPS fix with altitude <5m).

### Manual Landing Coordinates

Override automatic landing detection with known coordinates:

```bash
python cansat_landing_validation.py --csv flight_log.csv --actual-lat 12.9716 --actual-lon 80.2209
```

## CSV Log Format

The validation script automatically detects columns based on keywords. Your CSV needs these fields:

**Required:**
- Altitude (barometric): `altitude`, `baro_altitude`, etc.
- GPS Latitude: `latitude`, `lat`, etc.
- GPS Longitude: `longitude`, `lon`, etc.
- Predicted Latitude: `pred_lat`, `pred_latitude`, etc.
- Predicted Longitude: `pred_lon`, `pred_longitude`, etc.

**Optional** (defaults used if missing):
- GPS satellites, fix quality, uncertainty, time-to-land

The script searches column names (case-insensitive) and maps them automatically.

### Example CSV

```csv
timestamp,baro_altitude,latitude,longitude,gps_satellites,gps_fix_quality,pred_lat,pred_lon
2026-04-10 14:21:52,305.2,12.971600,80.220900,8,1,12.971650,80.220950
```

Column names can vary - the script detects them by keywords.

## Output Files

All outputs are saved to `./cansat_landing_results/`:

### Visualizations

1. **`fig1_prediction_track.png`**
   - Map view of GPS track and predicted landing points
   - Prediction opacity fades with altitude
   - Uncertainty circles at 300m, 200m, 100m, 50m
   - Shows launch site and actual landing location

2. **`fig2_error_vs_altitude.png`**
   - Prediction error throughout descent
   - Shaded uncertainty band
   - Final error highlighted

3. **`fig3_ttl_vs_altitude.png`**
   - Time-to-land estimates vs altitude
   - Highlights non-monotonic segments (algorithm issues)

4. **`fig4_summary_panel.png`**
   - 2×2 panel combining all metrics
   - Track, error vs altitude, TTL, error histogram

### Data Files

1. **`landing_accuracy_summary.csv`**
   - Final error, mean error, min/max error
   - Predicted vs actual coordinates
   - GPS fallback fraction

2. **`simulated_flight_log.csv`** (simulation mode only)
   - Complete synthetic descent telemetry

## Performance Metrics

The validation script reports:

- **Final error**: Distance between final prediction and actual landing
- **Mean error**: Average prediction error throughout descent
- **Min/Max error**: Error range during descent
- **GPS fallback fraction**: Percentage of time using stale GPS fix
- **Target status**: Whether <10m accuracy target was met

### Example Output

```
================================================================
  IGNITIA CanSat 2026 — Landing Prediction Validation Report
================================================================

  Actual landing:        12.9726753°, 80.2216403°
  Final predicted:       12.9726851°, 80.2216488°

  Final error:              1.43 m
  Mean error:               7.58 m
  Min error:                0.18 m
  Max error:              142.67 m

  GPS fallback fraction:    0.0 %
  Total descent samples: 101

  Target (<10 m):        ✓ YES — TARGET MET

================================================================
```

## Algorithm Details

### Vertical Velocity Estimation
- Computes descent rate from successive altitude readings
- Exponential moving average (EMA) filter with α=0.3
- Time-to-land = altitude / |descent_velocity|
- Clamped to [1s, 120s] range

### Horizontal Drift Estimation
- Computes north/east velocity from GPS position deltas
- EMA filter with α=0.3 for noise reduction
- Validates GPS quality (fix_type ≥2, HDOP ≤5.0)
- Falls back to last valid fix when GPS signal lost

### Uncertainty Estimation
- Base uncertainty: 10% of current altitude
- Velocity component: 2× descent rate
- Clamped to [5m, 200m] range
- Increases with altitude and descent rate

## Troubleshooting

### "Missing columns" error
Ensure your CSV log includes all required columns. Check column names match exactly (case-sensitive).

### High prediction errors
- Check GPS fix quality (fix_type, HDOP values)
- Verify barometric altitude calibration
- Look for GPS fallback events in the log
- Check for non-monotonic time-to-land values (algorithm instability)

### Unicode/emoji display issues on Windows
The script automatically handles Windows console encoding. If you still see garbled characters, use:
```bash
chcp 65001
python cansat_landing_validation.py --simulate
```

## Competition Context

This system is designed for CanSat competitions where teams must:
- Deploy a small satellite from a rocket
- Transmit real-time telemetry during descent
- Predict landing zone for recovery operations
- Demonstrate <10m prediction accuracy

## License

Developed by Team Ignition for IGNITIA CanSat 2026 competition.

## Contact

For questions or collaboration, contact Team Ignition.
