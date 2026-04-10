/**
 * ============================================================================
 *  IGNITIA CanSat 2026 — Integration Snippet
 *  Team Ignition
 *
 *  Integration example for TelemetrySnapshot structure.
 *  Only GPS-related fields are used for landing prediction.
 * ============================================================================
 */

#include "landing_prediction.h"

/* ── Your TelemetrySnapshot structure ────────────────────────────────── */
typedef struct {
    volatile uint32_t sequence;
    float altitude;
    float temperature;
    float baro_altitude;        // ← USED for prediction
    float accel_x, accel_y, accel_z;
    double latitude, longitude; // ← USED for prediction
    float gps_altitude;         // ← USED for prediction
    int gps_satellites;         // ← USED for GPS validation
    float gps_speed;
    int gps_fix_quality;        // ← USED for GPS validation
    float velocity;
    float uv_voltage;
    uint8_t gp12_value;
} TelemetrySnapshot;

TelemetrySnapshot telemetry;  // Your existing telemetry

/* ── When entering DESCENT state: ──────────────────────────────────── */
// resetLandingPrediction();

/* ── In main loop (1 Hz during DESCENT): ───────────────────────────── */

void loop() {
    // ... your sensor reading code updates telemetry ...
    
    // Map to SensorData (only GPS fields matter)
    SensorData sd;
    sd.altitude_m      = telemetry.baro_altitude;
    sd.temperature     = telemetry.temperature;
    sd.gps_lat         = telemetry.latitude;
    sd.gps_lon         = telemetry.longitude;
    sd.gps_alt_m       = telemetry.gps_altitude;
    sd.gps_satellites  = telemetry.gps_satellites;
    sd.gps_fix_quality = telemetry.gps_fix_quality;
    sd.gps_speed       = telemetry.gps_speed;
    sd.accel_x         = telemetry.accel_x;
    sd.accel_y         = telemetry.accel_y;
    sd.accel_z         = telemetry.accel_z;
    
    // Run prediction
    LandingPrediction lp = computeLandingPrediction(&sd);
    
    // Append to telemetry packet and SD log
    char pred_buf[100];
    snprintf(pred_buf, sizeof(pred_buf),
             ",%.7f,%.7f,%.1f,%.1f,%d",
             lp.pred_lat, lp.pred_lon,
             lp.uncertainty_m, lp.t_to_land_s,
             lp.gps_fallback);
    
    // Add to your telemetry string and SD log
    strcat(telemetry_packet, pred_buf);
    strcat(sd_log_line, pred_buf);
}

/* ── Fields Used for Prediction ──────────────────────────────────────── */
// FROM TelemetrySnapshot:
//   - baro_altitude  → vertical velocity calculation
//   - latitude       → horizontal drift calculation
//   - longitude      → horizontal drift calculation
//   - gps_satellites → GPS quality check (need ≥4)
//   - gps_fix_quality → GPS quality check (need ≥1)
//
// NOT USED (but logged for reference):
//   - temperature, accel_x/y/z, gps_speed, velocity, etc.

/* ── New CSV Columns (append to your log): ────────────────────────── */
// pred_lat, pred_lon, uncertainty_m, t_to_land_s, gps_fallback
