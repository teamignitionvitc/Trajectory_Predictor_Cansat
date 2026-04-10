/**
 * ============================================================================
 *  IGNITIA CanSat 2026 — Integration Snippet
 *  Team Ignition
 *
 *  Shows where to call computeLandingPrediction() in the main loop,
 *  how to pack output into telemetry, and how to log to SD card.
 *
 *  Paste this logic into your existing main loop, inside the
 *  DESCENT flight-state case.
 * ============================================================================
 */

#include "landing_prediction.h"

/* ── In your flight state machine, when entering DESCENT: ──────────── */
// case STATE_APOGEE_TO_DESCENT:
//     resetLandingPrediction();   // reinitialise filter state
//     flight_state = STATE_DESCENT;
//     break;

/* ── Inside the main 1 Hz telemetry loop, DESCENT case: ───────────── */

// 1. Populate SensorData from your existing sensor reads
SensorData sd;
sd.altitude_m  = bmp280_altitude;     // from BMP280 driver
sd.pressure_pa = bmp280_pressure;
sd.gps_lat     = neo6m_lat;           // from NEO-6M parser
sd.gps_lon     = neo6m_lon;
sd.gps_alt_m   = neo6m_alt;
sd.fix_type    = neo6m_fix;
sd.hdop        = neo6m_hdop;
sd.accel_x     = mpu6500_ax;          // from MPU6500 driver
sd.accel_y     = mpu6500_ay;
sd.accel_z     = mpu6500_az;
sd.gyro_x      = mpu6500_gx;
sd.gyro_y      = mpu6500_gy;
sd.gyro_z      = mpu6500_gz;

// 2. Run prediction (< 5 ms on RP2040)
LandingPrediction lp = computeLandingPrediction(&sd);

// 3. Append to telemetry string (comma-separated, end of packet)
char pred_buf[80];
snprintf(pred_buf, sizeof(pred_buf),
         ",%.6f,%.6f,%.1f,%.1f,%d",
         lp.pred_lat, lp.pred_lon,
         lp.uncertainty_m, lp.t_to_land_s,
         lp.gps_fallback);
strcat(telemetry_packet, pred_buf);   // append to existing packet

// 4. Append same fields to SD log line (same order)
strcat(sd_log_line, pred_buf);

// 5. Transmit via NRF52840 UART
Serial1.println(telemetry_packet);

// 6. Write to SD card
logFile.println(sd_log_line);
logFile.flush();

/* ── New telemetry fields (appended at end of CSV packet): ─────────── */
// pred_lat, pred_lon, uncertainty_m, t_to_land_s, gps_fallback
//
// Example full packet (existing + new):
// IGNITIA,12345,12.97160,80.22090,305.2,...,12.97198,80.22134,18.5,42.3,0
//
// SD log columns (append at end, same order):
// ..., pred_lat, pred_lon, uncertainty_m, t_to_land_s, gps_fallback
