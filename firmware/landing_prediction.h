/**
 * ============================================================================
 *  IGNITIA CanSat 2026 — Experiment 2: Real-Time Landing Zone Prediction
 *  Team Ignition
 *
 *  landing_prediction.h
 *  Onboard kinematic landing prediction algorithm header
 *
 *  Platform : RP2040-Zero @ 125 MHz
 *  Cycle    : Called once per telemetry cycle (1 Hz) during DESCENT state
 *  Budget   : < 5 ms per call, zero dynamic allocation
 * ============================================================================
 */

#ifndef LANDING_PREDICTION_H
#define LANDING_PREDICTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Input: fused sensor snapshot passed every cycle ─────────────────── */
typedef struct {
    /* BMP280 — barometric altitude */
    float altitude_m;        /* metres above ground (AGL)                  */
    float pressure_pa;       /* raw pressure in Pascals                    */

    /* NEO-6M — GNSS fix */
    float gps_lat;           /* decimal degrees, WGS-84                    */
    float gps_lon;           /* decimal degrees, WGS-84                    */
    float gps_alt_m;         /* GPS altitude (metres)                      */
    int   fix_type;          /* 0 = none, 2 = 2-D, 3 = 3-D                */
    float hdop;              /* horizontal dilution of precision           */

    /* MPU6500 — IMU (reserved for future fusion) */
    float accel_x;           /* m/s²                                       */
    float accel_y;           /* m/s²                                       */
    float accel_z;           /* m/s²                                       */
    float gyro_x;            /* °/s                                        */
    float gyro_y;            /* °/s                                        */
    float gyro_z;            /* °/s                                        */
} SensorData;

/* ─── Output: prediction result packed into telemetry ─────────────────── */
typedef struct {
    float pred_lat;          /* predicted landing latitude  (decimal deg)  */
    float pred_lon;          /* predicted landing longitude (decimal deg)  */
    float uncertainty_m;     /* estimated uncertainty radius (metres)      */
    float t_to_land_s;       /* estimated seconds to ground impact         */
    int   gps_fallback;      /* 1 = using stale GPS fix, 0 = live fix     */
} LandingPrediction;

/**
 * Compute the predicted landing coordinates from the latest sensor snapshot.
 *
 * Must ONLY be called during the DESCENT flight state.
 * On first call after DESCENT entry, all internal filter state is
 * reinitialised from the current sensor readings.
 *
 * @param s  Pointer to the current sensor data snapshot (non-NULL)
 * @return   LandingPrediction struct with results
 */
LandingPrediction computeLandingPrediction(SensorData* s);

/**
 * Call once when the flight state machine transitions INTO DESCENT.
 * Forces the prediction module to reinitialise its filter state on
 * the next computeLandingPrediction() call.
 */
void resetLandingPrediction(void);

#ifdef __cplusplus
}
#endif

#endif /* LANDING_PREDICTION_H */
