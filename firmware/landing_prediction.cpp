/**
 * ============================================================================
 *  IGNITIA CanSat 2026 — Experiment 2: Real-Time Landing Zone Prediction
 *  Team Ignition
 *
 *  landing_prediction.cpp
 *  Onboard kinematic landing prediction algorithm implementation
 *
 *  Platform : RP2040-Zero @ 125 MHz
 *  Cycle    : 1 Hz during DESCENT state only
 *  Budget   : < 5 ms per call, zero heap allocation
 * ============================================================================
 */

#include "landing_prediction.h"
#include <math.h>

/* ─── Constants ──────────────────────────────────────────────────────── */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_M          111320.0f   /* metres per degree latitude           */
#define EMA_ALPHA         0.3f        /* new-sample weight for EMA filter     */
#define EMA_BETA          0.7f        /* previous-value weight (1 − α)        */
#define DT_DEFAULT        1.0f        /* nominal sample period (seconds)      */

#define VZ_MIN_THRESH     0.05f       /* m/s — below this, treat as hovering  */
#define TTL_MIN           1.0f        /* seconds — minimum time-to-land       */
#define TTL_MAX           120.0f      /* seconds — maximum time-to-land       */
#define UNC_MIN           5.0f        /* metres — minimum uncertainty radius  */
#define UNC_MAX           200.0f      /* metres — maximum uncertainty radius  */
#define COS_LAT_GUARD     0.001f      /* guard against division-by-zero       */
#define GPS_MIN_SATS      4           /* minimum satellites for valid fix     */
#define GPS_MIN_FIX_QUAL  1           /* minimum fix quality (1=GPS, 2=DGPS)  */

/* ─── Static filter state (persists between calls) ───────────────────── */
static bool   s_first_call     = true;

/* vertical */
static float  s_alt_prev       = 0.0f;
static float  s_vz_filt        = 0.0f;

/* horizontal */
static double s_lat_prev       = 0.0;
static double s_lon_prev       = 0.0;
static float  s_v_north_filt   = 0.0f;
static float  s_v_east_filt    = 0.0f;

/* last valid GPS fix (for fallback) */
static double s_last_valid_lat = 0.0;
static double s_last_valid_lon = 0.0;

/* ─── Helper: clamp a float to [lo, hi] ──────────────────────────────── */
static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ─── Public: reset filter state (call on DESCENT entry) ─────────────── */
void resetLandingPrediction(void) {
    s_first_call = true;
}

/* ─── Public: main prediction function ───────────────────────────────── */
LandingPrediction computeLandingPrediction(SensorData* s) {
    LandingPrediction result;
    const float dt = DT_DEFAULT;  /* 1 Hz cycle → 1.0 s */

    /* ── First-call initialisation ───────────────────────────────────── */
    if (s_first_call) {
        s_alt_prev       = s->altitude_m;
        s_vz_filt        = 0.0f;

        s_lat_prev       = s->gps_lat;
        s_lon_prev       = s->gps_lon;
        s_v_north_filt   = 0.0f;
        s_v_east_filt    = 0.0f;

        s_last_valid_lat = s->gps_lat;
        s_last_valid_lon = s->gps_lon;

        s_first_call     = false;

        /* Return a safe "no prediction yet" result on first call */
        result.pred_lat      = (float)s->gps_lat;
        result.pred_lon      = (float)s->gps_lon;
        result.uncertainty_m = UNC_MAX;
        result.t_to_land_s   = TTL_MAX;
        result.gps_fallback  = (s->gps_fix_quality < GPS_MIN_FIX_QUAL || 
                                s->gps_satellites < GPS_MIN_SATS) ? 1 : 0;
        return result;
    }

    /* ── Step 1: Vertical velocity & time-to-land ────────────────────── */
    float v_z = (s->altitude_m - s_alt_prev) / dt;
    s_vz_filt = EMA_ALPHA * v_z + EMA_BETA * s_vz_filt;
    s_alt_prev = s->altitude_m;

    float t_to_land;
    if (fabsf(s_vz_filt) < VZ_MIN_THRESH) {
        t_to_land = TTL_MAX;
    } else {
        t_to_land = s->altitude_m / fabsf(s_vz_filt);
        t_to_land = clampf(t_to_land, TTL_MIN, TTL_MAX);
    }

    /* ── Step 2: Horizontal drift velocity ───────────────────────────── */
    int gps_fallback = 0;
    bool gps_valid = (s->gps_fix_quality >= GPS_MIN_FIX_QUAL) && 
                     (s->gps_satellites >= GPS_MIN_SATS);

    if (gps_valid) {
        /* PRIMARY: compute velocity from successive GPS fixes */
        float v_north = (float)(s->gps_lat - s_lat_prev) * DEG_TO_M / dt;
        float v_east  = (float)(s->gps_lon - s_lon_prev) * DEG_TO_M
                        * cosf((float)s->gps_lat * PI / 180.0f) / dt;

        s_v_north_filt = EMA_ALPHA * v_north + EMA_BETA * s_v_north_filt;
        s_v_east_filt  = EMA_ALPHA * v_east  + EMA_BETA * s_v_east_filt;

        /* Update previous & last-valid positions */
        s_lat_prev       = s->gps_lat;
        s_lon_prev       = s->gps_lon;
        s_last_valid_lat = s->gps_lat;
        s_last_valid_lon = s->gps_lon;

        gps_fallback = 0;
    } else {
        /* FALLBACK: keep last filtered velocities, project from last fix */
        gps_fallback = 1;
    }

    /* ── Step 3: Project landing coordinates ─────────────────────────── */
    float origin_lat = gps_fallback ? (float)s_last_valid_lat : (float)s->gps_lat;
    float origin_lon = gps_fallback ? (float)s_last_valid_lon : (float)s->gps_lon;

    float cos_lat = cosf(origin_lat * PI / 180.0f);
    if (fabsf(cos_lat) < COS_LAT_GUARD) {
        cos_lat = COS_LAT_GUARD;
    }

    result.pred_lat = origin_lat + (s_v_north_filt * t_to_land) / DEG_TO_M;
    result.pred_lon = origin_lon + (s_v_east_filt  * t_to_land)
                      / (DEG_TO_M * cos_lat);

    /* ── Step 4: Uncertainty radius ──────────────────────────────────── */
    float uncertainty = s->altitude_m * 0.1f + fabsf(s_vz_filt) * 2.0f;
    result.uncertainty_m = clampf(uncertainty, UNC_MIN, UNC_MAX);

    result.t_to_land_s  = t_to_land;
    result.gps_fallback = gps_fallback;

    return result;
}
