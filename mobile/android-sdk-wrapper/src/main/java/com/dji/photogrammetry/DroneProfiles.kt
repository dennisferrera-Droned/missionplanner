package com.dji.photogrammetry

/**
 * Camera and flight profiles for supported DJI drones.
 *
 * Each profile captures the real-world sensor geometry needed for accurate
 * GSD / overlap calculations, plus the hard limits the firmware enforces
 * so the SDK can warn or clamp values before uploading a mission.
 *
 * Supported profiles
 * ------------------
 *  MINI_3           – DJI Mini 3 (1/1.3" sensor, 24 mm equiv, 12 MP)
 *  MINI_3_PRO       – DJI Mini 3 Pro (1/1.3" sensor, 24 mm equiv, 48 MP)
 *  MINI_4_PRO       – DJI Mini 4 Pro (1/1.3" sensor, 24 mm equiv, 48 MP)
 *  MAVIC_3_CLASSIC  – DJI Mavic 3 Classic (4/3 sensor, 24 mm equiv, 20 MP)
 *  MAVIC_3_MULTISPECTRAL – Mavic 3 Multispectral (4/3 + 4-band MS sensor)
 *  PHANTOM_4_RTK    – DJI Phantom 4 RTK (1" sensor, 24 mm equiv, 20 MP)
 *  GENERIC          – Fallback for unlisted cameras
 */

enum class DroneModel {
    MINI_3,
    MINI_3_PRO,
    MINI_4_PRO,
    MAVIC_3_CLASSIC,
    MAVIC_3_MULTISPECTRAL,
    PHANTOM_4_RTK,
    GENERIC
}

data class FlightLimits(
    /** Maximum mission flight speed (m/s). Firmware will reject higher values. */
    val maxSpeedMs: Double,
    /** Minimum safe AGL altitude (m). */
    val minAltitudeM: Double,
    /** Maximum AGL altitude before regulatory warning (m). */
    val maxAltitudeM: Double,
    /** Maximum waypoints per WaypointV2 mission. */
    val maxWaypoints: Int,
    /** True when the drone has obstacle-sensing (forward + rear at minimum). */
    val hasObstacleSensing: Boolean,
    /** True when built-in RTK receiver is present. */
    val hasRtk: Boolean,
    /** Minimum camera trigger interval (seconds). */
    val minTriggerIntervalS: Double,
    /**
     * True when this model requires MSDK v5.
     * Mini 3, Mini 3 Pro, Mini 4 Pro, Mavic 3 series all require MSDK v5.
     * Phantom 4 RTK works with both v4 and v5.
     */
    val requiresMsdkV5: Boolean
)

data class DroneProfile(
    val model: DroneModel,
    val displayName: String,
    val cameraSettings: CameraSettings,
    val limits: FlightLimits
)

object DroneProfiles {

    // -----------------------------------------------------------------------
    // DJI Mini 3
    // -----------------------------------------------------------------------
    //  Sensor: 1/1.3" CMOS (9.6 mm × 7.2 mm)
    //  Physical focal length: 6.7 mm
    //  35 mm equivalent: 24 mm
    //  Effective pixels: 12 MP  (4032 × 3024)
    //  Max photo resolution: 4032 × 3024
    //  Video: 4K/60fps
    //  Max speed: 16 m/s sport; 10 m/s normal; 6 m/s cine
    //  Max flight time: 38 min
    //  Weight: 249 g (under most country registration thresholds)
    //  MSDK: v5 only
    //  Notes: NO obstacle sensing on sides; forward + downward + rear only
    // -----------------------------------------------------------------------
    val MINI_3 = DroneProfile(
        model = DroneModel.MINI_3,
        displayName = "DJI Mini 3",
        cameraSettings = CameraSettings(
            focalLengthMm   = 6.7,
            sensorWidthMm   = 9.6,
            sensorHeightMm  = 7.2,
            imageWidthPx    = 4032,
            imageHeightPx   = 3024,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 10.0,   // 10 m/s in Normal mode (recommended for photo)
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,  // 120 m AGL — regulatory default in most regions
            maxWaypoints         = 65535,  // MSDK v5 WaypointV2 supports up to 65535
            hasObstacleSensing   = true,
            hasRtk               = false,
            minTriggerIntervalS  = 0.7,    // Minimum shutter interval at this resolution
            requiresMsdkV5       = true
        )
    )

    // -----------------------------------------------------------------------
    // DJI Mini 3 Pro
    // -----------------------------------------------------------------------
    //  Sensor: 1/1.3" CMOS (9.6 mm × 7.2 mm)
    //  Physical focal length: 6.7 mm
    //  Effective pixels: 48 MP  (8064 × 6048) or 12 MP quad-bayer binned
    //  Obstacle sensing: omnidirectional (trinocular front/rear + side)
    //  MSDK: v5 only
    // -----------------------------------------------------------------------
    val MINI_3_PRO = DroneProfile(
        model = DroneModel.MINI_3_PRO,
        displayName = "DJI Mini 3 Pro",
        cameraSettings = CameraSettings(
            focalLengthMm   = 6.7,
            sensorWidthMm   = 9.6,
            sensorHeightMm  = 7.2,
            imageWidthPx    = 8064,    // 48 MP native
            imageHeightPx   = 6048,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 10.0,
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,
            maxWaypoints         = 65535,
            hasObstacleSensing   = true,
            hasRtk               = false,
            minTriggerIntervalS  = 0.7,
            requiresMsdkV5       = true
        )
    )

    // -----------------------------------------------------------------------
    // DJI Mini 4 Pro
    // -----------------------------------------------------------------------
    //  Sensor: 1/1.3" CMOS (9.6 mm × 7.2 mm)
    //  Physical focal length: 6.7 mm (main); also has 70 mm tele
    //  Effective pixels: 48 MP  (8064 × 6048)
    //  Obstacle sensing: omnidirectional
    //  MSDK: v5 only
    // -----------------------------------------------------------------------
    val MINI_4_PRO = DroneProfile(
        model = DroneModel.MINI_4_PRO,
        displayName = "DJI Mini 4 Pro",
        cameraSettings = CameraSettings(
            focalLengthMm   = 6.7,
            sensorWidthMm   = 9.6,
            sensorHeightMm  = 7.2,
            imageWidthPx    = 8064,
            imageHeightPx   = 6048,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 10.0,
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,
            maxWaypoints         = 65535,
            hasObstacleSensing   = true,
            hasRtk               = false,
            minTriggerIntervalS  = 0.7,
            requiresMsdkV5       = true
        )
    )

    // -----------------------------------------------------------------------
    // DJI Mavic 3 Classic
    // -----------------------------------------------------------------------
    //  Sensor: 4/3 CMOS (17.3 mm × 13 mm) — Hasselblad L-Format
    //  Physical focal length: 12.29 mm / 35 mm equiv: 24 mm
    //  Effective pixels: 20 MP  (5280 × 3956)
    //  MSDK: v5
    // -----------------------------------------------------------------------
    val MAVIC_3_CLASSIC = DroneProfile(
        model = DroneModel.MAVIC_3_CLASSIC,
        displayName = "DJI Mavic 3 Classic",
        cameraSettings = CameraSettings(
            focalLengthMm   = 12.29,
            sensorWidthMm   = 17.3,
            sensorHeightMm  = 13.0,
            imageWidthPx    = 5280,
            imageHeightPx   = 3956,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 15.0,
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,
            maxWaypoints         = 65535,
            hasObstacleSensing   = true,
            hasRtk               = false,
            minTriggerIntervalS  = 0.5,
            requiresMsdkV5       = true
        )
    )

    // -----------------------------------------------------------------------
    // DJI Phantom 4 RTK
    // -----------------------------------------------------------------------
    //  Sensor: 1" CMOS (13.2 mm × 8.8 mm)
    //  Physical focal length: 8.8 mm / 35 mm equiv: 24 mm
    //  Effective pixels: 20 MP  (5472 × 3648)
    //  Built-in RTK receiver (D-RTK 2 base or NTRIP)
    //  MSDK: v4 and v5 compatible
    // -----------------------------------------------------------------------
    val PHANTOM_4_RTK = DroneProfile(
        model = DroneModel.PHANTOM_4_RTK,
        displayName = "DJI Phantom 4 RTK",
        cameraSettings = CameraSettings(
            focalLengthMm   = 8.8,
            sensorWidthMm   = 13.2,
            sensorHeightMm  = 8.8,
            imageWidthPx    = 5472,
            imageHeightPx   = 3648,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 15.0,
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,
            maxWaypoints         = 65535,
            hasObstacleSensing   = false,
            hasRtk               = true,
            minTriggerIntervalS  = 0.5,
            requiresMsdkV5       = false
        )
    )

    // -----------------------------------------------------------------------
    // Generic / custom camera
    // -----------------------------------------------------------------------
    val GENERIC = DroneProfile(
        model = DroneModel.GENERIC,
        displayName = "Generic / Custom",
        cameraSettings = CameraSettings(
            focalLengthMm   = 24.0,
            sensorWidthMm   = 13.2,
            sensorHeightMm  = 8.8,
            imageWidthPx    = 4000,
            imageHeightPx   = 3000,
            gimbalPitchDeg  = -90.0
        ),
        limits = FlightLimits(
            maxSpeedMs           = 15.0,
            minAltitudeM         = 2.0,
            maxAltitudeM         = 120.0,
            maxWaypoints         = 65535,
            hasObstacleSensing   = false,
            hasRtk               = false,
            minTriggerIntervalS  = 0.5,
            requiresMsdkV5       = false
        )
    )

    /** Look up a profile by model enum. */
    fun forModel(model: DroneModel): DroneProfile = when (model) {
        DroneModel.MINI_3                -> MINI_3
        DroneModel.MINI_3_PRO            -> MINI_3_PRO
        DroneModel.MINI_4_PRO            -> MINI_4_PRO
        DroneModel.MAVIC_3_CLASSIC       -> MAVIC_3_CLASSIC
        DroneModel.MAVIC_3_MULTISPECTRAL -> MAVIC_3_CLASSIC   // same optics
        DroneModel.PHANTOM_4_RTK         -> PHANTOM_4_RTK
        DroneModel.GENERIC               -> GENERIC
    }

    /** Returns a validation result for proposed mission settings on this drone. */
    fun validate(profile: DroneProfile, settings: MissionSettings): ValidationResult {
        val warnings  = mutableListOf<String>()
        val errors    = mutableListOf<String>()
        val clamped   = settings.copy()

        // Speed check
        if (settings.flightSpeedMs > profile.limits.maxSpeedMs) {
            errors.add("Speed ${settings.flightSpeedMs} m/s exceeds ${profile.displayName} " +
                    "max of ${profile.limits.maxSpeedMs} m/s. " +
                    "Reduce to ${profile.limits.maxSpeedMs} m/s or less.")
        }

        // Altitude checks
        if (settings.altitudeM < profile.limits.minAltitudeM) {
            errors.add("Altitude ${settings.altitudeM} m is below the minimum " +
                    "${profile.limits.minAltitudeM} m AGL.")
        }
        if (settings.altitudeM > profile.limits.maxAltitudeM) {
            warnings.add("Altitude ${settings.altitudeM} m exceeds the recommended " +
                    "${profile.limits.maxAltitudeM} m AGL regulatory limit in most regions.")
        }

        // Mini 3 specific
        if (profile.model == DroneModel.MINI_3 || profile.model == DroneModel.MINI_3_PRO ||
            profile.model == DroneModel.MINI_4_PRO
        ) {
            if (settings.altitudeM > 120.0) {
                warnings.add("${profile.displayName} at ${settings.altitudeM} m: in most " +
                        "countries 249 g drones are still subject to 120 m AGL limits. " +
                        "Check local regulations.")
            }
            if (!profile.limits.hasRtk) {
                warnings.add("${profile.displayName} has no built-in RTK. For survey-grade " +
                        "accuracy use GCPs or an external PPK workflow.")
            }
            if (profile.limits.requiresMsdkV5) {
                warnings.add("${profile.displayName} requires DJI Mobile SDK v5. " +
                        "MSDK v4 is not compatible with this aircraft.")
            }
        }

        // Trigger interval
        settings.triggerTimeS?.let { ts ->
            if (ts < profile.limits.minTriggerIntervalS) {
                errors.add("Trigger interval ${ts} s is below the minimum " +
                        "${profile.limits.minTriggerIntervalS} s for ${profile.displayName}.")
            }
        }

        return ValidationResult(
            isValid   = errors.isEmpty(),
            errors    = errors,
            warnings  = warnings,
            clampedSettings = clamped
        )
    }
}

data class ValidationResult(
    val isValid: Boolean,
    val errors: List<String>,
    val warnings: List<String>,
    val clampedSettings: MissionSettings
)
