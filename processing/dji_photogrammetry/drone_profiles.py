"""
Camera and flight profiles for supported DJI drones.

Use these presets to get accurate GSD / overlap calculations without
manually specifying sensor dimensions. Each profile also records the
hard limits the firmware enforces so the SDK can warn before a mission
upload is rejected.

Supported profiles
------------------
  mini3            – DJI Mini 3 (1/1.3" sensor, 12 MP)    — MSDK v5
  mini3_pro        – DJI Mini 3 Pro (1/1.3" sensor, 48 MP) — MSDK v5
  mini4_pro        – DJI Mini 4 Pro (1/1.3" sensor, 48 MP) — MSDK v5
  mavic3_classic   – DJI Mavic 3 Classic (4/3 sensor, 20 MP) — MSDK v5
  phantom4_rtk     – DJI Phantom 4 RTK (1" sensor, 20 MP, built-in RTK)
  generic          – Fallback for custom / unlisted cameras
"""

from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class FlightLimits:
    """Hard limits enforced by the drone firmware."""
    max_speed_ms: float
    min_altitude_m: float
    max_altitude_m: float
    max_waypoints: int
    has_obstacle_sensing: bool
    has_rtk: bool
    min_trigger_interval_s: float
    requires_msdk_v5: bool


@dataclass
class DroneProfile:
    """Complete camera + flight profile for one drone model."""
    name: str
    display_name: str
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float
    image_width_px: int
    image_height_px: int
    gimbal_pitch_deg: float
    limits: FlightLimits


@dataclass
class ValidationResult:
    """Result of validating MissionSettings against a DroneProfile."""
    is_valid: bool
    errors: List[str]
    warnings: List[str]


# ---------------------------------------------------------------------------
# Profiles
# ---------------------------------------------------------------------------

PROFILES = {

    # -----------------------------------------------------------------------
    # DJI Mini 3
    # -----------------------------------------------------------------------
    # Sensor: 1/1.3" CMOS  |  9.6 × 7.2 mm
    # Physical focal length: 6.7 mm  (24 mm 35mm-equivalent)
    # Resolution: 12 MP  (4032 × 3024)
    # Max speed: 16 m/s sport; 10 m/s normal (use normal for photo missions)
    # Weight: 249 g — under most country registration thresholds
    # MSDK: v5 only  |  No built-in RTK
    # Obstacle sensing: forward + rear + downward (NOT side)
    # -----------------------------------------------------------------------
    "mini3": DroneProfile(
        name="mini3",
        display_name="DJI Mini 3",
        focal_length_mm=6.7,
        sensor_width_mm=9.6,
        sensor_height_mm=7.2,
        image_width_px=4032,
        image_height_px=3024,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=10.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=True,
            has_rtk=False,
            min_trigger_interval_s=0.7,
            requires_msdk_v5=True,
        ),
    ),

    # -----------------------------------------------------------------------
    # DJI Mini 3 Pro
    # -----------------------------------------------------------------------
    # Sensor: 1/1.3" CMOS  |  9.6 × 7.2 mm
    # Resolution: 48 MP  (8064 × 6048)  or  12 MP binned
    # Obstacle sensing: omnidirectional (trinocular front/rear + side)
    # MSDK: v5 only  |  No built-in RTK
    # -----------------------------------------------------------------------
    "mini3_pro": DroneProfile(
        name="mini3_pro",
        display_name="DJI Mini 3 Pro",
        focal_length_mm=6.7,
        sensor_width_mm=9.6,
        sensor_height_mm=7.2,
        image_width_px=8064,
        image_height_px=6048,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=10.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=True,
            has_rtk=False,
            min_trigger_interval_s=0.7,
            requires_msdk_v5=True,
        ),
    ),

    # -----------------------------------------------------------------------
    # DJI Mini 4 Pro
    # -----------------------------------------------------------------------
    # Sensor: 1/1.3" CMOS  |  9.6 × 7.2 mm
    # Resolution: 48 MP  (8064 × 6048)
    # Obstacle sensing: omnidirectional
    # MSDK: v5 only  |  No built-in RTK
    # -----------------------------------------------------------------------
    "mini4_pro": DroneProfile(
        name="mini4_pro",
        display_name="DJI Mini 4 Pro",
        focal_length_mm=6.7,
        sensor_width_mm=9.6,
        sensor_height_mm=7.2,
        image_width_px=8064,
        image_height_px=6048,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=10.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=True,
            has_rtk=False,
            min_trigger_interval_s=0.7,
            requires_msdk_v5=True,
        ),
    ),

    # -----------------------------------------------------------------------
    # DJI Mavic 3 Classic
    # -----------------------------------------------------------------------
    # Sensor: 4/3 CMOS (Hasselblad)  |  17.3 × 13.0 mm
    # Physical focal length: 12.29 mm  (24 mm equivalent)
    # Resolution: 20 MP  (5280 × 3956)
    # MSDK: v5  |  No built-in RTK
    # -----------------------------------------------------------------------
    "mavic3_classic": DroneProfile(
        name="mavic3_classic",
        display_name="DJI Mavic 3 Classic",
        focal_length_mm=12.29,
        sensor_width_mm=17.3,
        sensor_height_mm=13.0,
        image_width_px=5280,
        image_height_px=3956,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=15.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=True,
            has_rtk=False,
            min_trigger_interval_s=0.5,
            requires_msdk_v5=True,
        ),
    ),

    # -----------------------------------------------------------------------
    # DJI Mavic 3 Multispectral
    # -----------------------------------------------------------------------
    # Sensor: 4/3 CMOS (same optics as Mavic 3 Classic) + 4-band MS camera
    # MS sensor: 1/2.8" per band  |  2.08 MP each
    # Use the RGB camera settings for overlap calculations
    # MSDK: v5  |  Optional RTK via D-RTK 2
    # -----------------------------------------------------------------------
    "mavic3_multispectral": DroneProfile(
        name="mavic3_multispectral",
        display_name="DJI Mavic 3 Multispectral",
        focal_length_mm=12.29,
        sensor_width_mm=17.3,
        sensor_height_mm=13.0,
        image_width_px=5280,
        image_height_px=3956,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=15.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=True,
            has_rtk=True,
            min_trigger_interval_s=0.7,
            requires_msdk_v5=True,
        ),
    ),

    # -----------------------------------------------------------------------
    # DJI Phantom 4 RTK
    # -----------------------------------------------------------------------
    # Sensor: 1" CMOS  |  13.2 × 8.8 mm
    # Physical focal length: 8.8 mm  (24 mm equivalent)
    # Resolution: 20 MP  (5472 × 3648)
    # Built-in RTK (D-RTK 2 base or NTRIP)
    # MSDK: v4 and v5 compatible
    # -----------------------------------------------------------------------
    "phantom4_rtk": DroneProfile(
        name="phantom4_rtk",
        display_name="DJI Phantom 4 RTK",
        focal_length_mm=8.8,
        sensor_width_mm=13.2,
        sensor_height_mm=8.8,
        image_width_px=5472,
        image_height_px=3648,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=15.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=False,
            has_rtk=True,
            min_trigger_interval_s=0.5,
            requires_msdk_v5=False,
        ),
    ),

    # -----------------------------------------------------------------------
    # Generic / custom camera
    # -----------------------------------------------------------------------
    "generic": DroneProfile(
        name="generic",
        display_name="Generic / Custom",
        focal_length_mm=24.0,
        sensor_width_mm=13.2,
        sensor_height_mm=8.8,
        image_width_px=4000,
        image_height_px=3000,
        gimbal_pitch_deg=-90.0,
        limits=FlightLimits(
            max_speed_ms=15.0,
            min_altitude_m=2.0,
            max_altitude_m=120.0,
            max_waypoints=65535,
            has_obstacle_sensing=False,
            has_rtk=False,
            min_trigger_interval_s=0.5,
            requires_msdk_v5=False,
        ),
    ),
}


def get_profile(name: str) -> DroneProfile:
    """
    Return a drone profile by name (case-insensitive).

    Args:
        name: Profile key, e.g. "mini3", "mini3_pro", "phantom4_rtk".

    Returns:
        DroneProfile dataclass.

    Raises:
        ValueError: If name is not recognised. Lists available names.
    """
    key = name.lower().replace("-", "_").replace(" ", "_")
    if key not in PROFILES:
        available = ", ".join(sorted(PROFILES.keys()))
        raise ValueError(
            f"Unknown drone profile '{name}'. "
            f"Available profiles: {available}"
        )
    return PROFILES[key]


def list_profiles() -> List[str]:
    """Return sorted list of available profile names."""
    return sorted(PROFILES.keys())


def validate_mission(profile: DroneProfile, altitude_m: float,
                     speed_ms: float,
                     trigger_interval_s: Optional[float] = None) -> ValidationResult:
    """
    Validate proposed mission parameters against a drone profile's limits.

    Returns a ValidationResult with .is_valid, .errors, and .warnings.
    Always call this before generating waypoints and surface any errors
    to the user — the firmware will silently reject out-of-range values.
    """
    errors: List[str]   = []
    warnings: List[str] = []
    lim = profile.limits

    # Speed
    if speed_ms > lim.max_speed_ms:
        errors.append(
            f"Speed {speed_ms} m/s exceeds {profile.display_name} "
            f"maximum of {lim.max_speed_ms} m/s. "
            f"Reduce to {lim.max_speed_ms} m/s or less."
        )

    # Altitude
    if altitude_m < lim.min_altitude_m:
        errors.append(
            f"Altitude {altitude_m} m is below the {profile.display_name} "
            f"minimum of {lim.min_altitude_m} m AGL."
        )
    if altitude_m > lim.max_altitude_m:
        warnings.append(
            f"Altitude {altitude_m} m exceeds the recommended "
            f"{lim.max_altitude_m} m AGL regulatory limit in most regions."
        )

    # Trigger interval
    if trigger_interval_s is not None and trigger_interval_s < lim.min_trigger_interval_s:
        errors.append(
            f"Trigger interval {trigger_interval_s} s is below the "
            f"{profile.display_name} minimum of {lim.min_trigger_interval_s} s."
        )

    # Consumer Mini series — specific advisories
    if profile.name in ("mini3", "mini3_pro", "mini4_pro"):
        if not lim.has_rtk:
            warnings.append(
                f"{profile.display_name} has no built-in RTK. For survey-grade "
                f"accuracy use Ground Control Points (GCPs) or a PPK workflow."
            )
        if lim.requires_msdk_v5:
            warnings.append(
                f"{profile.display_name} requires DJI Mobile SDK v5. "
                f"MSDK v4 is not compatible with this aircraft."
            )
        warnings.append(
            f"{profile.display_name} weighs 249 g. Even sub-250 g drones are "
            f"subject to 120 m AGL limits in most countries — check local rules."
        )

    return ValidationResult(is_valid=len(errors) == 0, errors=errors, warnings=warnings)
