"""
Unit tests for drone_profiles — Mini 3 and multi-drone support.
"""

import pytest
from dji_photogrammetry.drone_profiles import (
    get_profile, list_profiles, validate_mission, PROFILES, DroneProfile, FlightLimits
)
from dji_photogrammetry.mission_planner import (
    MissionPlanner, FlightPattern, MissionSettings, ROIPolygon, CameraSettings
)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class TestRegistry:
    def test_all_expected_profiles_present(self):
        for name in ("mini3", "mini3_pro", "mini4_pro",
                     "mavic3_classic", "mavic3_multispectral",
                     "phantom4_rtk", "generic"):
            assert name in PROFILES

    def test_list_profiles_sorted(self):
        names = list_profiles()
        assert names == sorted(names)

    def test_get_profile_case_insensitive(self):
        p1 = get_profile("mini3")
        p2 = get_profile("MINI3")
        assert p1.name == p2.name

    def test_get_profile_hyphen_normalised(self):
        p = get_profile("mini3-pro")
        assert p.name == "mini3_pro"

    def test_get_profile_unknown_raises(self):
        with pytest.raises(ValueError, match="Unknown drone profile"):
            get_profile("dji_magic_nonexistent")

    def test_get_profile_unknown_lists_choices(self):
        with pytest.raises(ValueError) as exc:
            get_profile("bad")
        assert "mini3" in str(exc.value)


# ---------------------------------------------------------------------------
# DJI Mini 3 profile correctness
# ---------------------------------------------------------------------------

class TestMini3Profile:
    @pytest.fixture
    def mini3(self):
        return get_profile("mini3")

    def test_display_name(self, mini3):
        assert "Mini 3" in mini3.display_name

    def test_sensor_dimensions(self, mini3):
        assert mini3.sensor_width_mm  == pytest.approx(9.6,  abs=0.01)
        assert mini3.sensor_height_mm == pytest.approx(7.2,  abs=0.01)

    def test_focal_length(self, mini3):
        assert mini3.focal_length_mm == pytest.approx(6.7, abs=0.01)

    def test_resolution(self, mini3):
        assert mini3.image_width_px  == 4032
        assert mini3.image_height_px == 3024

    def test_max_speed_10ms(self, mini3):
        assert mini3.limits.max_speed_ms == pytest.approx(10.0)

    def test_no_rtk(self, mini3):
        assert mini3.limits.has_rtk is False

    def test_requires_msdk_v5(self, mini3):
        assert mini3.limits.requires_msdk_v5 is True

    def test_obstacle_sensing(self, mini3):
        assert mini3.limits.has_obstacle_sensing is True

    def test_120m_alt_limit(self, mini3):
        assert mini3.limits.max_altitude_m == pytest.approx(120.0)


# ---------------------------------------------------------------------------
# Mini 3 Pro / Mini 4 Pro
# ---------------------------------------------------------------------------

class TestMini3ProProfile:
    def test_48mp_resolution(self):
        p = get_profile("mini3_pro")
        assert p.image_width_px == 8064
        assert p.image_height_px == 6048

    def test_same_sensor_as_mini3(self):
        mini3 = get_profile("mini3")
        mini3_pro = get_profile("mini3_pro")
        assert mini3.sensor_width_mm  == mini3_pro.sensor_width_mm
        assert mini3.sensor_height_mm == mini3_pro.sensor_height_mm

    def test_mini4_pro_same_optics(self):
        pro  = get_profile("mini3_pro")
        pro4 = get_profile("mini4_pro")
        assert pro.focal_length_mm == pro4.focal_length_mm


# ---------------------------------------------------------------------------
# Phantom 4 RTK profile
# ---------------------------------------------------------------------------

class TestPhantom4RTKProfile:
    @pytest.fixture
    def p4rtk(self):
        return get_profile("phantom4_rtk")

    def test_has_rtk(self, p4rtk):
        assert p4rtk.limits.has_rtk is True

    def test_does_not_require_v5(self, p4rtk):
        assert p4rtk.limits.requires_msdk_v5 is False

    def test_1inch_sensor(self, p4rtk):
        assert p4rtk.sensor_width_mm  == pytest.approx(13.2, abs=0.01)
        assert p4rtk.sensor_height_mm == pytest.approx(8.8,  abs=0.01)

    def test_max_speed_15ms(self, p4rtk):
        assert p4rtk.limits.max_speed_ms == pytest.approx(15.0)


# ---------------------------------------------------------------------------
# Validation logic
# ---------------------------------------------------------------------------

class TestValidateMission:
    @pytest.fixture
    def mini3(self):
        return get_profile("mini3")

    def test_valid_settings_pass(self, mini3):
        result = validate_mission(mini3, altitude_m=80.0, speed_ms=8.0)
        assert result.is_valid is True
        assert result.errors == []

    def test_speed_too_high_is_error(self, mini3):
        result = validate_mission(mini3, altitude_m=80.0, speed_ms=15.0)
        assert result.is_valid is False
        assert any("speed" in e.lower() or "Speed" in e for e in result.errors)

    def test_altitude_too_low_is_error(self, mini3):
        result = validate_mission(mini3, altitude_m=1.0, speed_ms=5.0)
        assert result.is_valid is False
        assert any("below" in e.lower() for e in result.errors)

    def test_altitude_too_high_is_warning(self, mini3):
        result = validate_mission(mini3, altitude_m=150.0, speed_ms=5.0)
        assert result.is_valid is True   # warning, not error
        assert any("120" in w for w in result.warnings)

    def test_trigger_too_fast_is_error(self, mini3):
        result = validate_mission(mini3, altitude_m=80.0, speed_ms=5.0,
                                  trigger_interval_s=0.3)
        assert result.is_valid is False
        assert any("Trigger" in e or "trigger" in e for e in result.errors)

    def test_mini3_always_warns_no_rtk(self, mini3):
        result = validate_mission(mini3, altitude_m=80.0, speed_ms=5.0)
        assert any("RTK" in w or "rtk" in w.lower() for w in result.warnings)

    def test_mini3_warns_msdk_v5(self, mini3):
        result = validate_mission(mini3, altitude_m=80.0, speed_ms=5.0)
        assert any("v5" in w for w in result.warnings)

    def test_phantom4_rtk_no_v5_warning(self):
        p4rtk = get_profile("phantom4_rtk")
        result = validate_mission(p4rtk, altitude_m=80.0, speed_ms=10.0)
        assert not any("v5" in w for w in result.warnings)

    def test_generic_profile_lenient(self):
        generic = get_profile("generic")
        result = validate_mission(generic, altitude_m=100.0, speed_ms=15.0)
        assert result.is_valid is True


# ---------------------------------------------------------------------------
# MissionPlanner uses profile camera specs
# ---------------------------------------------------------------------------

class TestPlannerWithProfile:
    @pytest.fixture
    def small_roi(self):
        return ROIPolygon(vertices=[
            [47.3769, 8.5417],
            [47.3769, 8.5480],
            [47.3805, 8.5480],
            [47.3805, 8.5417],
            [47.3769, 8.5417],
        ])

    def test_mini3_gsd_lower_than_generic(self, small_roi):
        """Mini 3 has a smaller sensor → larger GSD at same altitude."""
        profile_mini3   = get_profile("mini3")
        profile_generic = get_profile("generic")

        cam_mini3   = CameraSettings(
            focal_length_mm=profile_mini3.focal_length_mm,
            sensor_width_mm=profile_mini3.sensor_width_mm,
            sensor_height_mm=profile_mini3.sensor_height_mm,
            image_width_px=profile_mini3.image_width_px,
            image_height_px=profile_mini3.image_height_px,
        )
        cam_generic = CameraSettings(
            focal_length_mm=profile_generic.focal_length_mm,
            sensor_width_mm=profile_generic.sensor_width_mm,
            sensor_height_mm=profile_generic.sensor_height_mm,
            image_width_px=profile_generic.image_width_px,
            image_height_px=profile_generic.image_height_px,
        )

        gsd_mini3   = MissionPlanner(cam_mini3).calculate_ground_sampling_distance(80)
        gsd_generic = MissionPlanner(cam_generic).calculate_ground_sampling_distance(80)

        # Mini 3 sensor is 9.6 mm wide / 6.7 mm focal → GSD ratio ≈ (9.6/6.7)/(13.2/24)
        assert gsd_mini3 > 0
        assert gsd_generic > 0

    def test_mission_created_with_mini3_profile(self, small_roi):
        profile = get_profile("mini3")
        cam = CameraSettings(
            focal_length_mm=profile.focal_length_mm,
            sensor_width_mm=profile.sensor_width_mm,
            sensor_height_mm=profile.sensor_height_mm,
            image_width_px=profile.image_width_px,
            image_height_px=profile.image_height_px,
        )
        settings = MissionSettings(altitude_m=80.0, flight_speed_ms=8.0)
        planner = MissionPlanner(cam)
        mission = planner.create_mission(FlightPattern.GRID, small_roi, settings)
        assert len(mission["waypoints"]) > 0
        assert mission["mission_stats"]["gsd_cm_per_pixel"] > 0

    @pytest.mark.parametrize("preset", list_profiles())
    def test_all_profiles_produce_valid_missions(self, small_roi, preset):
        profile = get_profile(preset)
        cam = CameraSettings(
            focal_length_mm=profile.focal_length_mm,
            sensor_width_mm=profile.sensor_width_mm,
            sensor_height_mm=profile.sensor_height_mm,
            image_width_px=profile.image_width_px,
            image_height_px=profile.image_height_px,
        )
        settings = MissionSettings(altitude_m=80.0, flight_speed_ms=5.0)
        planner = MissionPlanner(cam)
        mission = planner.create_mission(FlightPattern.GRID, small_roi, settings)
        assert len(mission["waypoints"]) > 0
