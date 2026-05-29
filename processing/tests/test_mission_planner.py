"""
Unit tests for MissionPlanner – all flight patterns.
"""

import math
import pytest
from dji_photogrammetry.mission_planner import (
    MissionPlanner, FlightPattern, MissionSettings, ROIPolygon, CameraSettings, Waypoint
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def small_roi():
    """A ~500 m × 400 m rectangular ROI near Zurich."""
    return ROIPolygon(vertices=[
        [47.3769, 8.5417],
        [47.3769, 8.5480],
        [47.3805, 8.5480],
        [47.3805, 8.5417],
        [47.3769, 8.5417],
    ])


@pytest.fixture
def default_settings():
    return MissionSettings(
        altitude_m=100.0,
        front_overlap_pct=80.0,
        side_overlap_pct=70.0,
        flight_speed_ms=15.0
    )


@pytest.fixture
def planner():
    return MissionPlanner(CameraSettings())


# ---------------------------------------------------------------------------
# GSD / coverage helpers
# ---------------------------------------------------------------------------

class TestCalculations:
    def test_gsd_positive(self, planner):
        gsd = planner.calculate_ground_sampling_distance(100)
        assert gsd > 0

    def test_gsd_scales_with_altitude(self, planner):
        gsd_100 = planner.calculate_ground_sampling_distance(100)
        gsd_200 = planner.calculate_ground_sampling_distance(200)
        assert gsd_200 == pytest.approx(gsd_100 * 2, rel=1e-6)

    def test_coverage_dimensions(self, planner):
        w, h = planner.calculate_coverage_dimensions(100)
        assert w > 0 and h > 0

    def test_line_spacing_decreases_with_overlap(self, planner):
        sp50 = planner.calculate_flight_line_spacing(100, 50)
        sp70 = planner.calculate_flight_line_spacing(100, 70)
        assert sp50 > sp70

    def test_photo_interval_positive(self, planner):
        dist, t = planner.calculate_photo_interval(100, 80, 15)
        assert dist > 0 and t > 0


# ---------------------------------------------------------------------------
# Grid pattern
# ---------------------------------------------------------------------------

class TestGridPattern:
    def test_produces_waypoints(self, planner, small_roi, default_settings):
        wps = planner.generate_grid_mission(small_roi, default_settings)
        assert len(wps) >= 2

    def test_waypoints_within_roi_bounds(self, planner, small_roi, default_settings):
        min_lat, min_lon, max_lat, max_lon = small_roi.get_bounds()
        for wp in planner.generate_grid_mission(small_roi, default_settings):
            assert min_lat - 1e-6 <= wp.latitude <= max_lat + 1e-6
            assert min_lon - 1e-6 <= wp.longitude <= max_lon + 1e-6

    def test_alternating_headings(self, planner, small_roi, default_settings):
        wps = planner.generate_grid_mission(small_roi, default_settings)
        headings = {wp.heading_deg for wp in wps}
        assert len(headings) >= 2   # at least two directions

    def test_altitude_matches_settings(self, planner, small_roi, default_settings):
        for wp in planner.generate_grid_mission(small_roi, default_settings):
            assert wp.altitude_m == default_settings.altitude_m


# ---------------------------------------------------------------------------
# Double-grid pattern
# ---------------------------------------------------------------------------

class TestDoubleGridPattern:
    def test_more_waypoints_than_grid(self, planner, small_roi, default_settings):
        grid   = planner.generate_grid_mission(small_roi, default_settings)
        dgrid  = planner.generate_double_grid_mission(small_roi, default_settings)
        assert len(dgrid) > len(grid)


# ---------------------------------------------------------------------------
# Oblique pattern
# ---------------------------------------------------------------------------

class TestObliquePattern:
    def test_camera_angled(self, planner, small_roi, default_settings):
        wps = planner.generate_oblique_mission(small_roi, default_settings)
        for wp in wps:
            assert wp.gimbal_pitch_deg == -45.0

    def test_closed_loop(self, planner, small_roi, default_settings):
        wps = planner.generate_oblique_mission(small_roi, default_settings)
        assert len(wps) >= 2
        assert (wps[0].latitude, wps[0].longitude) == (wps[-1].latitude, wps[-1].longitude)


# ---------------------------------------------------------------------------
# Perimeter pattern
# ---------------------------------------------------------------------------

class TestPerimeterPattern:
    def test_produces_waypoints(self, planner, small_roi, default_settings):
        wps = planner.generate_perimeter_mission(small_roi, default_settings)
        assert len(wps) >= 2

    def test_camera_inward(self, planner, small_roi, default_settings):
        wps = planner.generate_perimeter_mission(small_roi, default_settings)
        for wp in wps:
            assert wp.gimbal_pitch_deg == -45.0

    def test_closed_loop(self, planner, small_roi, default_settings):
        wps = planner.generate_perimeter_mission(small_roi, default_settings)
        assert (wps[0].latitude, wps[0].longitude) == (wps[-1].latitude, wps[-1].longitude)

    def test_headings_toward_centre(self, planner, small_roi, default_settings):
        min_lat, min_lon, max_lat, max_lon = small_roi.get_bounds()
        centre_lat = (min_lat + max_lat) / 2
        centre_lon = (min_lon + max_lon) / 2
        wps = planner.generate_perimeter_mission(small_roi, default_settings)
        # Spot-check: each heading should generally point inward
        for wp in wps[:-1]:   # skip closing duplicate
            dlat = centre_lat - wp.latitude
            dlon = centre_lon - wp.longitude
            expected_heading = math.degrees(math.atan2(dlon, dlat)) % 360
            # Allow ±5° tolerance
            diff = abs(wp.heading_deg - expected_heading)
            diff = min(diff, 360 - diff)
            assert diff < 5.0


# ---------------------------------------------------------------------------
# Spiral pattern
# ---------------------------------------------------------------------------

class TestSpiralPattern:
    def test_produces_waypoints(self, planner, small_roi, default_settings):
        wps = planner.generate_spiral_mission(small_roi, default_settings)
        assert len(wps) > 0

    def test_last_waypoint_is_centre(self, planner, small_roi, default_settings):
        min_lat, min_lon, max_lat, max_lon = small_roi.get_bounds()
        centre_lat = (min_lat + max_lat) / 2
        centre_lon = (min_lon + max_lon) / 2
        wps = planner.generate_spiral_mission(small_roi, default_settings)
        assert wps[-1].latitude  == pytest.approx(centre_lat, abs=1e-6)
        assert wps[-1].longitude == pytest.approx(centre_lon, abs=1e-6)

    def test_altitude_constant(self, planner, small_roi, default_settings):
        for wp in planner.generate_spiral_mission(small_roi, default_settings):
            assert wp.altitude_m == default_settings.altitude_m


# ---------------------------------------------------------------------------
# Terrain-follow pattern
# ---------------------------------------------------------------------------

class TestTerrainFollowPattern:
    def test_flat_terrain_equals_flat_grid(self, planner, small_roi, default_settings):
        flat_grid   = planner.generate_grid_mission(small_roi, default_settings)
        terrain_grid = [[0.0] * 3 for _ in range(3)]   # all zeros = flat
        terrain_wps = planner.generate_terrain_follow_mission(
            small_roi, default_settings, terrain_grid
        )
        assert len(terrain_wps) == len(flat_grid)
        for wt, wf in zip(terrain_wps, flat_grid):
            assert wt.altitude_m == pytest.approx(wf.altitude_m, abs=0.01)

    def test_elevated_terrain_raises_altitude(self, planner, small_roi, default_settings):
        elev = 50.0
        terrain_grid = [[elev] * 3 for _ in range(3)]
        terrain_wps = planner.generate_terrain_follow_mission(
            small_roi, default_settings, terrain_grid
        )
        for wp in terrain_wps:
            assert wp.altitude_m == pytest.approx(default_settings.altitude_m + elev, abs=0.1)

    def test_no_terrain_grid_falls_back(self, planner, small_roi, default_settings):
        flat = planner.generate_grid_mission(small_roi, default_settings)
        tf   = planner.generate_terrain_follow_mission(small_roi, default_settings, None)
        assert len(tf) == len(flat)


# ---------------------------------------------------------------------------
# create_mission factory
# ---------------------------------------------------------------------------

class TestCreateMission:
    @pytest.mark.parametrize("pattern", [
        FlightPattern.GRID,
        FlightPattern.DOUBLE_GRID,
        FlightPattern.OBLIQUE,
        FlightPattern.PERIMETER,
        FlightPattern.SPIRAL,
        FlightPattern.TERRAIN_FOLLOW,
    ])
    def test_all_patterns_return_dict(self, planner, small_roi, default_settings, pattern):
        result = planner.create_mission(pattern, small_roi, default_settings)
        assert isinstance(result, dict)
        assert 'waypoints' in result
        assert 'mission_stats' in result

    def test_unknown_pattern_raises(self, planner, small_roi, default_settings):
        with pytest.raises((ValueError, AttributeError)):
            planner.create_mission("bad_pattern", small_roi, default_settings)

    def test_stats_populated(self, planner, small_roi, default_settings):
        result = planner.create_mission(FlightPattern.GRID, small_roi, default_settings)
        stats = result['mission_stats']
        assert stats['total_waypoints'] > 0
        assert stats['gsd_cm_per_pixel'] > 0
        assert stats['coverage_area_m2'] > 0


# ---------------------------------------------------------------------------
# Densify polygon helper
# ---------------------------------------------------------------------------

class TestDensifyPolygon:
    def test_more_points_than_original(self, planner, small_roi):
        dense = planner._densify_polygon(small_roi.vertices, max_segment_m=50)
        assert len(dense) >= len(small_roi.vertices)

    def test_empty_polygon(self, planner):
        assert planner._densify_polygon([], 50) == []
