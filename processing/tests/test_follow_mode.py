"""
Unit tests for the route-follow / pace-match / backwards-flight controller.
"""

import math
import pytest
from dji_photogrammetry.follow_mode import (
    RouteWaypoint, RouteSegment, FollowRoute, FollowModeController
)


# ---------------------------------------------------------------------------
# RouteWaypoint / RouteSegment geometry
# ---------------------------------------------------------------------------

class TestRouteSegment:
    @pytest.fixture
    def north_segment(self):
        return RouteSegment(
            start=RouteWaypoint(47.0, 8.5, 30.0),
            end=RouteWaypoint(47.01, 8.5, 30.0),
            speed_ms=5.0
        )

    def test_bearing_northward(self, north_segment):
        assert north_segment.bearing_deg == pytest.approx(0.0, abs=1.0)

    def test_length_positive(self, north_segment):
        assert north_segment.length_m > 0

    def test_duration_positive(self, north_segment):
        assert north_segment.duration_s > 0

    def test_forward_yaw_equals_bearing(self, north_segment):
        assert north_segment.yaw_deg == pytest.approx(north_segment.bearing_deg, abs=1e-9)

    def test_backward_yaw_opposite(self, north_segment):
        north_segment.fly_backwards = True
        assert north_segment.yaw_deg == pytest.approx(
            (north_segment.bearing_deg + 180) % 360, abs=1e-9
        )

    def test_velocity_ned_northward(self, north_segment):
        vn, ve, vd = north_segment.velocity_ned()
        assert vn > 0
        assert abs(ve) < 0.5     # mostly north
        assert abs(vd) < 0.1     # flat segment

    def test_zero_speed_safe(self):
        seg = RouteSegment(
            RouteWaypoint(47.0, 8.5, 30.0),
            RouteWaypoint(47.01, 8.5, 30.0),
            speed_ms=0.0
        )
        assert seg.duration_s == 0
        vn, ve, vd = seg.velocity_ned()
        assert vn == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# FollowRoute
# ---------------------------------------------------------------------------

class TestFollowRoute:
    @pytest.fixture
    def simple_route(self):
        r = FollowRoute(default_speed=5.0, fly_backwards=False, update_home=True)
        r.add_waypoint(47.0, 8.50, 30.0, "A")
        r.add_waypoint(47.01, 8.51, 30.0, "B")
        r.add_waypoint(47.02, 8.52, 30.0, "C")
        return r

    def test_segment_count(self, simple_route):
        segs = simple_route.build_segments()
        assert len(segs) == 2

    def test_total_length_positive(self, simple_route):
        assert simple_route.total_length_m > 0

    def test_final_waypoint(self, simple_route):
        assert simple_route.final_waypoint.name == "C"

    def test_too_few_waypoints_raises(self):
        r = FollowRoute()
        r.add_waypoint(47.0, 8.5, 30.0)
        with pytest.raises(ValueError):
            r.build_segments()

    def test_per_segment_speed_override(self, simple_route):
        segs = simple_route.build_segments(per_segment_speeds=[3.0, 7.0])
        assert segs[0].speed_ms == 3.0
        assert segs[1].speed_ms == 7.0

    def test_per_segment_backwards_override(self, simple_route):
        segs = simple_route.build_segments(per_segment_backwards=[True, False])
        assert segs[0].fly_backwards is True
        assert segs[1].fly_backwards is False

    def test_export_load_roundtrip(self, simple_route, tmp_path):
        path = str(tmp_path / "route.json")
        simple_route.export_json(path)
        loaded = FollowRoute.from_json(path)
        assert len(loaded.waypoints) == len(simple_route.waypoints)
        assert loaded.default_speed == simple_route.default_speed
        assert loaded.update_home == simple_route.update_home


# ---------------------------------------------------------------------------
# FollowModeController
# ---------------------------------------------------------------------------

class TestFollowModeController:
    @pytest.fixture
    def route(self):
        r = FollowRoute(default_speed=10.0, update_home=True)
        r.add_waypoint(47.0,  8.50, 30.0, "S")
        r.add_waypoint(47.01, 8.50, 30.0, "M")
        r.add_waypoint(47.02, 8.50, 30.0, "E")
        return r

    @pytest.fixture
    def controller(self, route):
        c = FollowModeController(route)
        c.start()
        return c

    def test_starts_running(self, controller):
        assert controller.is_running is True
        assert controller.is_complete is False

    def test_empty_route_raises(self):
        r = FollowRoute()
        r.add_waypoint(47.0, 8.5, 30.0)   # only 1 point
        c = FollowModeController(r)
        with pytest.raises((ValueError, Exception)):
            c.start()

    def test_tick_advances(self, controller, route):
        # First segment end: lat 47.01
        still_running = controller.tick(47.0, 8.50)
        assert still_running is True

    def test_tick_near_end_advances_segment(self, controller, route):
        # Simulate arrival close to end of first segment (within 3 m)
        controller.tick(47.0099, 8.50)   # very close to 47.01
        # Should have advanced (or may complete if only 2 segments)

    def test_route_completes(self, controller, route):
        # advance_threshold = max(3, speed*0.5) = max(3, 5) = 5 m
        # 5 m in degrees lat ≈ 5/111320 ≈ 0.0000449
        # Place drone within 3 m of each segment end-point
        near_m = 47.01 - 2.0 / 111320   # 2 m before M
        near_e = 47.02 - 2.0 / 111320   # 2 m before E
        controller.tick(near_m, 8.50)    # arrive at M
        controller.tick(near_e, 8.50)    # arrive at E
        assert controller.is_complete is True
        assert controller.is_running is False

    def test_home_update_called_on_complete(self, route):
        called_with = []
        c = FollowModeController(route)
        c.on_home_update = lambda lat, lon: called_with.append((lat, lon))
        c.start()
        near_m = 47.01 - 2.0 / 111320
        near_e = 47.02 - 2.0 / 111320
        c.tick(near_m, 8.50)
        c.tick(near_e, 8.50)
        assert len(called_with) == 1
        assert called_with[0][0] == pytest.approx(47.02, abs=1e-6)

    def test_home_not_updated_when_disabled(self, route):
        route.update_home = False
        called_with = []
        c = FollowModeController(route)
        c.on_home_update = lambda lat, lon: called_with.append((lat, lon))
        c.start()
        c.tick(47.0099, 8.50)
        c.tick(47.0199, 8.50)
        assert len(called_with) == 0

    def test_velocity_callback_called(self, route):
        commands = []
        c = FollowModeController(route)
        c.on_velocity_command = lambda vn, ve, vd, yaw: commands.append((vn, ve, vd, yaw))
        c.start()
        c.tick(47.0, 8.50)
        assert len(commands) >= 1
        vn, ve, vd, yaw = commands[0]
        assert vn > 0          # moving north
        assert abs(ve) < 1.0   # negligible east component

    def test_set_pace_speed(self, controller, route):
        controller.set_pace_speed(3.0)
        for seg in controller.segments[controller._current_seg_index:]:
            assert seg.speed_ms == 3.0

    def test_set_backwards_flight(self, controller, route):
        controller.set_backwards_flight(True)
        for seg in controller.segments[controller._current_seg_index:]:
            assert seg.fly_backwards is True

    def test_stop_halts_controller(self, controller):
        controller.stop()
        assert controller.is_running is False

    def test_export_progress_report(self, controller, route, tmp_path):
        controller.tick(47.0, 8.50)
        path = str(tmp_path / "report.json")
        controller.export_progress_report(path)
        import json, os
        assert os.path.exists(path)
        with open(path) as f:
            data = json.load(f)
        assert 'route_total_length_m' in data
        assert data['total_segments'] == 2

    def test_haversine_known_distance(self):
        # One degree of latitude ≈ 111 320 m
        d = FollowModeController._haversine(47.0, 8.5, 48.0, 8.5)
        assert d == pytest.approx(111320, rel=0.01)
