"""
Route-follow / active-track mission controller.

Allows you to:
  1. Define a GPS route (list of waypoints with optional speed per segment).
  2. Match the drone's speed to your running or riding pace – the drone
     follows you along the predefined route at exactly that speed.
  3. Optionally fly the drone backwards (tail facing direction of travel).
  4. Update the home-point to the final waypoint of the route so RTH
     lands at the end of your run/ride, not the take-off point.

On Android the DJI Virtual Stick API is used for real-time velocity
commands.  This Python module models the same logic for simulation,
testing, and pre-flight planning.

Key classes
-----------
RouteSegment   – one straight leg of the route with speed and direction
FollowRoute    – holds the full route and produces per-segment commands
FollowModeController – stateful controller that streams velocity commands
"""

import math
import json
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import List, Optional, Dict, Any, Tuple, Callable


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class RouteWaypoint:
    """Single point along the follow route."""
    latitude: float
    longitude: float
    altitude_m: float
    name: str = ""


@dataclass
class RouteSegment:
    """
    One straight leg between two consecutive route waypoints.

    speed_ms:       Target ground speed for this leg (m/s).
    fly_backwards:  If True the drone's nose points *opposite* the travel
                    direction so the camera faces forward while the drone
                    moves backward relative to its nose heading.
    """
    start: RouteWaypoint
    end: RouteWaypoint
    speed_ms: float = 5.0
    fly_backwards: bool = False

    @property
    def bearing_deg(self) -> float:
        """True bearing from start to end (0 = North, clockwise)."""
        lat1 = math.radians(self.start.latitude)
        lat2 = math.radians(self.end.latitude)
        dlon = math.radians(self.end.longitude - self.start.longitude)
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    @property
    def length_m(self) -> float:
        """Haversine distance between start and end in metres."""
        lat1 = math.radians(self.start.latitude)
        lon1 = math.radians(self.start.longitude)
        lat2 = math.radians(self.end.latitude)
        lon2 = math.radians(self.end.longitude)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        return 6371000 * 2 * math.asin(math.sqrt(a))

    @property
    def duration_s(self) -> float:
        return self.length_m / self.speed_ms if self.speed_ms > 0 else 0

    @property
    def yaw_deg(self) -> float:
        """
        Drone nose heading.
        Forward mode: nose points along bearing.
        Backward mode: nose points 180° opposite (drone reverses).
        """
        if self.fly_backwards:
            return (self.bearing_deg + 180) % 360
        return self.bearing_deg

    def velocity_ned(self) -> Tuple[float, float, float]:
        """
        Velocity in North-East-Down frame (m/s).

        Returns (vn, ve, vd) where vd is positive downward.
        The vertical component is computed from altitude change / duration.
        """
        bearing_rad = math.radians(self.bearing_deg)
        vn = self.speed_ms * math.cos(bearing_rad)
        ve = self.speed_ms * math.sin(bearing_rad)

        dalt = self.end.altitude_m - self.start.altitude_m
        vd = -dalt / self.duration_s if self.duration_s > 0 else 0  # negative = climb

        return vn, ve, vd


@dataclass
class FollowRoute:
    """
    Complete follow-me route.

    Attributes
    ----------
    waypoints:      Ordered list of GPS waypoints along the route.
    default_speed:  Speed used when a segment does not override it (m/s).
    fly_backwards:  Global backwards-flight switch (overridable per segment).
    update_home:    If True, update the drone homepoint to the last waypoint
                    when the route completes.
    """
    waypoints: List[RouteWaypoint] = field(default_factory=list)
    default_speed: float = 5.0
    fly_backwards: bool = False
    update_home: bool = True

    def add_waypoint(self, lat: float, lon: float, alt: float, name: str = ""):
        self.waypoints.append(RouteWaypoint(lat, lon, alt, name))

    def build_segments(self,
                       per_segment_speeds: Optional[List[float]] = None,
                       per_segment_backwards: Optional[List[bool]] = None
                       ) -> List[RouteSegment]:
        """
        Generate RouteSegment list from waypoints.

        Args:
            per_segment_speeds:    Override speed for each leg (len = waypoints-1).
            per_segment_backwards: Override backwards flag per leg.
        """
        if len(self.waypoints) < 2:
            raise ValueError("Need at least 2 waypoints to build segments")

        segments = []
        for i in range(len(self.waypoints) - 1):
            speed = self.default_speed
            if per_segment_speeds and i < len(per_segment_speeds):
                speed = per_segment_speeds[i]

            backwards = self.fly_backwards
            if per_segment_backwards and i < len(per_segment_backwards):
                backwards = per_segment_backwards[i]

            seg = RouteSegment(
                start=self.waypoints[i],
                end=self.waypoints[i + 1],
                speed_ms=speed,
                fly_backwards=backwards
            )
            segments.append(seg)

        return segments

    @property
    def total_length_m(self) -> float:
        segs = self.build_segments()
        return sum(s.length_m for s in segs)

    @property
    def final_waypoint(self) -> Optional[RouteWaypoint]:
        return self.waypoints[-1] if self.waypoints else None

    def export_json(self, path: str):
        data = {
            'default_speed_ms': self.default_speed,
            'fly_backwards':    self.fly_backwards,
            'update_home':      self.update_home,
            'waypoints': [asdict(wp) for wp in self.waypoints]
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    @classmethod
    def from_json(cls, path: str) -> 'FollowRoute':
        with open(path, 'r') as f:
            data = json.load(f)
        route = cls(
            default_speed=data.get('default_speed_ms', 5.0),
            fly_backwards=data.get('fly_backwards', False),
            update_home=data.get('update_home', True)
        )
        for wp in data.get('waypoints', []):
            route.add_waypoint(wp['latitude'], wp['longitude'],
                               wp['altitude_m'], wp.get('name', ''))
        return route


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class FollowModeController:
    """
    Stateful follow-route controller.

    In a real DJI app the on_velocity_command callback sends
    FlightController.sendVirtualStickFlightControlData() and
    on_home_update calls FlightController.setHomeLocation().

    In simulation / desktop testing these callbacks just print the commands.

    Usage
    -----
    controller = FollowModeController(route)
    controller.on_velocity_command = lambda vn, ve, vd, yaw: dji_set_velocity(...)
    controller.on_home_update = lambda lat, lon: dji_set_home(lat, lon)
    controller.start()

    # From your GPS update loop (call ~10 Hz):
    while controller.is_running:
        controller.tick(current_lat, current_lon)
        time.sleep(0.1)
    """

    def __init__(self, route: FollowRoute):
        self.route = route
        self.segments: List[RouteSegment] = []
        self._current_seg_index = 0
        self._segment_start_time: Optional[float] = None
        self.is_running = False
        self.is_complete = False

        # Callbacks to wire into DJI SDK
        self.on_velocity_command: Optional[Callable[[float, float, float, float], None]] = None
        self.on_home_update: Optional[Callable[[float, float], None]] = None

        # Progress tracking
        self.progress_log: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------
    # Life-cycle
    # ------------------------------------------------------------------

    def start(self):
        """Build segments and begin the route."""
        self.segments = self.route.build_segments()
        if not self.segments:
            raise ValueError("Route has no segments (need ≥ 2 waypoints)")
        self._current_seg_index = 0
        self._segment_start_time = time.monotonic()
        self.is_running = True
        self.is_complete = False
        print(f"Follow-mode started: {len(self.segments)} segments, "
              f"{self.route.total_length_m:.1f} m total")

    def stop(self):
        """Halt the controller immediately."""
        self.is_running = False
        self._send_hover()
        print("Follow-mode stopped")

    # ------------------------------------------------------------------
    # Main loop – call at ~10 Hz from GPS update
    # ------------------------------------------------------------------

    def tick(self, current_lat: float, current_lon: float) -> bool:
        """
        Advance route execution.

        Computes the velocity command for the current segment and checks
        whether to advance to the next segment based on elapsed time
        (time-based advancement) or proximity to the segment end
        (position-based advancement).

        Args:
            current_lat: Drone's current latitude.
            current_lon: Drone's current longitude.

        Returns:
            True if the route is still running, False if complete.
        """
        if not self.is_running or self.is_complete:
            return False

        seg = self.segments[self._current_seg_index]
        dist_to_end = self._haversine(current_lat, current_lon,
                                      seg.end.latitude, seg.end.longitude)

        # Advance to next segment when within 3 m of end-point
        advance_threshold_m = max(3.0, seg.speed_ms * 0.5)
        if dist_to_end <= advance_threshold_m:
            self._advance_segment()
            return self.is_running

        # Send velocity command for current segment
        vn, ve, vd = seg.velocity_ned()
        yaw = seg.yaw_deg
        self._send_velocity(vn, ve, vd, yaw)

        # Log progress
        self.progress_log.append({
            'timestamp':    datetime.now().isoformat(),
            'segment':      self._current_seg_index,
            'lat':          current_lat,
            'lon':          current_lon,
            'dist_to_end_m': dist_to_end,
            'speed_ms':     seg.speed_ms,
            'fly_backwards': seg.fly_backwards,
            'yaw_deg':      yaw
        })

        return True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _advance_segment(self):
        """Move to next segment or finish the route."""
        self._current_seg_index += 1

        if self._current_seg_index >= len(self.segments):
            self._finish()
        else:
            seg = self.segments[self._current_seg_index]
            print(f"  → segment {self._current_seg_index}/{len(self.segments)}: "
                  f"{seg.length_m:.0f} m @ {seg.speed_ms:.1f} m/s "
                  f"{'(BACKWARDS)' if seg.fly_backwards else ''}")

    def _finish(self):
        """Called when the last segment is complete."""
        self.is_running = False
        self.is_complete = True
        self._send_hover()

        if self.route.update_home and self.route.final_waypoint:
            wp = self.route.final_waypoint
            print(f"Updating home-point to final waypoint: "
                  f"{wp.latitude:.6f}, {wp.longitude:.6f}")
            if self.on_home_update:
                self.on_home_update(wp.latitude, wp.longitude)

        print("Follow-mode route complete")

    def _send_velocity(self, vn: float, ve: float, vd: float, yaw: float):
        if self.on_velocity_command:
            self.on_velocity_command(vn, ve, vd, yaw)
        else:
            print(f"VEL vN={vn:.2f} vE={ve:.2f} vD={vd:.2f} yaw={yaw:.1f}°")

    def _send_hover(self):
        if self.on_velocity_command:
            self.on_velocity_command(0.0, 0.0, 0.0,
                                     self.segments[min(self._current_seg_index,
                                                       len(self.segments) - 1)].yaw_deg)
        else:
            print("VEL hover (0, 0, 0)")

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distance in metres between two WGS-84 points."""
        r = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
        return r * 2 * math.asin(math.sqrt(a))

    # ------------------------------------------------------------------
    # Convenience: match speed to user pace
    # ------------------------------------------------------------------

    def set_pace_speed(self, pace_ms: float):
        """
        Dynamically adjust all remaining segment speeds to match the
        user's current running / riding pace.

        Args:
            pace_ms: Current user speed in m/s (e.g. 3.5 for a fast walk,
                     6.0 for a jog, 10+ for cycling).
        """
        for i in range(self._current_seg_index, len(self.segments)):
            self.segments[i].speed_ms = pace_ms
        print(f"Pace updated to {pace_ms:.1f} m/s "
              f"({pace_ms * 3.6:.1f} km/h) for remaining {len(self.segments) - self._current_seg_index} segments")

    def set_backwards_flight(self, enabled: bool):
        """Toggle backwards flight for all remaining segments."""
        for i in range(self._current_seg_index, len(self.segments)):
            self.segments[i].fly_backwards = enabled
        direction = "BACKWARDS" if enabled else "FORWARD"
        print(f"Flight direction set to {direction} for remaining segments")

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------

    def export_progress_report(self, output_path: str):
        """Dump the progress log to JSON."""
        data = {
            'route_total_length_m': self.route.total_length_m,
            'segments_completed':   self._current_seg_index,
            'total_segments':       len(self.segments),
            'route_complete':       self.is_complete,
            'home_updated':         self.route.update_home and self.is_complete,
            'final_waypoint': asdict(self.route.final_waypoint) if self.route.final_waypoint else None,
            'log': self.progress_log
        }
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Progress report saved to {output_path}")
