"""
Mission planner for DJI drone photogrammetry surveys.

Supports grid, double-grid, and oblique flight patterns with configurable
altitude, overlap percentages, camera settings, and ROI polygons.
"""

import math
from enum import Enum
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass


class FlightPattern(Enum):
    """Supported flight patterns for photogrammetry missions."""
    GRID = "grid"
    DOUBLE_GRID = "double_grid"
    OBLIQUE = "oblique"
    PERIMETER = "perimeter"
    SPIRAL = "spiral"
    TERRAIN_FOLLOW = "terrain_follow"


@dataclass
class CameraSettings:
    """Camera configuration for mission planning."""
    focal_length_mm: float = 24.0  # Camera focal length in mm
    sensor_width_mm: float = 13.2  # Sensor width in mm
    sensor_height_mm: float = 8.8  # Sensor height in mm
    image_width_px: int = 4000  # Image width in pixels
    image_height_px: int = 3000  # Image height in pixels
    gimbal_pitch_deg: float = -90.0  # Downward facing camera


@dataclass
class MissionSettings:
    """Mission configuration parameters."""
    altitude_m: float = 100.0  # Flight altitude in meters AGL
    front_overlap_pct: float = 80.0  # Forward overlap percentage
    side_overlap_pct: float = 70.0  # Side overlap percentage
    flight_speed_ms: float = 15.0  # Flight speed in m/s
    trigger_distance_m: Optional[float] = None  # Distance-based trigger interval
    trigger_time_s: Optional[float] = None  # Time-based trigger interval


@dataclass
class Waypoint:
    """Individual waypoint in flight mission."""
    latitude: float
    longitude: float
    altitude_m: float
    heading_deg: float = 0.0
    gimbal_pitch_deg: float = -90.0
    trigger_camera: bool = True
    fly_backwards: bool = False  # nose opposite travel direction


@dataclass
class ROIPolygon:
    """Region of Interest polygon for mission area."""
    vertices: List[Tuple[float, float]]  # List of (lat, lon) coordinates
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get bounding box of ROI polygon."""
        lats = [v[0] for v in self.vertices]
        lons = [v[1] for v in self.vertices]
        return min(lats), min(lons), max(lats), max(lons)


class MissionPlanner:
    """
    Mission planner for DJI drone photogrammetry surveys.
    
    Generates flight paths with optimal waypoints for different patterns
    including grid, double-grid, and oblique surveys.
    """
    
    def __init__(self, camera_settings: CameraSettings = None):
        """Initialize mission planner with camera configuration."""
        self.camera_settings = camera_settings or CameraSettings()
    
    def calculate_ground_sampling_distance(self, altitude_m: float) -> float:
        """
        Calculate Ground Sampling Distance (GSD) for given altitude.
        
        Args:
            altitude_m: Flight altitude in meters
            
        Returns:
            GSD in cm/pixel
        """
        # GSD = (altitude * sensor_width) / (focal_length * image_width)
        gsd_m = (altitude_m * (self.camera_settings.sensor_width_mm / 1000)) / (
            (self.camera_settings.focal_length_mm / 1000) * self.camera_settings.image_width_px
        )
        return gsd_m * 100  # Convert to cm/pixel
    
    def calculate_coverage_dimensions(self, altitude_m: float) -> Tuple[float, float]:
        """
        Calculate ground coverage dimensions for single image.
        
        Args:
            altitude_m: Flight altitude in meters
            
        Returns:
            Tuple of (width_m, height_m) ground coverage
        """
        gsd_m = self.calculate_ground_sampling_distance(altitude_m) / 100  # cm to m
        width_m = gsd_m * self.camera_settings.image_width_px
        height_m = gsd_m * self.camera_settings.image_height_px
        return width_m, height_m
    
    def calculate_flight_line_spacing(self, altitude_m: float, side_overlap_pct: float) -> float:
        """Calculate spacing between parallel flight lines."""
        coverage_width, _ = self.calculate_coverage_dimensions(altitude_m)
        effective_width = coverage_width * (1 - side_overlap_pct / 100)
        return effective_width
    
    def calculate_photo_interval(self, altitude_m: float, front_overlap_pct: float, 
                               flight_speed_ms: float) -> Tuple[float, float]:
        """
        Calculate photo capture interval for distance and time-based triggers.
        
        Returns:
            Tuple of (distance_interval_m, time_interval_s)
        """
        _, coverage_height = self.calculate_coverage_dimensions(altitude_m)
        effective_distance = coverage_height * (1 - front_overlap_pct / 100)
        time_interval = effective_distance / flight_speed_ms
        return effective_distance, time_interval
    
    def generate_grid_mission(self, roi: ROIPolygon, settings: MissionSettings) -> List[Waypoint]:
        """
        Generate grid pattern mission waypoints.
        
        Args:
            roi: Region of Interest polygon
            settings: Mission configuration
            
        Returns:
            List of waypoints for grid pattern
        """
        waypoints = []
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        
        # Calculate flight line spacing
        line_spacing = self.calculate_flight_line_spacing(
            settings.altitude_m, settings.side_overlap_pct
        )
        
        # Convert to approximate meters (rough approximation for mission planning)
        lat_per_m = 1 / 111320  # degrees per meter latitude
        lon_per_m = 1 / (111320 * math.cos(math.radians((min_lat + max_lat) / 2)))
        
        # Generate parallel flight lines
        current_lat = min_lat
        line_direction = 1  # 1 for west->east, -1 for east->west
        
        while current_lat <= max_lat:
            if line_direction == 1:
                # West to East
                start_waypoint = Waypoint(
                    latitude=current_lat,
                    longitude=min_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=90.0,  # East
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
                end_waypoint = Waypoint(
                    latitude=current_lat,
                    longitude=max_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=90.0,
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
            else:
                # East to West
                start_waypoint = Waypoint(
                    latitude=current_lat,
                    longitude=max_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=270.0,  # West
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
                end_waypoint = Waypoint(
                    latitude=current_lat,
                    longitude=min_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=270.0,
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
            
            waypoints.extend([start_waypoint, end_waypoint])
            
            # Move to next line
            current_lat += line_spacing * lat_per_m
            line_direction *= -1  # Alternate direction
        
        return waypoints
    
    def generate_double_grid_mission(self, roi: ROIPolygon, settings: MissionSettings) -> List[Waypoint]:
        """
        Generate double grid (cross-hatch) pattern mission.
        
        Flies the area twice with 90-degree rotated patterns for better coverage.
        """
        # First grid (North-South lines)
        grid1_waypoints = self.generate_grid_mission(roi, settings)
        
        # Second grid (East-West lines) - rotate the ROI coordinates
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        
        # Calculate flight line spacing
        line_spacing = self.calculate_flight_line_spacing(
            settings.altitude_m, settings.side_overlap_pct
        )
        
        lat_per_m = 1 / 111320
        lon_per_m = 1 / (111320 * math.cos(math.radians((min_lat + max_lat) / 2)))
        
        grid2_waypoints = []
        current_lon = min_lon
        line_direction = 1
        
        while current_lon <= max_lon:
            if line_direction == 1:
                # South to North
                start_waypoint = Waypoint(
                    latitude=min_lat,
                    longitude=current_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=0.0,  # North
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
                end_waypoint = Waypoint(
                    latitude=max_lat,
                    longitude=current_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=0.0,
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
            else:
                # North to South
                start_waypoint = Waypoint(
                    latitude=max_lat,
                    longitude=current_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=180.0,  # South
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
                end_waypoint = Waypoint(
                    latitude=min_lat,
                    longitude=current_lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=180.0,
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg
                )
            
            grid2_waypoints.extend([start_waypoint, end_waypoint])
            
            current_lon += line_spacing * lon_per_m
            line_direction *= -1
        
        return grid1_waypoints + grid2_waypoints
    
    def generate_oblique_mission(self, roi: ROIPolygon, settings: MissionSettings) -> List[Waypoint]:
        """
        Generate oblique imagery mission with angled camera.
        
        Flies around the perimeter of ROI with camera angled toward center.
        """
        waypoints = []
        
        # Calculate center point of ROI
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2
        
        # Create waypoints around perimeter
        for i, vertex in enumerate(roi.vertices):
            lat, lon = vertex
            
            # Calculate heading toward center
            dlat = center_lat - lat
            dlon = center_lon - lon
            heading_deg = math.degrees(math.atan2(dlon, dlat))
            if heading_deg < 0:
                heading_deg += 360
            
            waypoint = Waypoint(
                latitude=lat,
                longitude=lon,
                altitude_m=settings.altitude_m,
                heading_deg=heading_deg,
                gimbal_pitch_deg=-45.0  # 45-degree oblique angle
            )
            waypoints.append(waypoint)
        
        # Close the loop by returning to first waypoint
        if waypoints:
            waypoints.append(waypoints[0])
        
        return waypoints
    
    def create_mission(self, pattern: FlightPattern, roi: ROIPolygon, 
                      settings: MissionSettings) -> Dict[str, Any]:
        """
        Create complete mission with specified pattern and settings.
        
        Returns:
            Mission dictionary with waypoints and metadata
        """
        if pattern == FlightPattern.GRID:
            waypoints = self.generate_grid_mission(roi, settings)
        elif pattern == FlightPattern.DOUBLE_GRID:
            waypoints = self.generate_double_grid_mission(roi, settings)
        elif pattern == FlightPattern.OBLIQUE:
            waypoints = self.generate_oblique_mission(roi, settings)
        elif pattern == FlightPattern.PERIMETER:
            waypoints = self.generate_perimeter_mission(roi, settings)
        elif pattern == FlightPattern.SPIRAL:
            waypoints = self.generate_spiral_mission(roi, settings)
        elif pattern == FlightPattern.TERRAIN_FOLLOW:
            waypoints = self.generate_terrain_follow_mission(roi, settings)
        else:
            raise ValueError(f"Unsupported flight pattern: {pattern}")
        
        # Calculate photo intervals
        distance_interval, time_interval = self.calculate_photo_interval(
            settings.altitude_m, settings.front_overlap_pct, settings.flight_speed_ms
        )
        
        # Calculate mission statistics
        gsd = self.calculate_ground_sampling_distance(settings.altitude_m)
        coverage_area = self._calculate_roi_area(roi)
        
        mission_data = {
            "pattern": pattern.value,
            "waypoints": waypoints,
            "settings": settings,
            "camera_settings": self.camera_settings,
            "photo_intervals": {
                "distance_m": distance_interval,
                "time_s": time_interval
            },
            "mission_stats": {
                "total_waypoints": len(waypoints),
                "gsd_cm_per_pixel": gsd,
                "coverage_area_m2": coverage_area,
                "estimated_photos": self._estimate_photo_count(waypoints, distance_interval)
            }
        }
        
        return mission_data
    
    def generate_perimeter_mission(self, roi: ROIPolygon,
                                   settings: MissionSettings) -> List[Waypoint]:
        """
        Perimeter / orbit pattern.

        Flies a closed loop around the ROI boundary at a fixed altitude,
        camera pointing inward at a configurable gimbal pitch so the
        imagery covers all vertical faces of a structure or crop edge.

        Unlike the oblique pattern (which visits only the ROI vertices),
        perimeter densely samples the full boundary at the configured
        front-overlap interval so no gaps appear in the coverage.
        """
        waypoints = []
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2

        # Build a densely-sampled closed polygon from the ROI vertices
        # by interpolating additional points along each edge.
        distance_interval, _ = self.calculate_photo_interval(
            settings.altitude_m, settings.front_overlap_pct, settings.flight_speed_ms
        )
        dense_boundary = self._densify_polygon(roi.vertices, distance_interval)

        for lat, lon in dense_boundary:
            # Heading toward the ROI centre
            dlat = center_lat - lat
            dlon = center_lon - lon
            heading_deg = math.degrees(math.atan2(dlon, dlat))
            if heading_deg < 0:
                heading_deg += 360

            waypoints.append(Waypoint(
                latitude=lat,
                longitude=lon,
                altitude_m=settings.altitude_m,
                heading_deg=heading_deg,
                gimbal_pitch_deg=-45.0,   # angled inward for structure coverage
                trigger_camera=True
            ))

        # Close the loop
        if waypoints:
            waypoints.append(waypoints[0])

        return waypoints

    def generate_spiral_mission(self, roi: ROIPolygon,
                                settings: MissionSettings) -> List[Waypoint]:
        """
        Inward spiral (Archimedean) pattern.

        Starts at the outer boundary of the ROI bounding ellipse and
        spirals inward toward the centre.  Gives 360° coverage at
        every scale, which is especially useful for mapping circular
        fields, round reservoirs, or archaeological mounds.

        The inter-ring spacing equals the flight-line spacing computed
        from the configured side-overlap percentage.
        """
        waypoints = []
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2

        lat_per_m = 1 / 111320
        lon_per_m = 1 / (111320 * math.cos(math.radians(center_lat)))

        ring_spacing = self.calculate_flight_line_spacing(
            settings.altitude_m, settings.side_overlap_pct
        )

        # Outer radius: half the shorter bounding-box dimension
        radius_lat_m = (max_lat - min_lat) / (2 * lat_per_m)
        radius_lon_m = (max_lon - min_lon) / (2 * lon_per_m)
        max_radius_m = min(radius_lat_m, radius_lon_m)

        if max_radius_m < ring_spacing:
            return []   # ROI too small for spiral

        # Number of full rings that fit inside the ROI
        num_rings = int(max_radius_m / ring_spacing)
        points_per_ring = 36   # 10° increments for smooth arcs

        for ring in range(num_rings + 1):
            # Outermost ring first, then inward
            radius_m = max_radius_m - ring * ring_spacing
            if radius_m <= 0:
                break

            for step in range(points_per_ring):
                angle_deg = (step / points_per_ring) * 360
                angle_rad = math.radians(angle_deg)
                lat = center_lat + math.cos(angle_rad) * radius_m * lat_per_m
                lon = center_lon + math.sin(angle_rad) * radius_m * lon_per_m

                # Heading tangent to the spiral
                tangent_deg = (angle_deg + 90) % 360

                waypoints.append(Waypoint(
                    latitude=lat,
                    longitude=lon,
                    altitude_m=settings.altitude_m,
                    heading_deg=tangent_deg,
                    gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg,
                    trigger_camera=True
                ))

        # Add centre point
        waypoints.append(Waypoint(
            latitude=center_lat,
            longitude=center_lon,
            altitude_m=settings.altitude_m,
            heading_deg=0.0,
            gimbal_pitch_deg=self.camera_settings.gimbal_pitch_deg,
            trigger_camera=True
        ))

        return waypoints

    def generate_terrain_follow_mission(self, roi: ROIPolygon,
                                        settings: MissionSettings,
                                        terrain_grid: Optional[List[List[float]]] = None
                                        ) -> List[Waypoint]:
        """
        Terrain-following grid pattern.

        Identical to a regular grid but the altitude at each waypoint is
        adjusted by an elevation offset derived from ``terrain_grid``
        (a 2-D list of elevation values in metres, row-major, SW to NE).

        When no terrain grid is provided the mission falls back to a flat
        grid at the configured altitude.  In production the terrain grid
        would be populated from an SRTM/Copernicus DEM or a previous
        lower-resolution flight.

        Args:
            roi:          Region of interest polygon.
            settings:     Mission settings; altitude_m is used as the
                          AGL (above-ground-level) clearance to maintain.
            terrain_grid: Optional 2-D elevation array.  Shape:
                          (num_rows, num_cols) covering roi bounding box.
        """
        flat_waypoints = self.generate_grid_mission(roi, settings)
        if terrain_grid is None or not terrain_grid:
            return flat_waypoints

        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        num_rows = len(terrain_grid)
        num_cols = len(terrain_grid[0]) if terrain_grid else 0

        adjusted = []
        for wp in flat_waypoints:
            # Bilinear interpolation into the terrain grid
            row_frac = ((wp.latitude - min_lat) / (max_lat - min_lat)) * (num_rows - 1) \
                       if max_lat != min_lat else 0.0
            col_frac = ((wp.longitude - min_lon) / (max_lon - min_lon)) * (num_cols - 1) \
                       if max_lon != min_lon else 0.0

            row_frac = max(0.0, min(row_frac, num_rows - 1))
            col_frac = max(0.0, min(col_frac, num_cols - 1))

            r0 = int(row_frac); r1 = min(r0 + 1, num_rows - 1)
            c0 = int(col_frac); c1 = min(c0 + 1, num_cols - 1)
            dr = row_frac - r0;  dc = col_frac - c0

            elev = (terrain_grid[r0][c0] * (1 - dr) * (1 - dc) +
                    terrain_grid[r1][c0] * dr       * (1 - dc) +
                    terrain_grid[r0][c1] * (1 - dr) * dc       +
                    terrain_grid[r1][c1] * dr       * dc)

            new_wp = Waypoint(
                latitude=wp.latitude,
                longitude=wp.longitude,
                altitude_m=elev + settings.altitude_m,   # terrain + AGL clearance
                heading_deg=wp.heading_deg,
                gimbal_pitch_deg=wp.gimbal_pitch_deg,
                trigger_camera=wp.trigger_camera
            )
            adjusted.append(new_wp)

        return adjusted

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    def _densify_polygon(self, vertices: List[Tuple[float, float]],
                         max_segment_m: float) -> List[Tuple[float, float]]:
        """
        Interpolate extra points along each polygon edge so the spacing
        between consecutive boundary points is at most ``max_segment_m``.
        """
        if not vertices:
            return []

        dense: List[Tuple[float, float]] = []
        n = len(vertices)

        for i in range(n):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % n]

            # Haversine length of this edge
            lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
            lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
            dlat = lat2 - lat1;  dlon = lon2 - lon1
            a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
            edge_m = 6371000 * 2 * math.asin(math.sqrt(max(0.0, min(1.0, a))))

            num_steps = max(1, int(math.ceil(edge_m / max_segment_m)))
            for step in range(num_steps):
                t = step / num_steps
                lat = p1[0] + t * (p2[0] - p1[0])
                lon = p1[1] + t * (p2[1] - p1[1])
                dense.append((lat, lon))

        return dense

    def _calculate_roi_area(self, roi: ROIPolygon) -> float:
        """Calculate approximate area of ROI polygon in square meters."""
        # Simplified area calculation using bounding box
        min_lat, min_lon, max_lat, max_lon = roi.get_bounds()
        
        # Convert to meters (rough approximation)
        lat_diff_m = (max_lat - min_lat) * 111320
        lon_diff_m = (max_lon - min_lon) * 111320 * math.cos(math.radians((min_lat + max_lat) / 2))
        
        return lat_diff_m * lon_diff_m
    
    def _estimate_photo_count(self, waypoints: List[Waypoint], distance_interval: float) -> int:
        """Estimate total number of photos for mission."""
        total_distance = 0
        for i in range(len(waypoints) - 1):
            wp1, wp2 = waypoints[i], waypoints[i + 1]
            # Simplified distance calculation
            dlat = (wp2.latitude - wp1.latitude) * 111320
            dlon = (wp2.longitude - wp1.longitude) * 111320 * math.cos(math.radians(wp1.latitude))
            distance = math.sqrt(dlat**2 + dlon**2)
            total_distance += distance
        
        return int(total_distance / distance_interval) if distance_interval > 0 else 0