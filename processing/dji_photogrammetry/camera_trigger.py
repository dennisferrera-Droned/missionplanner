"""
Camera trigger system for DJI drones with GPS/distance-based automation.

Handles automatic camera triggering based on time intervals, distance traveled,
or GPS coordinates. Logs all triggers with exact GPS position, altitude, 
drone attitude, timestamp, and camera settings.
"""

import time
import json
import math
from enum import Enum
from typing import Dict, Any, List, Optional, Tuple, Callable
from dataclasses import dataclass, asdict
from datetime import datetime


class TriggerMode(Enum):
    """Camera trigger modes."""
    TIME_BASED = "time"
    DISTANCE_BASED = "distance"
    WAYPOINT_BASED = "waypoint"
    MANUAL = "manual"


@dataclass
class DroneState:
    """Current drone state information."""
    latitude: float
    longitude: float
    altitude_m: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    yaw_deg: float
    ground_speed_ms: float
    timestamp: datetime
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary with ISO timestamp."""
        data = asdict(self)
        data['timestamp'] = self.timestamp.isoformat()
        return data


@dataclass
class CameraState:
    """Camera settings and EXIF information."""
    iso: int = 100
    aperture: str = "f/2.8"
    shutter_speed: str = "1/500"
    focal_length_mm: float = 24.0
    white_balance: str = "auto"
    exposure_mode: str = "auto"
    focus_mode: str = "infinity"
    image_format: str = "JPEG"
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class TriggerEvent:
    """Individual camera trigger event with complete metadata."""
    trigger_id: str
    trigger_mode: TriggerMode
    drone_state: DroneState
    camera_state: CameraState
    image_filename: Optional[str] = None
    trigger_reason: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON export."""
        return {
            'trigger_id': self.trigger_id,
            'trigger_mode': self.trigger_mode.value,
            'drone_state': self.drone_state.to_dict(),
            'camera_state': self.camera_state.to_dict(),
            'image_filename': self.image_filename,
            'trigger_reason': self.trigger_reason
        }


class CameraTrigger:
    """
    Camera trigger system for DJI drones.
    
    Manages automatic camera triggering based on configurable intervals
    and logs complete metadata for each capture.
    """
    
    def __init__(self):
        """Initialize camera trigger system."""
        self.trigger_events: List[TriggerEvent] = []
        self.last_trigger_time: Optional[datetime] = None
        self.last_trigger_position: Optional[Tuple[float, float]] = None
        self.trigger_count = 0
        self.is_active = False
        
        # Trigger settings
        self.time_interval_s: Optional[float] = None
        self.distance_interval_m: Optional[float] = None
        self.current_mode = TriggerMode.MANUAL
        
        # Callbacks for DJI SDK integration
        self.on_trigger_callback: Optional[Callable[[], str]] = None
        self.get_drone_state_callback: Optional[Callable[[], DroneState]] = None
        self.get_camera_state_callback: Optional[Callable[[], CameraState]] = None
    
    def set_trigger_mode(self, mode: TriggerMode, **kwargs):
        """
        Set trigger mode and parameters.
        
        Args:
            mode: Trigger mode (TIME_BASED, DISTANCE_BASED, etc.)
            **kwargs: Mode-specific parameters:
                - time_interval_s: for TIME_BASED mode
                - distance_interval_m: for DISTANCE_BASED mode
        """
        self.current_mode = mode
        
        if mode == TriggerMode.TIME_BASED:
            self.time_interval_s = kwargs.get('time_interval_s', 2.0)
            self.distance_interval_m = None
        elif mode == TriggerMode.DISTANCE_BASED:
            self.distance_interval_m = kwargs.get('distance_interval_m', 30.0)
            self.time_interval_s = None
        elif mode == TriggerMode.WAYPOINT_BASED:
            # Trigger only at specific waypoints
            self.time_interval_s = None
            self.distance_interval_m = None
        else:  # MANUAL
            self.time_interval_s = None
            self.distance_interval_m = None
    
    def set_callbacks(self, 
                     on_trigger: Callable[[], str],
                     get_drone_state: Callable[[], DroneState],
                     get_camera_state: Callable[[], CameraState]):
        """
        Set callback functions for DJI SDK integration.
        
        Args:
            on_trigger: Function to trigger camera, returns image filename
            get_drone_state: Function to get current drone state
            get_camera_state: Function to get current camera settings
        """
        self.on_trigger_callback = on_trigger
        self.get_drone_state_callback = get_drone_state
        self.get_camera_state_callback = get_camera_state
    
    def start_triggering(self):
        """Start automatic camera triggering."""
        self.is_active = True
        self.last_trigger_time = None
        self.last_trigger_position = None
        print(f"Camera triggering started in {self.current_mode.value} mode")
    
    def stop_triggering(self):
        """Stop automatic camera triggering."""
        self.is_active = False
        print(f"Camera triggering stopped. Total triggers: {self.trigger_count}")
    
    def update(self) -> bool:
        """
        Update trigger system with current drone state.
        
        Should be called regularly during flight to check trigger conditions.
        
        Returns:
            True if camera was triggered, False otherwise
        """
        if not self.is_active or not self.get_drone_state_callback:
            return False
        
        current_drone_state = self.get_drone_state_callback()
        current_time = datetime.now()
        
        should_trigger = False
        trigger_reason = ""
        
        if self.current_mode == TriggerMode.TIME_BASED:
            should_trigger, trigger_reason = self._check_time_trigger(current_time)
        elif self.current_mode == TriggerMode.DISTANCE_BASED:
            should_trigger, trigger_reason = self._check_distance_trigger(current_drone_state)
        elif self.current_mode == TriggerMode.WAYPOINT_BASED:
            # Waypoint triggering is handled externally
            should_trigger = False
        
        if should_trigger:
            return self.trigger_camera(current_drone_state, trigger_reason)
        
        return False
    
    def _check_time_trigger(self, current_time: datetime) -> Tuple[bool, str]:
        """Check if time-based trigger condition is met."""
        if self.time_interval_s is None:
            return False, ""
        
        if self.last_trigger_time is None:
            return True, f"Initial time-based trigger (interval: {self.time_interval_s}s)"
        
        time_elapsed = (current_time - self.last_trigger_time).total_seconds()
        if time_elapsed >= self.time_interval_s:
            return True, f"Time interval reached ({time_elapsed:.1f}s >= {self.time_interval_s}s)"
        
        return False, ""
    
    def _check_distance_trigger(self, drone_state: DroneState) -> Tuple[bool, str]:
        """Check if distance-based trigger condition is met."""
        if self.distance_interval_m is None:
            return False, ""
        
        current_pos = (drone_state.latitude, drone_state.longitude)
        
        if self.last_trigger_position is None:
            return True, f"Initial distance-based trigger (interval: {self.distance_interval_m}m)"
        
        distance_traveled = self._calculate_distance(
            self.last_trigger_position, current_pos
        )
        
        if distance_traveled >= self.distance_interval_m:
            return True, f"Distance interval reached ({distance_traveled:.1f}m >= {self.distance_interval_m}m)"
        
        return False, ""
    
    def _calculate_distance(self, pos1: Tuple[float, float], 
                          pos2: Tuple[float, float]) -> float:
        """
        Calculate distance between two GPS coordinates using Haversine formula.
        
        Returns:
            Distance in meters
        """
        lat1, lon1 = pos1
        lat2, lon2 = pos2
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = (math.sin(dlat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
        c = 2 * math.asin(math.sqrt(a))
        
        # Earth radius in meters
        earth_radius_m = 6371000
        return earth_radius_m * c
    
    def trigger_camera(self, drone_state: DroneState = None, 
                      reason: str = "Manual trigger") -> bool:
        """
        Trigger camera capture and log metadata.
        
        Args:
            drone_state: Current drone state (if None, will get from callback)
            reason: Reason for trigger (for logging)
            
        Returns:
            True if trigger was successful, False otherwise
        """
        if not self.on_trigger_callback:
            print("Warning: No trigger callback set")
            return False
        
        # Get current states
        if drone_state is None and self.get_drone_state_callback:
            drone_state = self.get_drone_state_callback()
        
        if self.get_camera_state_callback:
            camera_state = self.get_camera_state_callback()
        else:
            camera_state = CameraState()  # Default settings
        
        if drone_state is None:
            print("Warning: No drone state available")
            return False
        
        # Trigger camera
        try:
            image_filename = self.on_trigger_callback()
        except Exception as e:
            print(f"Camera trigger failed: {e}")
            return False
        
        # Create trigger event
        self.trigger_count += 1
        trigger_id = f"trigger_{self.trigger_count:06d}_{int(time.time())}"
        
        trigger_event = TriggerEvent(
            trigger_id=trigger_id,
            trigger_mode=self.current_mode,
            drone_state=drone_state,
            camera_state=camera_state,
            image_filename=image_filename,
            trigger_reason=reason
        )
        
        self.trigger_events.append(trigger_event)
        
        # Update trigger history
        self.last_trigger_time = drone_state.timestamp
        self.last_trigger_position = (drone_state.latitude, drone_state.longitude)
        
        print(f"Camera triggered: {trigger_id} - {reason}")
        return True
    
    def trigger_at_waypoint(self, waypoint_id: str, drone_state: DroneState = None) -> bool:
        """
        Trigger camera at specific waypoint.
        
        Args:
            waypoint_id: Identifier for the waypoint
            drone_state: Current drone state
            
        Returns:
            True if trigger was successful
        """
        reason = f"Waypoint trigger: {waypoint_id}"
        return self.trigger_camera(drone_state, reason)
    
    def export_metadata_csv(self, filename: str):
        """
        Export trigger metadata to CSV file.
        
        Args:
            filename: Output CSV filename
        """
        import pandas as pd
        
        if not self.trigger_events:
            print("No trigger events to export")
            return
        
        # Flatten trigger events for CSV export
        csv_data = []
        for event in self.trigger_events:
            row = {
                'trigger_id': event.trigger_id,
                'trigger_mode': event.trigger_mode.value,
                'image_filename': event.image_filename,
                'trigger_reason': event.trigger_reason,
                
                # Drone state
                'latitude': event.drone_state.latitude,
                'longitude': event.drone_state.longitude,
                'altitude_m': event.drone_state.altitude_m,
                'heading_deg': event.drone_state.heading_deg,
                'pitch_deg': event.drone_state.pitch_deg,
                'roll_deg': event.drone_state.roll_deg,
                'yaw_deg': event.drone_state.yaw_deg,
                'ground_speed_ms': event.drone_state.ground_speed_ms,
                'timestamp': event.drone_state.timestamp.isoformat(),
                
                # Camera state
                'iso': event.camera_state.iso,
                'aperture': event.camera_state.aperture,
                'shutter_speed': event.camera_state.shutter_speed,
                'focal_length_mm': event.camera_state.focal_length_mm,
                'white_balance': event.camera_state.white_balance,
                'exposure_mode': event.camera_state.exposure_mode,
                'focus_mode': event.camera_state.focus_mode,
                'image_format': event.camera_state.image_format,
            }
            csv_data.append(row)
        
        df = pd.DataFrame(csv_data)
        df.to_csv(filename, index=False)
        print(f"Exported {len(csv_data)} trigger events to {filename}")
    
    def export_metadata_json(self, filename: str):
        """
        Export trigger metadata to JSON file.
        
        Args:
            filename: Output JSON filename
        """
        if not self.trigger_events:
            print("No trigger events to export")
            return
        
        export_data = {
            'mission_info': {
                'total_triggers': len(self.trigger_events),
                'trigger_mode': self.current_mode.value,
                'export_timestamp': datetime.now().isoformat()
            },
            'trigger_events': [event.to_dict() for event in self.trigger_events]
        }
        
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Exported {len(self.trigger_events)} trigger events to {filename}")
    
    def get_mission_summary(self) -> Dict[str, Any]:
        """
        Get summary statistics for current mission.
        
        Returns:
            Dictionary with mission statistics
        """
        if not self.trigger_events:
            return {'total_triggers': 0}
        
        # Calculate mission duration
        first_trigger = self.trigger_events[0].drone_state.timestamp
        last_trigger = self.trigger_events[-1].drone_state.timestamp
        duration_s = (last_trigger - first_trigger).total_seconds()
        
        # Calculate total distance traveled
        total_distance = 0
        for i in range(1, len(self.trigger_events)):
            pos1 = (self.trigger_events[i-1].drone_state.latitude,
                   self.trigger_events[i-1].drone_state.longitude)
            pos2 = (self.trigger_events[i].drone_state.latitude,
                   self.trigger_events[i].drone_state.longitude)
            total_distance += self._calculate_distance(pos1, pos2)
        
        # Get altitude range
        altitudes = [event.drone_state.altitude_m for event in self.trigger_events]
        
        return {
            'total_triggers': len(self.trigger_events),
            'mission_duration_s': duration_s,
            'total_distance_m': total_distance,
            'trigger_mode': self.current_mode.value,
            'altitude_range_m': {
                'min': min(altitudes) if altitudes else 0,
                'max': max(altitudes) if altitudes else 0,
                'avg': sum(altitudes) / len(altitudes) if altitudes else 0
            },
            'first_trigger': first_trigger.isoformat(),
            'last_trigger': last_trigger.isoformat()
        }