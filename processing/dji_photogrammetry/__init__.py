"""
DJI Photogrammetry SDK

A comprehensive SDK for DJI drone photogrammetry missions including:
- Mission planning (grid, double-grid, oblique, perimeter, spiral, terrain-follow)
- Camera trigger automation (time/distance-based)
- Metadata logging with GPS and drone telemetry
- Image processing with OpenDroneMap integration
- RTK/PPK support for survey-grade accuracy
- Route-follow / pace-match / backwards-flight / homepoint-update
"""

__version__ = "0.2.0"
__author__ = "DJI Photogrammetry SDK"

from .mission_planner import MissionPlanner, FlightPattern
from .camera_trigger import CameraTrigger, TriggerMode
from .metadata_processor import MetadataProcessor
from .image_processor import ImageProcessor
from .processing_engine import ProcessingEngine
from .rtk_ppk import RinexParser, GCPManager, PPKProcessor, GroundControlPoint
from .follow_mode import FollowRoute, FollowModeController, RouteWaypoint, RouteSegment

__all__ = [
    "MissionPlanner",
    "FlightPattern",
    "CameraTrigger",
    "TriggerMode",
    "MetadataProcessor",
    "ImageProcessor",
    "ProcessingEngine",
    "RinexParser",
    "GCPManager",
    "PPKProcessor",
    "GroundControlPoint",
    "FollowRoute",
    "FollowModeController",
    "RouteWaypoint",
    "RouteSegment",
]
