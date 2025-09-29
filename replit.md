# Overview

The DJI Photogrammetry SDK is a comprehensive multi-platform solution for drone survey missions. It provides mission planning capabilities, automated camera triggering, metadata logging, and image processing for creating orthomosaics, point clouds, and 3D models from DJI drone imagery. The SDK consists of three main components: an Android wrapper for DJI Mobile SDK integration, a Python processing package that interfaces with OpenDroneMap/OpenSfM, and CLI tools for dataset management.

# User Preferences

Preferred communication style: Simple, everyday language.

# System Architecture

## Multi-Platform SDK Design
The system is architected as a modular SDK with three distinct but integrated components:
- **Mobile SDK Wrapper**: Android library that wraps DJI Mobile SDK for mission planning and camera control
- **Python Processing Package**: Core processing engine that handles image analysis and photogrammetry
- **CLI Interface**: Command-line tools for mission creation and dataset management

This separation allows users to integrate only the components they need while maintaining consistent data formats across platforms.

## Mission Planning Architecture
The mission planner uses a coordinate-based approach with configurable flight patterns:
- **ROI Definition**: Polygon-based region of interest with GPS coordinates
- **Pattern Generation**: Grid, double-grid, and oblique flight patterns with calculated waypoints
- **Overlap Calculation**: Automatic spacing based on camera parameters and desired overlap percentages
- **Waypoint Export**: Standardized JSON format compatible with DJI mission protocols

## Camera Trigger System
The camera triggering system supports multiple automation modes:
- **Time-based**: Configurable interval triggering with GPS logging
- **Distance-based**: Trigger based on traveled distance using GPS tracking
- **Waypoint-based**: Trigger at specific GPS coordinates during mission execution
- **Manual**: User-controlled triggering with full metadata capture

## Metadata Management
Comprehensive metadata handling combines EXIF data with drone telemetry:
- **EXIF Extraction**: Camera settings, timestamps, and embedded GPS data
- **Drone Telemetry**: Attitude, altitude, speed, and precise GPS coordinates
- **Data Fusion**: Synchronized metadata combining camera and flight data
- **Export Formats**: JSON and CSV outputs for downstream processing

## Image Processing Pipeline
The processing engine provides a wrapper around OpenDroneMap with quality assessment:
- **Quality Metrics**: Blur detection, brightness analysis, and noise assessment
- **Format Standardization**: Image preparation and validation for photogrammetry
- **ODM Integration**: CLI wrapper with configurable processing options
- **Output Generation**: Orthomosaics, point clouds, DEMs, and textured meshes

## Android SDK Integration
The mobile wrapper provides minimal DJI SDK integration:
- **Mission Upload**: Direct integration with DJI waypoint mission APIs
- **Camera Control**: Automated triggering during flight execution
- **Metadata Logging**: Real-time GPS and attitude data capture
- **Export Functions**: JSON/CSV export of mission data and telemetry

# External Dependencies

## DJI Mobile SDK
- **Purpose**: Core drone control and mission execution on Android
- **Integration**: Requires developer account and API key configuration
- **Features**: Waypoint missions, camera control, telemetry access
- **Version**: Compatible with DJI Mobile SDK v4.16.4

## OpenDroneMap (ODM)
- **Purpose**: Photogrammetry processing engine for generating outputs
- **Integration**: CLI wrapper with Docker containerization support
- **License**: AGPL (requires consideration for commercial use)
- **Outputs**: Orthomosaics, point clouds, DEMs, textured meshes

## Python Image Processing Stack
- **OpenCV**: Image quality assessment and computer vision operations
- **PIL/Pillow**: Image manipulation and EXIF data extraction
- **ExifRead**: Advanced EXIF metadata parsing
- **NumPy**: Numerical operations for image analysis

## Geospatial Libraries
- **PyProj**: Coordinate system transformations and projections
- **Pandas**: Metadata processing and CSV/JSON export
- **Click**: Command-line interface framework

## Development and Build Tools
- **Android Gradle**: Build system for Android AAR generation
- **GitHub Actions**: CI/CD for building and releasing artifacts
- **pytest**: Testing framework for Python components
- **Black/isort**: Code formatting and import organization

## Optional RTK/PPK Support
- **RINEX**: Industry standard format for high-precision GPS data
- **Ground Control Points**: Survey-grade georeferencing integration
- **Post-processing**: Hooks for external RTK/PPK correction workflows