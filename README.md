# DJI Photogrammetry SDK

A comprehensive multi-platform SDK for DJI drone photogrammetry that includes mobile wrappers, Python processing capabilities, and CLI tools for drone survey missions.

## 🚁 Features

### Core Capabilities
- **Mission Planning**: Grid, double-grid, and oblique flight patterns with configurable altitude, overlap, and camera settings
- **Auto Camera Trigger**: Time and distance-based triggering with GPS/INS data logging
- **RTK/PPK Support**: Integration hooks for survey-grade georeferencing accuracy
- **Image Processing**: Quality assessment, metadata extraction, and OpenDroneMap integration
- **Multi-Platform**: Android SDK wrapper and Python processing package

### Mobile SDK (Android)
- DJI Mobile SDK integration with mission planner APIs
- Camera trigger helpers with metadata logging
- Export to JSON/CSV formats with EXIF and drone telemetry

### Python Processing Package
- OpenDroneMap/OpenSfM CLI wrapper for orthomosaic and point cloud generation
- Image quality assessment and preprocessing
- Comprehensive metadata processing with EXIF extraction
- CLI tools for mission creation and dataset management

## 📦 Installation

### Python Package
```bash
pip install dji-photogrammetry
```

### Android AAR
Add to your `build.gradle`:
```gradle
dependencies {
    implementation 'com.dji:photogrammetry-sdk:0.1.0'
}
```

## 🚀 Quick Start

### 1. Mission Planning (Python CLI)
```bash
# Create ROI polygon
dji-photogrammetry create-roi 37.7749 -122.4194 500 300 --output roi.json

# Generate flight mission
dji-photogrammetry create-mission roi.json --pattern grid --altitude 100 --output mission.json

# Process images from drone
dji-photogrammetry ingest ./images --output-csv images.csv --odm-format

# Generate orthomosaic with OpenDroneMap
dji-photogrammetry process ./dataset --output ./results --resolution 5.0
```

### 2. Android Integration
```kotlin
// Mission planning
val roi = ROIPolygon(listOf(Pair(37.7749, -122.4194), /* ... */))
val settings = MissionSettings(altitudeM = 100.0, frontOverlapPct = 80.0)
val mission = MissionPlanner().createMission(FlightPattern.GRID, roi, settings)

// Camera triggering
val trigger = CameraTrigger()
trigger.setTriggerMode(TriggerMode.DISTANCE_BASED, "distanceIntervalM" to 30.0)
trigger.startTriggering()
```

### 3. Python Processing
```python
from dji_photogrammetry import process_dataset, MetadataProcessor

# Process image dataset
result = process_dataset("./images", output_dir="./output", 
                        resolution=5.0, feature_quality="high")

# Extract metadata
processor = MetadataProcessor()
metadata = processor.process_image_directory("./images")
processor.export_odm_format("./output")
```

## 🛠️ Development Setup

### Prerequisites
- Python 3.8+
- Android Studio (for mobile development)
- Docker (for OpenDroneMap)
- DJI Developer Account

### Local Development
```bash
git clone https://github.com/dji/photogrammetry-sdk
cd photogrammetry-sdk

# Python package development
cd processing
pip install -e .[dev]
pytest tests/

# Android development
cd mobile/android-sdk-wrapper
./gradlew assembleDebug
```

### OpenDroneMap Setup
```bash
# Using Docker (recommended)
dji-photogrammetry setup-docker
docker-compose up -d

# Or install ODM directly
# See: https://docs.opendronemap.org/installation/
```

## 📊 Supported Workflows

### Survey Mission Workflow
1. **Plan Mission**: Define ROI polygon and flight parameters
2. **Execute Flight**: Use DJI app with auto-triggering
3. **Process Images**: Extract metadata and assess quality  
4. **Generate Products**: Create orthomosaic, DSM, point cloud
5. **Post-Process**: Apply RTK/PPK corrections if needed

### Supported Output Formats
- **Orthomosaic**: GeoTIFF with proper georeferencing
- **Point Cloud**: PLY format with RGB colors
- **Digital Surface Model (DSM)**: GeoTIFF elevation data
- **Digital Terrain Model (DTM)**: Ground-classified elevation
- **Textured 3D Model**: OBJ with texture mapping

## 🔧 Configuration

### Camera Settings
```python
camera_settings = CameraSettings(
    focal_length_mm=24.0,
    sensor_width_mm=13.2,
    sensor_height_mm=8.8,
    gimbal_pitch_deg=-90.0
)
```

### Processing Options
```python
options = ProcessingOptions(
    output_resolution=5.0,  # cm/pixel
    feature_quality="high",
    use_gpu=True,
    orthophoto=True,
    point_cloud=True,
    textured_mesh=True
)
```

## 📱 DJI Integration

### Required Setup
1. Register at [DJI Developer Portal](https://developer.dji.com/)
2. Create app and obtain App Key
3. Add App Key to Android manifest:
```xml
<meta-data
    android:name="com.dji.sdk.API_KEY"
    android:value="YOUR_DJI_APP_KEY" />
```

### Supported DJI Aircraft
- Mavic series (Mavic 2, Mavic 3, etc.)
- Phantom 4 series
- Inspire series
- Matrice series (M300 RTK, M600, etc.)

## 🧪 Sample Dataset

A sample dataset with 6 images and metadata is included in `examples/sample_dataset/`:

```bash
# Process sample dataset
dji-photogrammetry process examples/sample_dataset --output results
```

## 🏗️ Architecture

```
dji-photogrammetry-sdk/
├── mobile/                    # Mobile SDK wrappers
│   └── android-sdk-wrapper/   # Android AAR library
├── processing/                # Python processing package
│   └── dji_photogrammetry/   # Core Python modules
├── cli/                      # Command-line tools (deprecated - moved to processing)
├── examples/                 # Sample datasets and tutorials
└── .github/workflows/        # CI/CD automation
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## ⚖️ OpenDroneMap License Notice

This SDK integrates with OpenDroneMap, which is licensed under AGPL-3.0. When using ODM for processing, you must comply with AGPL-3.0 license terms. The SDK itself (mobile wrappers, metadata processing) is MIT licensed.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## 📞 Support

- 📚 [Documentation](https://github.com/dji/photogrammetry-sdk/docs)
- 🐛 [Issue Tracker](https://github.com/dji/photogrammetry-sdk/issues)
- 💬 [DJI Developer Forum](https://forum.dji.com/forum-139-1.html)
- 📧 Email: sdk@example.com

## 🙏 Acknowledgments

- [DJI Mobile SDK](https://developer.dji.com/mobile-sdk/) for drone integration
- [OpenDroneMap](https://www.opendronemap.org/) for photogrammetry processing
- [OpenSfM](https://opensfm.org/) for structure-from-motion algorithms