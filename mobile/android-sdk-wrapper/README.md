# DJI Photogrammetry SDK - Android Wrapper

Android library wrapper for DJI Mobile SDK providing photogrammetry mission planning, camera triggering, and metadata logging capabilities.

## Features

- **Mission Planning**: Grid, double-grid, and oblique flight patterns
- **Camera Triggering**: Time-based, distance-based, and waypoint triggering
- **Metadata Logging**: Complete GPS, attitude, and camera settings capture
- **DJI SDK Integration**: Direct integration with DJI Mobile SDK v4.16.4

## Setup

### Requirements

- Android Studio 4.0+
- Minimum SDK: API 21 (Android 5.0)
- Target SDK: API 34 (Android 14)
- DJI Developer Account and App Key

### Installation

1. **Add to your project**:
   ```gradle
   dependencies {
       implementation 'com.dji:photogrammetry-sdk:0.1.0'
   }
   ```

2. **Get DJI App Key**:
   - Register at [DJI Developer Portal](https://developer.dji.com/)
   - Create new app and get App Key
   - Add to `AndroidManifest.xml`:
   ```xml
   <meta-data
       android:name="com.dji.sdk.API_KEY"
       android:value="YOUR_DJI_APP_KEY" />
   ```

3. **Add permissions**:
   ```xml
   <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
   <uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION" />
   <uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE" />
   <uses-permission android:name="android.permission.READ_EXTERNAL_STORAGE" />
   ```

## Usage

### Mission Planning

```kotlin
import com.dji.photogrammetry.*

// Create ROI polygon
val roi = ROIPolygon(listOf(
    Pair(37.7749, -122.4194),  // SW
    Pair(37.7749, -122.4094),  // SE  
    Pair(37.7849, -122.4094),  // NE
    Pair(37.7849, -122.4194)   // NW
))

// Configure mission settings
val settings = MissionSettings(
    altitudeM = 100.0,
    frontOverlapPct = 80.0,
    sideOverlapPct = 70.0,
    flightSpeedMs = 15.0
)

// Create mission planner
val planner = MissionPlanner()
val mission = planner.createMission(FlightPattern.GRID, roi, settings)

// Export mission
val json = planner.exportMissionToJson(mission)
```

### Camera Triggering

```kotlin
// Implement camera trigger listener
class MyTriggerListener : CameraTriggerListener {
    override fun onCameraTrigger(): String? {
        // Trigger DJI camera and return filename
        return "IMG_${System.currentTimeMillis()}.jpg"
    }
    
    override fun getCurrentDroneState(): DroneState? {
        // Get current drone state from DJI SDK
        return DroneState(/* ... */)
    }
    
    override fun getCurrentCameraState(): CameraState? {
        // Get current camera settings
        return CameraState(/* ... */)
    }
}

// Set up camera trigger
val trigger = CameraTrigger()
trigger.setListener(MyTriggerListener())
trigger.setTriggerMode(TriggerMode.DISTANCE_BASED, "distanceIntervalM" to 30.0)
trigger.startTriggering()

// In flight loop
trigger.update() // Call regularly during flight
```

### Metadata Export

```kotlin
// Export trigger metadata
val metadataJson = trigger.exportMetadataJson()

// Get mission summary
val summary = trigger.getMissionSummary()
println("Total photos: ${summary.totalTriggers}")
println("Flight duration: ${summary.missionDurationS}s")
```

## Building AAR

```bash
./gradlew assembleRelease
```

The AAR file will be generated in `build/outputs/aar/`

## Integration with DJI SDK

This library is designed to work alongside the DJI Mobile SDK. You'll need to:

1. Initialize DJI SDK in your application
2. Handle aircraft connection and authorization
3. Implement the camera trigger callbacks to use DJI camera APIs
4. Use DJI waypoint mission or virtual stick APIs for flight control

See the [DJI Mobile SDK Documentation](https://developer.dji.com/mobile-sdk/) for complete integration details.

## License

MIT License - see LICENSE file for details.