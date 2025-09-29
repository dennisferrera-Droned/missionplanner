"""
Metadata processor for extracting and managing image EXIF data and drone telemetry.

Handles reading EXIF data from images, combining with drone telemetry logs,
and exporting comprehensive metadata for photogrammetry processing.
"""

import os
import json
import pandas as pd
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, asdict
from PIL import Image
from PIL.ExifTags import TAGS
import exifread


@dataclass
class ImageMetadata:
    """Complete metadata for a single image."""
    filename: str
    filepath: str
    # GPS/Position data
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude_m: Optional[float] = None
    
    # Drone attitude
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    heading_deg: Optional[float] = None
    
    # Camera/EXIF data
    focal_length_mm: Optional[float] = None
    sensor_width_mm: Optional[float] = None
    sensor_height_mm: Optional[float] = None
    image_width_px: Optional[int] = None
    image_height_px: Optional[int] = None
    iso: Optional[int] = None
    aperture: Optional[str] = None
    shutter_speed: Optional[str] = None
    white_balance: Optional[str] = None
    
    # Timestamps
    capture_time: Optional[datetime] = None
    gps_time: Optional[datetime] = None
    
    # Additional metadata
    camera_model: Optional[str] = None
    lens_model: Optional[str] = None
    ground_speed_ms: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary with proper timestamp formatting."""
        data = asdict(self)
        if self.capture_time:
            data['capture_time'] = self.capture_time.isoformat()
        if self.gps_time:
            data['gps_time'] = self.gps_time.isoformat()
        return data


class MetadataProcessor:
    """
    Processor for extracting and managing image metadata.
    
    Combines EXIF data from images with drone telemetry logs to create
    comprehensive metadata for photogrammetry processing.
    """
    
    def __init__(self):
        """Initialize metadata processor."""
        self.image_metadata: List[ImageMetadata] = []
        self.default_camera_params = {
            'focal_length_mm': 24.0,
            'sensor_width_mm': 13.2,
            'sensor_height_mm': 8.8
        }
    
    def process_image_directory(self, image_dir: str, 
                              telemetry_file: Optional[str] = None) -> List[ImageMetadata]:
        """
        Process all images in directory and extract metadata.
        
        Args:
            image_dir: Directory containing images
            telemetry_file: Optional CSV/JSON file with drone telemetry data
            
        Returns:
            List of ImageMetadata objects
        """
        self.image_metadata = []
        
        # Load telemetry data if provided
        telemetry_data = {}
        if telemetry_file and os.path.exists(telemetry_file):
            telemetry_data = self._load_telemetry_data(telemetry_file)
        
        # Process all image files
        supported_formats = {'.jpg', '.jpeg', '.tiff', '.tif', '.png'}
        
        for filename in sorted(os.listdir(image_dir)):
            file_ext = os.path.splitext(filename)[1].lower()
            if file_ext in supported_formats:
                filepath = os.path.join(image_dir, filename)
                metadata = self._extract_image_metadata(filepath)
                
                # Merge with telemetry data if available
                if filename in telemetry_data:
                    metadata = self._merge_telemetry_data(metadata, telemetry_data[filename])
                
                self.image_metadata.append(metadata)
        
        print(f"Processed {len(self.image_metadata)} images from {image_dir}")
        return self.image_metadata
    
    def _load_telemetry_data(self, telemetry_file: str) -> Dict[str, Dict[str, Any]]:
        """Load telemetry data from CSV or JSON file."""
        telemetry = {}
        
        try:
            if telemetry_file.lower().endswith('.csv'):
                df = pd.read_csv(telemetry_file)
                for _, row in df.iterrows():
                    filename = row.get('image_filename', row.get('filename'))
                    if filename:
                        telemetry[filename] = row.to_dict()
            
            elif telemetry_file.lower().endswith('.json'):
                with open(telemetry_file, 'r') as f:
                    data = json.load(f)
                    if 'trigger_events' in data:
                        # DJI camera trigger format
                        for event in data['trigger_events']:
                            filename = event.get('image_filename')
                            if filename:
                                telemetry[filename] = {
                                    'latitude': event['drone_state']['latitude'],
                                    'longitude': event['drone_state']['longitude'],
                                    'altitude_m': event['drone_state']['altitude_m'],
                                    'yaw_deg': event['drone_state']['yaw_deg'],
                                    'pitch_deg': event['drone_state']['pitch_deg'],
                                    'roll_deg': event['drone_state']['roll_deg'],
                                    'heading_deg': event['drone_state']['heading_deg'],
                                    'ground_speed_ms': event['drone_state']['ground_speed_ms'],
                                    'capture_time': event['drone_state']['timestamp']
                                }
                    else:
                        # Generic JSON format
                        telemetry = data
            
        except Exception as e:
            print(f"Warning: Could not load telemetry data from {telemetry_file}: {e}")
        
        return telemetry
    
    def _extract_image_metadata(self, filepath: str) -> ImageMetadata:
        """Extract metadata from single image file."""
        filename = os.path.basename(filepath)
        metadata = ImageMetadata(filename=filename, filepath=filepath)
        
        try:
            # Use PIL for basic image info
            with Image.open(filepath) as img:
                metadata.image_width_px = img.width
                metadata.image_height_px = img.height
                
                # Extract EXIF data
                exif_data = img._getexif()
                if exif_data:
                    metadata = self._parse_exif_data(metadata, exif_data)
            
            # Use exifread for more detailed GPS data
            with open(filepath, 'rb') as f:
                exif_tags = exifread.process_file(f)
                metadata = self._parse_gps_data(metadata, exif_tags)
                
        except Exception as e:
            print(f"Warning: Could not extract metadata from {filepath}: {e}")
        
        # Apply default camera parameters if not found in EXIF
        if metadata.focal_length_mm is None:
            metadata.focal_length_mm = self.default_camera_params['focal_length_mm']
        if metadata.sensor_width_mm is None:
            metadata.sensor_width_mm = self.default_camera_params['sensor_width_mm']
        if metadata.sensor_height_mm is None:
            metadata.sensor_height_mm = self.default_camera_params['sensor_height_mm']
        
        return metadata
    
    def _parse_exif_data(self, metadata: ImageMetadata, exif_data: Dict) -> ImageMetadata:
        """Parse EXIF data from PIL."""
        try:
            # Camera settings
            if 34855 in exif_data:  # ISO
                metadata.iso = exif_data[34855]
            
            if 33437 in exif_data:  # F-number
                f_num = exif_data[33437]
                if isinstance(f_num, tuple):
                    metadata.aperture = f"f/{f_num[0]/f_num[1]:.1f}"
                else:
                    metadata.aperture = f"f/{f_num}"
            
            if 33434 in exif_data:  # Exposure time
                exp_time = exif_data[33434]
                if isinstance(exp_time, tuple):
                    metadata.shutter_speed = f"1/{int(exp_time[1]/exp_time[0])}"
                else:
                    metadata.shutter_speed = str(exp_time)
            
            if 37386 in exif_data:  # Focal length
                focal = exif_data[37386]
                if isinstance(focal, tuple):
                    metadata.focal_length_mm = focal[0] / focal[1]
                else:
                    metadata.focal_length_mm = focal
            
            # Camera/lens info
            if 272 in exif_data:  # Camera model
                metadata.camera_model = exif_data[272]
            
            if 42036 in exif_data:  # Lens model
                metadata.lens_model = exif_data[42036]
            
            # Capture time
            if 36867 in exif_data:  # DateTimeOriginal
                try:
                    metadata.capture_time = datetime.strptime(
                        exif_data[36867], "%Y:%m:%d %H:%M:%S"
                    )
                except:
                    pass
                    
        except Exception as e:
            print(f"Warning: Error parsing EXIF data: {e}")
        
        return metadata
    
    def _parse_gps_data(self, metadata: ImageMetadata, exif_tags: Dict) -> ImageMetadata:
        """Parse GPS data from exifread tags."""
        try:
            # GPS coordinates
            if 'GPS GPSLatitude' in exif_tags and 'GPS GPSLatitudeRef' in exif_tags:
                lat = self._convert_gps_coordinate(exif_tags['GPS GPSLatitude'])
                if exif_tags['GPS GPSLatitudeRef'].values[0] == 'S':
                    lat = -lat
                metadata.latitude = lat
            
            if 'GPS GPSLongitude' in exif_tags and 'GPS GPSLongitudeRef' in exif_tags:
                lon = self._convert_gps_coordinate(exif_tags['GPS GPSLongitude'])
                if exif_tags['GPS GPSLongitudeRef'].values[0] == 'W':
                    lon = -lon
                metadata.longitude = lon
            
            # GPS altitude
            if 'GPS GPSAltitude' in exif_tags:
                alt_ratio = exif_tags['GPS GPSAltitude'].values[0]
                metadata.altitude_m = float(alt_ratio.num) / float(alt_ratio.den)
                
                # Check altitude reference
                if ('GPS GPSAltitudeRef' in exif_tags and 
                    exif_tags['GPS GPSAltitudeRef'].values[0] == 1):
                    metadata.altitude_m = -metadata.altitude_m
            
            # GPS timestamp
            if 'GPS GPSTimeStamp' in exif_tags and 'GPS GPSDateStamp' in exif_tags:
                try:
                    time_values = exif_tags['GPS GPSTimeStamp'].values
                    date_stamp = str(exif_tags['GPS GPSDateStamp'])
                    
                    hours = float(time_values[0].num) / float(time_values[0].den)
                    minutes = float(time_values[1].num) / float(time_values[1].den)
                    seconds = float(time_values[2].num) / float(time_values[2].den)
                    
                    gps_datetime_str = f"{date_stamp} {int(hours):02d}:{int(minutes):02d}:{int(seconds):02d}"
                    metadata.gps_time = datetime.strptime(gps_datetime_str, "%Y:%m:%d %H:%M:%S")
                except:
                    pass
                    
        except Exception as e:
            print(f"Warning: Error parsing GPS data: {e}")
        
        return metadata
    
    def _convert_gps_coordinate(self, gps_coord) -> float:
        """Convert GPS coordinate from DMS to decimal degrees."""
        degrees = float(gps_coord.values[0].num) / float(gps_coord.values[0].den)
        minutes = float(gps_coord.values[1].num) / float(gps_coord.values[1].den)
        seconds = float(gps_coord.values[2].num) / float(gps_coord.values[2].den)
        
        return degrees + minutes/60.0 + seconds/3600.0
    
    def _merge_telemetry_data(self, metadata: ImageMetadata, 
                            telemetry: Dict[str, Any]) -> ImageMetadata:
        """Merge telemetry data with image metadata."""
        # GPS/Position (prefer telemetry over EXIF for accuracy)
        if 'latitude' in telemetry and telemetry['latitude'] is not None:
            metadata.latitude = float(telemetry['latitude'])
        if 'longitude' in telemetry and telemetry['longitude'] is not None:
            metadata.longitude = float(telemetry['longitude'])
        if 'altitude_m' in telemetry and telemetry['altitude_m'] is not None:
            metadata.altitude_m = float(telemetry['altitude_m'])
        
        # Drone attitude
        if 'yaw_deg' in telemetry:
            metadata.yaw_deg = float(telemetry['yaw_deg'])
        if 'pitch_deg' in telemetry:
            metadata.pitch_deg = float(telemetry['pitch_deg'])
        if 'roll_deg' in telemetry:
            metadata.roll_deg = float(telemetry['roll_deg'])
        if 'heading_deg' in telemetry:
            metadata.heading_deg = float(telemetry['heading_deg'])
        
        # Additional data
        if 'ground_speed_ms' in telemetry:
            metadata.ground_speed_ms = float(telemetry['ground_speed_ms'])
        
        # Timestamps
        if 'capture_time' in telemetry:
            try:
                if isinstance(telemetry['capture_time'], str):
                    metadata.capture_time = datetime.fromisoformat(
                        telemetry['capture_time'].replace('Z', '+00:00')
                    )
            except:
                pass
        
        return metadata
    
    def export_csv(self, output_file: str):
        """Export metadata to CSV file compatible with photogrammetry software."""
        if not self.image_metadata:
            print("No metadata to export")
            return
        
        # Flatten metadata for CSV export
        csv_data = []
        for metadata in self.image_metadata:
            row = metadata.to_dict()
            csv_data.append(row)
        
        df = pd.DataFrame(csv_data)
        df.to_csv(output_file, index=False)
        print(f"Exported metadata for {len(csv_data)} images to {output_file}")
    
    def export_json(self, output_file: str):
        """Export metadata to JSON file."""
        if not self.image_metadata:
            print("No metadata to export")
            return
        
        export_data = {
            'dataset_info': {
                'total_images': len(self.image_metadata),
                'export_timestamp': datetime.now().isoformat(),
                'default_camera_params': self.default_camera_params
            },
            'images': [metadata.to_dict() for metadata in self.image_metadata]
        }
        
        with open(output_file, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Exported metadata for {len(self.image_metadata)} images to {output_file}")
    
    def export_odm_format(self, output_dir: str):
        """
        Export metadata in OpenDroneMap compatible format.
        
        Creates images.csv and gcp_list.txt files for ODM processing.
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # Create images.csv for ODM
        csv_file = os.path.join(output_dir, 'images.csv')
        odm_data = []
        
        for metadata in self.image_metadata:
            if metadata.latitude is not None and metadata.longitude is not None:
                row = {
                    'filename': metadata.filename,
                    'lat': metadata.latitude,
                    'lon': metadata.longitude,
                    'alt': metadata.altitude_m or 0,
                    'yaw': metadata.yaw_deg or 0,
                    'pitch': metadata.pitch_deg or 0,
                    'roll': metadata.roll_deg or 0,
                    'focal_length': metadata.focal_length_mm or 24.0,
                    'sensor_width': metadata.sensor_width_mm or 13.2
                }
                odm_data.append(row)
        
        if odm_data:
            df = pd.DataFrame(odm_data)
            df.to_csv(csv_file, index=False)
            print(f"Exported ODM-compatible metadata to {csv_file}")
        else:
            print("Warning: No images with GPS coordinates found for ODM export")
    
    def get_dataset_summary(self) -> Dict[str, Any]:
        """Get summary statistics for the processed dataset."""
        if not self.image_metadata:
            return {'total_images': 0}
        
        # Count images with GPS data
        gps_images = [m for m in self.image_metadata if m.latitude is not None]
        
        # Calculate coverage area
        if gps_images:
            lats = [m.latitude for m in gps_images]
            lons = [m.longitude for m in gps_images]
            alts = [m.altitude_m for m in gps_images if m.altitude_m is not None]
            
            coverage_area = {
                'lat_range': [min(lats), max(lats)],
                'lon_range': [min(lons), max(lons)],
                'center': [sum(lats)/len(lats), sum(lons)/len(lons)]
            }
            
            altitude_stats = {
                'min': min(alts) if alts else None,
                'max': max(alts) if alts else None,
                'avg': sum(alts)/len(alts) if alts else None
            }
        else:
            coverage_area = None
            altitude_stats = None
        
        # Count camera models
        camera_models = {}
        for metadata in self.image_metadata:
            model = metadata.camera_model or 'Unknown'
            camera_models[model] = camera_models.get(model, 0) + 1
        
        return {
            'total_images': len(self.image_metadata),
            'images_with_gps': len(gps_images),
            'coverage_area': coverage_area,
            'altitude_stats': altitude_stats,
            'camera_models': camera_models,
            'time_range': self._get_time_range()
        }
    
    def _get_time_range(self) -> Optional[Dict[str, str]]:
        """Get time range of captured images."""
        timestamps = [m.capture_time for m in self.image_metadata if m.capture_time]
        if not timestamps:
            return None
        
        return {
            'start': min(timestamps).isoformat(),
            'end': max(timestamps).isoformat(),
            'duration_hours': (max(timestamps) - min(timestamps)).total_seconds() / 3600
        }