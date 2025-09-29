"""
Command-line interface for DJI Photogrammetry SDK.

Provides commands for mission creation, image ingestion, metadata processing,
and dataset management for drone photogrammetry workflows.
"""

import os
import sys
import json
import click
from typing import Optional, List
from datetime import datetime

from .mission_planner import MissionPlanner, FlightPattern, MissionSettings, ROIPolygon, CameraSettings
from .metadata_processor import MetadataProcessor
from .image_processor import ImageProcessor
from .processing_engine import ProcessingEngine, ProcessingOptions, process_dataset


@click.group()
@click.version_option(version='0.1.0')
def cli():
    """DJI Photogrammetry SDK - Tools for drone survey missions and processing."""
    pass


@cli.command()
@click.argument('roi_file', type=click.Path(exists=True))
@click.option('--pattern', type=click.Choice(['grid', 'double_grid', 'oblique']), 
              default='grid', help='Flight pattern type')
@click.option('--altitude', type=float, default=100.0, help='Flight altitude in meters')
@click.option('--front-overlap', type=float, default=80.0, help='Forward overlap percentage')
@click.option('--side-overlap', type=float, default=70.0, help='Side overlap percentage')
@click.option('--speed', type=float, default=15.0, help='Flight speed in m/s')
@click.option('--output', type=click.Path(), default='mission.json', help='Output mission file')
@click.option('--focal-length', type=float, default=24.0, help='Camera focal length in mm')
@click.option('--sensor-width', type=float, default=13.2, help='Sensor width in mm')
@click.option('--gimbal-pitch', type=float, default=-90.0, help='Gimbal pitch angle in degrees')
def create_mission(roi_file: str, pattern: str, altitude: float, front_overlap: float,
                  side_overlap: float, speed: float, output: str, focal_length: float,
                  sensor_width: float, gimbal_pitch: float):
    """
    Create flight mission from ROI polygon.
    
    ROI_FILE should be a JSON file with polygon coordinates:
    {"vertices": [[lat1, lon1], [lat2, lon2], ...]}
    """
    try:
        # Load ROI polygon
        with open(roi_file, 'r') as f:
            roi_data = json.load(f)
        
        roi = ROIPolygon(vertices=roi_data['vertices'])
        
        # Create camera settings
        camera_settings = CameraSettings(
            focal_length_mm=focal_length,
            sensor_width_mm=sensor_width,
            gimbal_pitch_deg=gimbal_pitch
        )
        
        # Create mission settings
        mission_settings = MissionSettings(
            altitude_m=altitude,
            front_overlap_pct=front_overlap,
            side_overlap_pct=side_overlap,
            flight_speed_ms=speed
        )
        
        # Create mission planner
        planner = MissionPlanner(camera_settings)
        
        # Generate mission
        flight_pattern = FlightPattern(pattern)
        mission = planner.create_mission(flight_pattern, roi, mission_settings)
        
        # Export mission
        mission_export = {
            'mission_info': {
                'created': datetime.now().isoformat(),
                'pattern': pattern,
                'roi_file': roi_file,
                'total_waypoints': len(mission['waypoints'])
            },
            'mission_data': mission
        }
        
        # Convert waypoints to serializable format
        waypoints_data = []
        for wp in mission['waypoints']:
            waypoints_data.append({
                'latitude': wp.latitude,
                'longitude': wp.longitude,
                'altitude_m': wp.altitude_m,
                'heading_deg': wp.heading_deg,
                'gimbal_pitch_deg': wp.gimbal_pitch_deg,
                'trigger_camera': wp.trigger_camera
            })
        
        mission_export['mission_data']['waypoints'] = waypoints_data
        
        # Convert other dataclasses to dicts
        mission_export['mission_data']['settings'] = {
            'altitude_m': mission_settings.altitude_m,
            'front_overlap_pct': mission_settings.front_overlap_pct,
            'side_overlap_pct': mission_settings.side_overlap_pct,
            'flight_speed_ms': mission_settings.flight_speed_ms
        }
        
        mission_export['mission_data']['camera_settings'] = {
            'focal_length_mm': camera_settings.focal_length_mm,
            'sensor_width_mm': camera_settings.sensor_width_mm,
            'sensor_height_mm': camera_settings.sensor_height_mm,
            'gimbal_pitch_deg': camera_settings.gimbal_pitch_deg
        }
        
        with open(output, 'w') as f:
            json.dump(mission_export, f, indent=2)
        
        click.echo(f"✓ Mission created: {output}")
        click.echo(f"  Pattern: {pattern}")
        click.echo(f"  Waypoints: {len(mission['waypoints'])}")
        click.echo(f"  Estimated photos: {mission['mission_stats']['estimated_photos']}")
        click.echo(f"  Coverage area: {mission['mission_stats']['coverage_area_m2']:.0f} m²")
        click.echo(f"  GSD: {mission['mission_stats']['gsd_cm_per_pixel']:.1f} cm/pixel")
        
    except Exception as e:
        click.echo(f"✗ Error creating mission: {e}", err=True)
        sys.exit(1)


@cli.command()
@click.argument('image_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--telemetry', type=click.Path(exists=True), 
              help='Optional telemetry CSV/JSON file')
@click.option('--output-csv', type=click.Path(), default='images.csv',
              help='Output CSV file with metadata')
@click.option('--output-json', type=click.Path(), default='images.json',
              help='Output JSON file with metadata')
@click.option('--odm-format', is_flag=True, help='Export in ODM-compatible format')
def ingest(image_dir: str, telemetry: Optional[str], output_csv: str, 
          output_json: str, odm_format: bool):
    """
    Ingest images and extract metadata for photogrammetry processing.
    
    IMAGE_DIR should contain drone images with EXIF data.
    """
    try:
        # Create metadata processor
        processor = MetadataProcessor()
        
        # Process images
        click.echo(f"Processing images from: {image_dir}")
        metadata_list = processor.process_image_directory(image_dir, telemetry)
        
        if not metadata_list:
            click.echo("✗ No images found or processed", err=True)
            sys.exit(1)
        
        # Export metadata
        processor.export_csv(output_csv)
        processor.export_json(output_json)
        
        if odm_format:
            odm_dir = os.path.dirname(output_csv)
            processor.export_odm_format(odm_dir)
            click.echo(f"✓ ODM-compatible files created in: {odm_dir}")
        
        # Show summary
        summary = processor.get_dataset_summary()
        click.echo(f"✓ Processed {summary['total_images']} images")
        click.echo(f"  Images with GPS: {summary['images_with_gps']}")
        
        if summary['coverage_area']:
            center_lat, center_lon = summary['coverage_area']['center']
            click.echo(f"  Coverage center: {center_lat:.6f}, {center_lon:.6f}")
        
        if summary['altitude_stats'] and summary['altitude_stats']['avg']:
            click.echo(f"  Average altitude: {summary['altitude_stats']['avg']:.1f} m")
        
    except Exception as e:
        click.echo(f"✗ Error ingesting images: {e}", err=True)
        sys.exit(1)


@cli.command()
@click.argument('dataset_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--output', type=click.Path(), default='output',
              help='Output directory for processing results')
@click.option('--resolution', type=float, default=5.0,
              help='Output resolution in cm/pixel')
@click.option('--feature-quality', type=click.Choice(['low', 'medium', 'high', 'ultra']),
              default='high', help='Feature extraction quality')
@click.option('--pc-quality', type=click.Choice(['low', 'medium', 'high', 'ultra']),
              default='medium', help='Point cloud quality')
@click.option('--skip-orthophoto', is_flag=True, help='Skip orthophoto generation')
@click.option('--skip-dsm', is_flag=True, help='Skip DSM generation')
@click.option('--skip-mesh', is_flag=True, help='Skip textured mesh generation')
@click.option('--use-gpu', is_flag=True, default=True, help='Use GPU acceleration')
@click.option('--max-concurrency', type=int, default=4, help='Maximum concurrent processes')
def process(dataset_dir: str, output: str, resolution: float, feature_quality: str,
           pc_quality: str, skip_orthophoto: bool, skip_dsm: bool, skip_mesh: bool,
           use_gpu: bool, max_concurrency: int):
    """
    Process dataset using OpenDroneMap to generate orthomosaic and 3D models.
    
    DATASET_DIR should contain images and metadata files.
    """
    try:
        # Check dependencies
        engine = ProcessingEngine()
        deps = engine.check_dependencies()
        
        if not any(deps.values()):
            click.echo("✗ OpenDroneMap not found. Please install ODM or Docker.", err=True)
            click.echo("See: https://docs.opendronemap.org/installation/")
            sys.exit(1)
        
        if deps['docker']:
            click.echo("Using Docker-based OpenDroneMap")
        elif deps['odm_direct']:
            click.echo("Using direct OpenDroneMap installation")
        
        # Create processing options
        options = ProcessingOptions(
            output_resolution=resolution,
            feature_quality=feature_quality,
            pc_quality=pc_quality,
            orthophoto=not skip_orthophoto,
            dsm=not skip_dsm,
            textured_mesh=not skip_mesh,
            use_gpu=use_gpu,
            max_concurrency=max_concurrency
        )
        
        # Process dataset
        click.echo(f"Starting photogrammetry processing...")
        click.echo(f"Dataset: {dataset_dir}")
        click.echo(f"Output: {output}")
        
        result = engine.process_dataset(dataset_dir, output, options)
        
        if result['success']:
            click.echo(f"✓ Processing completed successfully")
            click.echo(f"  Processing time: {result['processing_time_s']:.1f} seconds")
            click.echo(f"  Input images: {result['input_images']}")
            click.echo(f"  Results in: {result['project_dir']}")
            
            # Show available outputs
            outputs = result['outputs']
            for output_type, info in outputs.items():
                if info['exists']:
                    click.echo(f"  {output_type}: {info['size_mb']:.1f} MB")
        else:
            click.echo(f"✗ Processing failed: {result['error_message']}", err=True)
            if result.get('log_file'):
                click.echo(f"Check log file: {result['log_file']}")
            sys.exit(1)
            
    except Exception as e:
        click.echo(f"✗ Error during processing: {e}", err=True)
        sys.exit(1)


@cli.command()
@click.argument('image_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--remove-low-quality', is_flag=True, help='Remove low quality images')
@click.option('--backup-dir', type=click.Path(), help='Backup directory for removed images')
@click.option('--max-dimension', type=int, default=4000, help='Maximum image dimension')
@click.option('--output-dir', type=click.Path(), help='Output directory for processed images')
def quality_check(image_dir: str, remove_low_quality: bool, backup_dir: Optional[str],
                 max_dimension: int, output_dir: Optional[str]):
    """
    Assess and improve image quality for photogrammetry processing.
    
    IMAGE_DIR should contain drone images to analyze.
    """
    try:
        processor = ImageProcessor()
        
        # Assess quality
        click.echo(f"Assessing image quality in: {image_dir}")
        quality_results = processor.assess_dataset_quality(image_dir)
        
        total_images = quality_results['total_images']
        acceptable_images = quality_results['acceptable_images']
        acceptance_rate = quality_results['acceptance_rate']
        
        click.echo(f"✓ Quality assessment complete")
        click.echo(f"  Total images: {total_images}")
        click.echo(f"  Acceptable images: {acceptable_images}")
        click.echo(f"  Acceptance rate: {acceptance_rate:.1%}")
        
        if quality_results['low_quality_images']:
            click.echo(f"  Low quality images: {len(quality_results['low_quality_images'])}")
            for filename in quality_results['low_quality_images'][:5]:  # Show first 5
                click.echo(f"    {filename}")
            if len(quality_results['low_quality_images']) > 5:
                click.echo(f"    ... and {len(quality_results['low_quality_images']) - 5} more")
        
        # Remove low quality images if requested
        if remove_low_quality and quality_results['low_quality_images']:
            if backup_dir:
                click.echo(f"Moving low quality images to: {backup_dir}")
            else:
                click.echo("Deleting low quality images...")
            
            processor.remove_low_quality_images(image_dir, backup_dir)
        
        # Prepare images for ODM if output directory specified
        if output_dir:
            click.echo(f"Preparing images for ODM processing...")
            prep_summary = processor.prepare_for_odm(image_dir, output_dir, max_dimension)
            click.echo(f"✓ Prepared {prep_summary['output_images']} images for ODM")
            
    except Exception as e:
        click.echo(f"✗ Error during quality check: {e}", err=True)
        sys.exit(1)


@cli.command()
@click.option('--output', type=click.Path(), default='docker-compose.yml',
              help='Output file for docker-compose configuration')
def setup_docker(output: str):
    """
    Create docker-compose.yml for OpenDroneMap processing.
    """
    try:
        engine = ProcessingEngine()
        engine.create_docker_compose(output)
        
        click.echo(f"✓ Docker Compose configuration created: {output}")
        click.echo("To start ODM services:")
        click.echo("  docker-compose up -d")
        click.echo("WebODM interface will be available at: http://localhost:8000")
        
    except Exception as e:
        click.echo(f"✗ Error creating docker-compose: {e}", err=True)
        sys.exit(1)


@cli.command()
@click.argument('lat', type=float)
@click.argument('lon', type=float)
@click.argument('width_m', type=float)
@click.argument('height_m', type=float)
@click.option('--output', type=click.Path(), default='roi.json',
              help='Output ROI polygon file')
def create_roi(lat: float, lon: float, width_m: float, height_m: float, output: str):
    """
    Create ROI polygon from center point and dimensions.
    
    Creates a rectangular polygon centered at LAT, LON with given WIDTH_M and HEIGHT_M.
    """
    try:
        # Convert meters to approximate degrees
        lat_per_m = 1 / 111320  # degrees per meter latitude
        lon_per_m = 1 / (111320 * abs(lat * 3.14159 / 180))  # approximate longitude
        
        # Calculate half dimensions in degrees
        half_height_deg = (height_m / 2) * lat_per_m
        half_width_deg = (width_m / 2) * lon_per_m
        
        # Create rectangular vertices
        vertices = [
            [lat - half_height_deg, lon - half_width_deg],  # SW
            [lat - half_height_deg, lon + half_width_deg],  # SE
            [lat + half_height_deg, lon + half_width_deg],  # NE
            [lat + half_height_deg, lon - half_width_deg],  # NW
            [lat - half_height_deg, lon - half_width_deg]   # Close polygon
        ]
        
        roi_data = {
            'center': [lat, lon],
            'dimensions_m': [width_m, height_m],
            'vertices': vertices
        }
        
        with open(output, 'w') as f:
            json.dump(roi_data, f, indent=2)
        
        click.echo(f"✓ ROI polygon created: {output}")
        click.echo(f"  Center: {lat:.6f}, {lon:.6f}")
        click.echo(f"  Dimensions: {width_m} x {height_m} meters")
        click.echo(f"  Vertices: {len(vertices)} points")
        
    except Exception as e:
        click.echo(f"✗ Error creating ROI: {e}", err=True)
        sys.exit(1)


def main():
    """Main CLI entry point."""
    cli()


if __name__ == '__main__':
    main()