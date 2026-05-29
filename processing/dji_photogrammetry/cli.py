"""
Command-line interface for DJI Photogrammetry SDK.

Provides commands for mission creation, image ingestion, metadata processing,
dataset management, RTK/PPK correction, and follow-route planning.
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
from .rtk_ppk import GCPManager, PPKProcessor, GroundControlPoint
from .follow_mode import FollowRoute, FollowModeController


@click.group()
@click.version_option(version='0.2.0')
def cli():
    """DJI Photogrammetry SDK - Tools for drone survey missions and processing."""
    pass


# ---------------------------------------------------------------------------
# create-mission
# ---------------------------------------------------------------------------

PATTERN_CHOICES = ['grid', 'double_grid', 'oblique', 'perimeter', 'spiral', 'terrain_follow']


@cli.command()
@click.argument('roi_file', type=click.Path(exists=True))
@click.option('--pattern', type=click.Choice(PATTERN_CHOICES),
              default='grid', help='Flight pattern type')
@click.option('--altitude', type=float, default=100.0, help='Flight altitude in meters')
@click.option('--front-overlap', type=float, default=80.0, help='Forward overlap percentage')
@click.option('--side-overlap', type=float, default=70.0, help='Side overlap percentage')
@click.option('--speed', type=float, default=15.0, help='Flight speed in m/s')
@click.option('--output', type=click.Path(), default='mission.json', help='Output mission file')
@click.option('--focal-length', type=float, default=24.0, help='Camera focal length in mm')
@click.option('--sensor-width', type=float, default=13.2, help='Sensor width in mm')
@click.option('--gimbal-pitch', type=float, default=-90.0, help='Gimbal pitch angle in degrees')
@click.option('--terrain-dem', type=click.Path(exists=True),
              help='JSON file with 2-D elevation grid for terrain-follow pattern')
def create_mission(roi_file, pattern, altitude, front_overlap, side_overlap,
                   speed, output, focal_length, sensor_width, gimbal_pitch,
                   terrain_dem):
    """
    Create flight mission from ROI polygon.

    ROI_FILE is a JSON file:  {"vertices": [[lat1, lon1], [lat2, lon2], ...]}

    Patterns available:
      grid          – Classic parallel lawnmower grid (default)
      double_grid   – Cross-hatch grid for 3D modelling
      oblique       – Fly around vertices with angled camera
      perimeter     – Dense boundary orbit with inward-facing camera
      spiral        – Archimedean inward spiral (good for round areas)
      terrain_follow – Grid that adjusts altitude to match ground elevation
    """
    try:
        with open(roi_file, 'r') as f:
            roi_data = json.load(f)

        roi = ROIPolygon(vertices=roi_data['vertices'])

        camera_settings = CameraSettings(
            focal_length_mm=focal_length,
            sensor_width_mm=sensor_width,
            gimbal_pitch_deg=gimbal_pitch
        )

        mission_settings = MissionSettings(
            altitude_m=altitude,
            front_overlap_pct=front_overlap,
            side_overlap_pct=side_overlap,
            flight_speed_ms=speed
        )

        planner = MissionPlanner(camera_settings)
        flight_pattern = FlightPattern(pattern)

        # Terrain-follow needs an elevation grid
        if flight_pattern == FlightPattern.TERRAIN_FOLLOW and terrain_dem:
            with open(terrain_dem, 'r') as f:
                terrain_grid = json.load(f)
            waypoints = planner.generate_terrain_follow_mission(
                roi, mission_settings, terrain_grid
            )
            distance_interval, time_interval = planner.calculate_photo_interval(
                altitude, front_overlap, speed
            )
            mission = {
                'pattern': pattern,
                'waypoints': waypoints,
                'settings': mission_settings,
                'camera_settings': camera_settings,
                'photo_intervals': {'distance_m': distance_interval, 'time_s': time_interval},
                'mission_stats': {
                    'total_waypoints': len(waypoints),
                    'gsd_cm_per_pixel': planner.calculate_ground_sampling_distance(altitude),
                    'coverage_area_m2': planner._calculate_roi_area(roi),
                    'estimated_photos': planner._estimate_photo_count(waypoints, distance_interval)
                }
            }
        else:
            mission = planner.create_mission(flight_pattern, roi, mission_settings)

        # Serialise
        waypoints_data = []
        for wp in mission['waypoints']:
            waypoints_data.append({
                'latitude': wp.latitude,
                'longitude': wp.longitude,
                'altitude_m': wp.altitude_m,
                'heading_deg': wp.heading_deg,
                'gimbal_pitch_deg': wp.gimbal_pitch_deg,
                'trigger_camera': wp.trigger_camera,
                'fly_backwards': getattr(wp, 'fly_backwards', False)
            })

        mission_export = {
            'mission_info': {
                'created': datetime.now().isoformat(),
                'pattern': pattern,
                'roi_file': roi_file,
                'total_waypoints': len(mission['waypoints'])
            },
            'mission_data': {
                'pattern': mission['pattern'],
                'waypoints': waypoints_data,
                'settings': {
                    'altitude_m': mission_settings.altitude_m,
                    'front_overlap_pct': mission_settings.front_overlap_pct,
                    'side_overlap_pct': mission_settings.side_overlap_pct,
                    'flight_speed_ms': mission_settings.flight_speed_ms
                },
                'camera_settings': {
                    'focal_length_mm': camera_settings.focal_length_mm,
                    'sensor_width_mm': camera_settings.sensor_width_mm,
                    'sensor_height_mm': camera_settings.sensor_height_mm,
                    'gimbal_pitch_deg': camera_settings.gimbal_pitch_deg
                },
                'photo_intervals': mission.get('photo_intervals', {}),
                'mission_stats': mission.get('mission_stats', {})
            }
        }

        with open(output, 'w') as f:
            json.dump(mission_export, f, indent=2)

        click.echo(f"✓ Mission created: {output}")
        click.echo(f"  Pattern: {pattern}")
        click.echo(f"  Waypoints: {len(mission['waypoints'])}")
        stats = mission.get('mission_stats', {})
        if stats.get('estimated_photos'):
            click.echo(f"  Estimated photos: {stats['estimated_photos']}")
        if stats.get('coverage_area_m2'):
            click.echo(f"  Coverage area: {stats['coverage_area_m2']:.0f} m²")
        if stats.get('gsd_cm_per_pixel'):
            click.echo(f"  GSD: {stats['gsd_cm_per_pixel']:.1f} cm/pixel")

    except Exception as e:
        click.echo(f"✗ Error creating mission: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# create-roi
# ---------------------------------------------------------------------------

@cli.command()
@click.argument('lat', type=float)
@click.argument('lon', type=float)
@click.argument('width_m', type=float)
@click.argument('height_m', type=float)
@click.option('--output', type=click.Path(), default='roi.json', help='Output ROI polygon file')
def create_roi(lat, lon, width_m, height_m, output):
    """
    Create ROI polygon from center point and dimensions.

    Creates a rectangular polygon centred at LAT, LON with given WIDTH_M and HEIGHT_M.
    """
    try:
        lat_per_m = 1 / 111320
        lon_per_m = 1 / (111320 * abs(lat * 3.14159 / 180) + 1e-10)

        half_h = (height_m / 2) * lat_per_m
        half_w = (width_m / 2) * lon_per_m

        vertices = [
            [lat - half_h, lon - half_w],
            [lat - half_h, lon + half_w],
            [lat + half_h, lon + half_w],
            [lat + half_h, lon - half_w],
            [lat - half_h, lon - half_w]
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
        click.echo(f"  Dimensions: {width_m} x {height_m} m")
        click.echo(f"  Vertices: {len(vertices)} points")

    except Exception as e:
        click.echo(f"✗ Error creating ROI: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# ingest
# ---------------------------------------------------------------------------

@cli.command()
@click.argument('image_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--telemetry', type=click.Path(exists=True),
              help='Optional telemetry CSV/JSON file')
@click.option('--output-csv', type=click.Path(), default='images.csv')
@click.option('--output-json', type=click.Path(), default='images.json')
@click.option('--odm-format', is_flag=True, help='Also export ODM-compatible files')
def ingest(image_dir, telemetry, output_csv, output_json, odm_format):
    """
    Ingest images and extract metadata for photogrammetry processing.

    IMAGE_DIR should contain drone images with EXIF data.
    """
    try:
        processor = MetadataProcessor()
        click.echo(f"Processing images from: {image_dir}")
        metadata_list = processor.process_image_directory(image_dir, telemetry)

        if not metadata_list:
            click.echo("✗ No images found", err=True)
            sys.exit(1)

        processor.export_csv(output_csv)
        processor.export_json(output_json)

        if odm_format:
            odm_dir = os.path.dirname(output_csv) or '.'
            processor.export_odm_format(odm_dir)
            click.echo(f"✓ ODM-compatible files created in: {odm_dir}")

        summary = processor.get_dataset_summary()
        click.echo(f"✓ Processed {summary['total_images']} images")
        click.echo(f"  Images with GPS: {summary['images_with_gps']}")
        if summary.get('coverage_area'):
            clat, clon = summary['coverage_area']['center']
            click.echo(f"  Coverage centre: {clat:.6f}, {clon:.6f}")
        if summary.get('altitude_stats') and summary['altitude_stats'].get('avg'):
            click.echo(f"  Average altitude: {summary['altitude_stats']['avg']:.1f} m")

    except Exception as e:
        click.echo(f"✗ Error ingesting images: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# process
# ---------------------------------------------------------------------------

@cli.command()
@click.argument('dataset_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--output', type=click.Path(), default='output')
@click.option('--resolution', type=float, default=5.0, help='cm/pixel')
@click.option('--feature-quality', type=click.Choice(['low', 'medium', 'high', 'ultra']),
              default='high')
@click.option('--pc-quality', type=click.Choice(['low', 'medium', 'high', 'ultra']),
              default='medium')
@click.option('--skip-orthophoto', is_flag=True)
@click.option('--skip-dsm', is_flag=True)
@click.option('--skip-mesh', is_flag=True)
@click.option('--gcp-file', type=click.Path(exists=True), help='GCP file for georeferencing')
@click.option('--use-gpu/--no-gpu', default=True)
@click.option('--max-concurrency', type=int, default=4)
def process(dataset_dir, output, resolution, feature_quality, pc_quality,
            skip_orthophoto, skip_dsm, skip_mesh, gcp_file, use_gpu, max_concurrency):
    """
    Process dataset using OpenDroneMap to generate orthomosaic and 3D models.

    DATASET_DIR should contain images and metadata files.
    Optionally supply a --gcp-file for survey-grade georeferencing.
    """
    try:
        engine = ProcessingEngine()
        deps = engine.check_dependencies()

        if not any(deps.values()):
            click.echo("✗ OpenDroneMap not found. Please install ODM or Docker.", err=True)
            click.echo("See: https://docs.opendronemap.org/installation/")
            sys.exit(1)

        options = ProcessingOptions(
            output_resolution=resolution,
            feature_quality=feature_quality,
            pc_quality=pc_quality,
            orthophoto=not skip_orthophoto,
            dsm=not skip_dsm,
            textured_mesh=not skip_mesh,
            gcp_file=gcp_file,
            use_gpu=use_gpu,
            max_concurrency=max_concurrency
        )

        click.echo(f"Starting photogrammetry processing…")
        result = engine.process_dataset(dataset_dir, output, options)

        if result['success']:
            click.echo(f"✓ Processing completed in {result['processing_time_s']:.1f} s")
            click.echo(f"  Input images: {result['input_images']}")
            click.echo(f"  Results in: {result['project_dir']}")
            for out_type, info in result['outputs'].items():
                if info.get('exists'):
                    click.echo(f"  {out_type}: {info['size_mb']:.1f} MB")
        else:
            click.echo(f"✗ Processing failed: {result['error_message']}", err=True)
            sys.exit(1)

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# quality-check
# ---------------------------------------------------------------------------

@cli.command()
@click.argument('image_dir', type=click.Path(exists=True, file_okay=False))
@click.option('--remove-low-quality', is_flag=True)
@click.option('--backup-dir', type=click.Path())
@click.option('--max-dimension', type=int, default=4000)
@click.option('--output-dir', type=click.Path())
def quality_check(image_dir, remove_low_quality, backup_dir, max_dimension, output_dir):
    """Assess and improve image quality for photogrammetry processing."""
    try:
        processor = ImageProcessor()
        click.echo(f"Assessing image quality in: {image_dir}")
        qr = processor.assess_dataset_quality(image_dir)

        click.echo(f"✓ Quality assessment complete")
        click.echo(f"  Total: {qr['total_images']}  Acceptable: {qr['acceptable_images']}"
                   f"  ({qr['acceptance_rate']:.1%})")

        if qr.get('low_quality_images'):
            click.echo(f"  Low quality: {len(qr['low_quality_images'])}")
            for fn in qr['low_quality_images'][:5]:
                click.echo(f"    {fn}")
            if len(qr['low_quality_images']) > 5:
                click.echo(f"    … and {len(qr['low_quality_images']) - 5} more")

        if remove_low_quality and qr.get('low_quality_images'):
            processor.remove_low_quality_images(image_dir, backup_dir)

        if output_dir:
            prep = processor.prepare_for_odm(image_dir, output_dir, max_dimension)
            click.echo(f"✓ Prepared {prep['output_images']} images for ODM")

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# setup-docker
# ---------------------------------------------------------------------------

@cli.command()
@click.option('--output', type=click.Path(), default='docker-compose.yml')
def setup_docker(output):
    """Create docker-compose.yml for OpenDroneMap processing."""
    try:
        engine = ProcessingEngine()
        engine.create_docker_compose(output)
        click.echo(f"✓ docker-compose.yml created: {output}")
        click.echo("  docker-compose up -d")
        click.echo("  WebODM: http://localhost:8000")
    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# ppk-correct  (RTK/PPK integration)
# ---------------------------------------------------------------------------

@cli.command('ppk-correct')
@click.argument('drone_log', type=click.Path(exists=True))
@click.option('--base-lat', type=float, required=True,
              help='Base-station latitude (decimal degrees)')
@click.option('--base-lon', type=float, required=True,
              help='Base-station longitude (decimal degrees)')
@click.option('--base-alt', type=float, required=True,
              help='Base-station altitude above WGS-84 ellipsoid (m)')
@click.option('--rinex', type=click.Path(exists=True),
              help='Base-station RINEX observation file (.obs/.rnx)')
@click.option('--rtklib', type=click.Path(exists=True),
              help='Path to RTKLIB rnx2rtkp binary (optional, uses simplified model otherwise)')
@click.option('--output-csv', type=click.Path(), default='corrected_positions.csv')
@click.option('--report', type=click.Path(), default='survey_report.json')
def ppk_correct(drone_log, base_lat, base_lon, base_alt,
                rinex, rtklib, output_csv, report):
    """
    Apply PPK (Post-Processing Kinematic) corrections to drone image positions.

    DRONE_LOG is the JSON or CSV file produced by the camera trigger export.
    Corrected positions are written to OUTPUT_CSV and a quality report to REPORT.

    For survey-grade (cm) accuracy supply a RINEX base-station file and the
    RTKLIB rnx2rtkp binary.  Without them a simplified baseline model is used
    (~0.5 m accuracy) that is still sufficient for dataset preparation.
    """
    try:
        processor = PPKProcessor()
        processor.set_base_station(base_lat, base_lon, base_alt)

        click.echo(f"Loading drone log: {drone_log}")
        rows = processor.load_drone_log(drone_log)
        click.echo(f"  {len(rows)} image records loaded")

        if rinex:
            click.echo(f"RINEX base-station file: {rinex}")
        if rtklib:
            click.echo(f"RTKLIB binary: {rtklib}")
        else:
            click.echo("  (No RTKLIB path – using simplified correction model)")

        processor.apply_corrections(rows, rinex_path=rinex, rtklib_path=rtklib)

        processor.export_corrected_csv(output_csv)
        processor.generate_survey_report(report)

        click.echo(f"✓ PPK correction complete")
        click.echo(f"  Corrected positions: {output_csv}")
        click.echo(f"  Survey report: {report}")

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# create-gcp  (GCP file creation helper)
# ---------------------------------------------------------------------------

@cli.command('create-gcp')
@click.argument('gcp_json', type=click.Path(exists=True))
@click.option('--output', type=click.Path(), default='gcp_list.txt',
              help='Output ODM gcp_list.txt file')
@click.option('--proj', default='WGS84',
              help='PROJ string for the coordinate system (default: WGS84)')
def create_gcp(gcp_json, output, proj):
    """
    Convert a GCP JSON file to ODM gcp_list.txt format.

    GCP_JSON format:
    {
      "gcps": [
        {
          "name": "GCP01",
          "latitude": 47.1234,
          "longitude": 8.5678,
          "altitude_m": 435.2,
          "pixel_observations": [
            {"image": "DJI_001.jpg", "x": 1024, "y": 768}
          ]
        }
      ]
    }
    """
    try:
        manager = GCPManager()
        manager.load_from_json(gcp_json)

        validation = manager.validate()
        if not validation['valid']:
            click.echo(f"✗ GCP validation failed: {validation['error']}", err=True)
            sys.exit(1)

        manager.export_odm_gcp_list(output, proj_string=proj)

        click.echo(f"✓ GCP list created: {output}")
        click.echo(f"  GCPs: {validation['count']}")
        click.echo(f"  Pixel observations: {validation['total_pixel_observations']}")

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# create-route  (Follow-mode route builder)
# ---------------------------------------------------------------------------

@cli.command('create-route')
@click.option('--output', type=click.Path(), default='follow_route.json')
@click.option('--speed', type=float, default=5.0,
              help='Default speed in m/s (e.g. 3=jog, 6=run, 10=cycle)')
@click.option('--altitude', type=float, default=30.0,
              help='Flight altitude AGL in metres')
@click.option('--backwards/--forward', default=False,
              help='Fly backwards so camera faces the athlete/rider from front')
@click.option('--update-home/--keep-home', default=True,
              help='Update RTH home-point to the final route waypoint')
@click.argument('coords', nargs=-1)
def create_route(output, speed, altitude, backwards, update_home, coords):
    """
    Create a follow-mode route from a list of GPS coordinates.

    COORDS are pairs of LAT LON values:

      dji-photogrammetry create-route 47.1 8.5 47.11 8.51 47.12 8.52

    Each consecutive pair of points forms one segment. Supply --speed to
    set the default pace (override per-segment via the JSON file directly).

    Use --backwards so the drone flies in reverse and the camera always
    faces the approaching runner or rider.

    Use --update-home (default) so the Return-to-Home lands at the end of
    the route rather than the take-off point.
    """
    try:
        if len(coords) < 4 or len(coords) % 2 != 0:
            click.echo("✗ Provide at least 2 coordinate pairs (lat lon lat lon …)", err=True)
            sys.exit(1)

        route = FollowRoute(default_speed=speed,
                            fly_backwards=backwards,
                            update_home=update_home)

        for i in range(0, len(coords), 2):
            lat = float(coords[i])
            lon = float(coords[i + 1])
            route.add_waypoint(lat, lon, altitude, name=f"WP{i // 2 + 1}")

        route.export_json(output)

        total_m = route.total_length_m
        segs = route.build_segments()

        click.echo(f"✓ Follow route created: {output}")
        click.echo(f"  Waypoints: {len(route.waypoints)}")
        click.echo(f"  Segments: {len(segs)}")
        click.echo(f"  Total distance: {total_m:.0f} m")
        click.echo(f"  Speed: {speed} m/s ({speed * 3.6:.1f} km/h)")
        click.echo(f"  Direction: {'BACKWARDS (camera faces athlete)' if backwards else 'FORWARD'}")
        click.echo(f"  Home update at route end: {'yes' if update_home else 'no'}")
        est_time = total_m / speed if speed > 0 else 0
        click.echo(f"  Estimated flight time: {est_time / 60:.1f} min")

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


# ---------------------------------------------------------------------------
# simulate-route  (offline test of follow-mode controller)
# ---------------------------------------------------------------------------

@cli.command('simulate-route')
@click.argument('route_file', type=click.Path(exists=True))
@click.option('--pace', type=float, default=None,
              help='Override all segment speeds with this pace (m/s)')
@click.option('--backwards/--forward', default=None,
              help='Override backwards-flight flag for all segments')
@click.option('--report', type=click.Path(), default='route_progress.json')
def simulate_route(route_file, pace, backwards, report):
    """
    Simulate a follow-mode route offline and print velocity commands.

    ROUTE_FILE is a JSON file produced by create-route.
    Walks through every segment and prints the NED velocity commands that
    would be sent to the DJI Virtual Stick API.  Useful for verifying a
    route before flying.
    """
    try:
        route = FollowRoute.from_json(route_file)
        controller = FollowModeController(route)
        controller.start()

        if pace is not None:
            controller.set_pace_speed(pace)
        if backwards is not None:
            controller.set_backwards_flight(backwards)

        click.echo(f"Simulating {len(controller.segments)} segments…")
        click.echo(f"{'Seg':>4}  {'Bear':>6}  {'Dist':>8}  {'Speed':>6}  {'Yaw':>6}  {'Dir':>10}  vN/vE/vD")
        click.echo("─" * 80)

        for i, seg in enumerate(controller.segments):
            vn, ve, vd = seg.velocity_ned()
            direction = "BACKWARD" if seg.fly_backwards else "forward"
            click.echo(
                f"{i + 1:>4}  {seg.bearing_deg:>6.1f}°  {seg.length_m:>7.0f}m  "
                f"{seg.speed_ms:>5.1f}m/s  {seg.yaw_deg:>5.1f}°  {direction:>10}  "
                f"{vn:+.2f}/{ve:+.2f}/{vd:+.2f}"
            )

        final = route.final_waypoint
        if route.update_home and final:
            click.echo(f"\nHome-point will update to: {final.latitude:.6f}, {final.longitude:.6f}")

        controller.export_progress_report(report)
        click.echo(f"\n✓ Simulation report: {report}")

    except Exception as e:
        click.echo(f"✗ Error: {e}", err=True)
        sys.exit(1)


def main():
    """Main CLI entry point."""
    cli()


if __name__ == '__main__':
    main()
