"""
Processing engine for photogrammetry using OpenDroneMap/OpenSfM.

Provides a Python wrapper around ODM CLI commands to generate
orthomosaics, point clouds, DEMs, and textured meshes from drone imagery.
"""

import os
import subprocess
import json
import shutil
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime


@dataclass
class ProcessingOptions:
    """Configuration options for photogrammetry processing."""
    # General options
    project_name: str = "dji_survey"
    output_resolution: float = 5.0  # cm/pixel
    
    # Processing features
    orthophoto: bool = True
    dsm: bool = True  # Digital Surface Model
    dtm: bool = True  # Digital Terrain Model
    point_cloud: bool = True
    textured_mesh: bool = True
    
    # Quality settings
    feature_quality: str = "high"  # low, medium, high, ultra
    pc_quality: str = "medium"     # low, medium, high, ultra
    
    # Advanced options
    use_gpu: bool = True
    max_concurrency: int = 4
    feature_type: str = "sift"     # sift, hahog
    matcher_type: str = "flann"    # flann, bow
    
    # RTK/PPK options
    gcp_file: Optional[str] = None
    use_rtk: bool = False
    rtk_format: str = "rinex"
    
    def to_odm_args(self) -> List[str]:
        """Convert to ODM command line arguments."""
        args = []
        
        # Resolution
        args.extend(["--orthophoto-resolution", str(self.output_resolution)])
        
        # Feature settings
        args.extend(["--feature-quality", self.feature_quality])
        args.extend(["--pc-quality", self.pc_quality])
        args.extend(["--feature-type", self.feature_type])
        args.extend(["--matcher-type", self.matcher_type])
        
        # Processing options
        if not self.orthophoto:
            args.append("--skip-orthophoto")
        if not self.dsm:
            args.append("--skip-3dmodel")
        if not self.point_cloud:
            args.append("--skip-report")
        
        # Performance
        args.extend(["--max-concurrency", str(self.max_concurrency)])
        if self.use_gpu:
            args.append("--use-gpu")
        
        # GCP file
        if self.gcp_file and os.path.exists(self.gcp_file):
            args.extend(["--gcp", self.gcp_file])
        
        return args


class ProcessingEngine:
    """
    Processing engine for drone photogrammetry using OpenDroneMap.
    
    Provides high-level interface for processing image datasets
    into orthomosaics, point clouds, and 3D models.
    """
    
    def __init__(self, odm_path: Optional[str] = None):
        """
        Initialize processing engine.
        
        Args:
            odm_path: Path to ODM installation (if None, assumes 'docker' command available)
        """
        self.odm_path = odm_path
        self.use_docker = odm_path is None
        self.processing_results: List[Dict[str, Any]] = []
    
    def check_dependencies(self) -> Dict[str, bool]:
        """
        Check if required dependencies are available.
        
        Returns:
            Dictionary indicating availability of each dependency
        """
        dependencies = {}
        
        # Check Docker
        try:
            result = subprocess.run(['docker', '--version'], 
                                  capture_output=True, text=True, timeout=10)
            dependencies['docker'] = result.returncode == 0
        except:
            dependencies['docker'] = False
        
        # Check ODM direct installation
        if self.odm_path:
            dependencies['odm_direct'] = os.path.exists(self.odm_path)
        else:
            dependencies['odm_direct'] = False
        
        # Check OpenSfM
        try:
            import opensfm
            dependencies['opensfm'] = True
        except ImportError:
            dependencies['opensfm'] = False
        
        return dependencies
    
    def process_dataset(self, dataset_dir: str, output_dir: str, 
                       options: ProcessingOptions = None) -> Dict[str, Any]:
        """
        Process complete dataset using OpenDroneMap.
        
        Args:
            dataset_dir: Directory containing images and metadata
            output_dir: Output directory for results
            options: Processing configuration options
            
        Returns:
            Processing results dictionary
        """
        if options is None:
            options = ProcessingOptions()
        
        # Validate inputs
        if not os.path.exists(dataset_dir):
            raise ValueError(f"Dataset directory does not exist: {dataset_dir}")
        
        # Check for images
        image_files = self._find_images(dataset_dir)
        if not image_files:
            raise ValueError(f"No images found in dataset directory: {dataset_dir}")
        
        print(f"Processing {len(image_files)} images...")
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Prepare ODM project structure
        project_dir = os.path.join(output_dir, options.project_name)
        self._prepare_odm_project(dataset_dir, project_dir)
        
        # Run ODM processing
        start_time = datetime.now()
        
        try:
            if self.use_docker:
                result = self._run_odm_docker(project_dir, options)
            else:
                result = self._run_odm_direct(project_dir, options)
            
            end_time = datetime.now()
            processing_time = (end_time - start_time).total_seconds()
            
            # Collect results
            processing_result = {
                'success': result['success'],
                'project_dir': project_dir,
                'input_images': len(image_files),
                'processing_time_s': processing_time,
                'start_time': start_time.isoformat(),
                'end_time': end_time.isoformat(),
                'options': options,
                'outputs': self._collect_outputs(project_dir),
                'log_file': result.get('log_file'),
                'command': result.get('command', ''),
                'error_message': result.get('error', '')
            }
            
            self.processing_results.append(processing_result)
            
            if result['success']:
                print(f"Processing completed successfully in {processing_time:.1f} seconds")
                print(f"Results available in: {project_dir}")
            else:
                print(f"Processing failed: {result.get('error', 'Unknown error')}")
            
            return processing_result
            
        except Exception as e:
            end_time = datetime.now()
            processing_time = (end_time - start_time).total_seconds()
            
            error_result = {
                'success': False,
                'project_dir': project_dir,
                'input_images': len(image_files),
                'processing_time_s': processing_time,
                'error_message': str(e),
                'start_time': start_time.isoformat(),
                'end_time': end_time.isoformat()
            }
            
            print(f"Processing failed with exception: {e}")
            return error_result
    
    def _find_images(self, directory: str) -> List[str]:
        """Find all image files in directory."""
        supported_formats = {'.jpg', '.jpeg', '.tiff', '.tif', '.png'}
        images = []
        
        for filename in os.listdir(directory):
            if os.path.splitext(filename)[1].lower() in supported_formats:
                images.append(os.path.join(directory, filename))
        
        return sorted(images)
    
    def _prepare_odm_project(self, dataset_dir: str, project_dir: str):
        """Prepare ODM project directory structure."""
        # Create ODM project structure
        images_dir = os.path.join(project_dir, 'images')
        os.makedirs(images_dir, exist_ok=True)
        
        # Copy images to project
        image_files = self._find_images(dataset_dir)
        for image_path in image_files:
            filename = os.path.basename(image_path)
            dest_path = os.path.join(images_dir, filename)
            shutil.copy2(image_path, dest_path)
        
        # Copy metadata files if they exist
        metadata_files = ['images.csv', 'gcp_list.txt', 'camera_calibration.txt']
        for metadata_file in metadata_files:
            src_path = os.path.join(dataset_dir, metadata_file)
            if os.path.exists(src_path):
                dest_path = os.path.join(project_dir, metadata_file)
                shutil.copy2(src_path, dest_path)
                print(f"Copied metadata file: {metadata_file}")
    
    def _run_odm_docker(self, project_dir: str, options: ProcessingOptions) -> Dict[str, Any]:
        """Run ODM using Docker."""
        # Prepare Docker command
        docker_cmd = [
            'docker', 'run', '--rm',
            '-v', f'{os.path.abspath(project_dir)}:/datasets/project',
            'opendronemap/odm'
        ]
        
        # Add ODM arguments
        docker_cmd.extend(options.to_odm_args())
        docker_cmd.append('/datasets/project')
        
        # Run command
        log_file = os.path.join(project_dir, 'odm_log.txt')
        
        try:
            with open(log_file, 'w') as log:
                result = subprocess.run(
                    docker_cmd,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=7200  # 2 hour timeout
                )
            
            return {
                'success': result.returncode == 0,
                'command': ' '.join(docker_cmd),
                'log_file': log_file,
                'error': None if result.returncode == 0 else f"Process exited with code {result.returncode}"
            }
            
        except subprocess.TimeoutExpired:
            return {
                'success': False,
                'command': ' '.join(docker_cmd),
                'log_file': log_file,
                'error': 'Processing timeout (2 hours exceeded)'
            }
        except Exception as e:
            return {
                'success': False,
                'command': ' '.join(docker_cmd),
                'log_file': log_file,
                'error': str(e)
            }
    
    def _run_odm_direct(self, project_dir: str, options: ProcessingOptions) -> Dict[str, Any]:
        """Run ODM directly (non-Docker installation)."""
        if not self.odm_path or not os.path.exists(self.odm_path):
            return {
                'success': False,
                'error': f"ODM path not found: {self.odm_path}"
            }
        
        # Prepare command
        odm_cmd = [self.odm_path]
        odm_cmd.extend(options.to_odm_args())
        odm_cmd.append(project_dir)
        
        # Run command
        log_file = os.path.join(project_dir, 'odm_log.txt')
        
        try:
            with open(log_file, 'w') as log:
                result = subprocess.run(
                    odm_cmd,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=7200  # 2 hour timeout
                )
            
            return {
                'success': result.returncode == 0,
                'command': ' '.join(odm_cmd),
                'log_file': log_file,
                'error': None if result.returncode == 0 else f"Process exited with code {result.returncode}"
            }
            
        except subprocess.TimeoutExpired:
            return {
                'success': False,
                'command': ' '.join(odm_cmd),
                'log_file': log_file,
                'error': 'Processing timeout (2 hours exceeded)'
            }
        except Exception as e:
            return {
                'success': False,
                'command': ' '.join(odm_cmd),
                'log_file': log_file,
                'error': str(e)
            }
    
    def _collect_outputs(self, project_dir: str) -> Dict[str, Any]:
        """Collect information about generated outputs."""
        outputs = {
            'orthophoto': None,
            'dsm': None,
            'dtm': None,
            'point_cloud': None,
            'textured_mesh': None,
            'report': None
        }
        
        # Standard ODM output paths
        output_paths = {
            'orthophoto': 'odm_orthophoto/odm_orthophoto.tif',
            'dsm': 'odm_dem/dsm.tif',
            'dtm': 'odm_dem/dtm.tif',
            'point_cloud': 'odm_georeferencing/odm_georeferenced_model.ply',
            'textured_mesh': 'odm_texturing/odm_textured_model.obj',
            'report': 'odm_report/report.pdf'
        }
        
        for output_type, relative_path in output_paths.items():
            full_path = os.path.join(project_dir, relative_path)
            if os.path.exists(full_path):
                file_size = os.path.getsize(full_path)
                outputs[output_type] = {
                    'path': full_path,
                    'size_mb': file_size / (1024 * 1024),
                    'exists': True
                }
            else:
                outputs[output_type] = {'exists': False}
        
        return outputs
    
    def create_docker_compose(self, output_file: str = "docker-compose.yml"):
        """
        Create docker-compose file for running ODM.
        
        Args:
            output_file: Path for docker-compose.yml file
        """
        compose_content = """version: '3.8'

services:
  odm:
    image: opendronemap/odm:latest
    container_name: dji-photogrammetry-odm
    volumes:
      - ./datasets:/datasets
      - ./results:/results
    working_dir: /datasets
    environment:
      - ODM_MAX_CONCURRENCY=4
    command: >
      --project-path /datasets
      --orthophoto-resolution 5
      --feature-quality high
      --pc-quality medium
      --use-gpu
      --verbose

  webodm:
    image: opendronemap/webodm_webapp:latest
    container_name: dji-photogrammetry-webodm
    ports:
      - "8000:8000"
    volumes:
      - ./webodm_data:/webodm/app/media
    environment:
      - WO_PORT=8000
      - WO_DEBUG=NO
    depends_on:
      - db
      - redis

  db:
    image: postgres:13
    container_name: dji-photogrammetry-db
    environment:
      - POSTGRES_DB=webodm
      - POSTGRES_USER=webodm
      - POSTGRES_PASSWORD=webodm
    volumes:
      - ./db_data:/var/lib/postgresql/data

  redis:
    image: redis:6
    container_name: dji-photogrammetry-redis

volumes:
  db_data:
  webodm_data:

networks:
  default:
    name: dji-photogrammetry-network
"""
        
        with open(output_file, 'w') as f:
            f.write(compose_content)
        
        print(f"Created docker-compose.yml at {output_file}")
        print("To start ODM services: docker-compose up -d")
        print("WebODM interface will be available at: http://localhost:8000")
    
    def export_processing_report(self, output_file: str):
        """Export processing results to JSON report."""
        if not self.processing_results:
            print("No processing results to export")
            return
        
        report = {
            'report_generated': datetime.now().isoformat(),
            'total_processing_jobs': len(self.processing_results),
            'successful_jobs': sum(1 for r in self.processing_results if r['success']),
            'processing_jobs': self.processing_results
        }
        
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"Exported processing report to {output_file}")


def process_dataset(dataset_dir: str, output_dir: str = "output", **kwargs) -> Dict[str, Any]:
    """
    Convenience function for processing a dataset.
    
    Args:
        dataset_dir: Directory containing images and metadata
        output_dir: Output directory for results
        **kwargs: Additional processing options
        
    Returns:
        Processing results dictionary
    """
    # Create processing options from kwargs
    options = ProcessingOptions(**kwargs)
    
    # Initialize processing engine
    engine = ProcessingEngine()
    
    # Check dependencies
    deps = engine.check_dependencies()
    if not any(deps.values()):
        print("Warning: No ODM installation found. Please install OpenDroneMap or Docker.")
        print("See: https://docs.opendronemap.org/installation/")
        return {'success': False, 'error': 'ODM not available'}
    
    # Process dataset
    return engine.process_dataset(dataset_dir, output_dir, options)