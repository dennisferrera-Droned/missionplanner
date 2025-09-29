"""
Image processor for basic image operations and preparation for photogrammetry.

Handles image resizing, format conversion, quality assessment,
and preparation for OpenDroneMap processing.
"""

import os
import cv2
import numpy as np
from typing import List, Dict, Any, Tuple, Optional
from PIL import Image
from dataclasses import dataclass


@dataclass
class ImageQualityMetrics:
    """Quality metrics for an image."""
    filename: str
    blur_score: float  # Laplacian variance (higher = sharper)
    brightness: float  # Mean pixel intensity
    contrast: float   # Standard deviation of pixel intensities
    noise_level: float  # Noise estimation
    resolution: Tuple[int, int]  # (width, height)
    file_size_mb: float
    is_acceptable: bool = True


class ImageProcessor:
    """
    Image processor for photogrammetry preparation.
    
    Provides tools for image quality assessment, format conversion,
    and preparation for OpenDroneMap processing.
    """
    
    def __init__(self):
        """Initialize image processor."""
        self.quality_thresholds = {
            'min_blur_score': 100.0,  # Minimum sharpness
            'min_brightness': 30.0,   # Too dark
            'max_brightness': 225.0,  # Too bright
            'min_contrast': 20.0,     # Too flat
            'max_noise': 50.0,        # Too noisy
            'min_resolution': (1000, 1000)  # Minimum dimensions
        }
    
    def assess_image_quality(self, image_path: str) -> ImageQualityMetrics:
        """
        Assess quality metrics for a single image.
        
        Args:
            image_path: Path to image file
            
        Returns:
            ImageQualityMetrics object with quality assessment
        """
        filename = os.path.basename(image_path)
        
        # Load image
        image = cv2.imread(image_path)
        if image is None:
            return ImageQualityMetrics(
                filename=filename,
                blur_score=0,
                brightness=0,
                contrast=0,
                noise_level=100,
                resolution=(0, 0),
                file_size_mb=0,
                is_acceptable=False
            )
        
        # Convert to grayscale for analysis
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Calculate metrics
        blur_score = self._calculate_blur_score(gray)
        brightness = self._calculate_brightness(gray)
        contrast = self._calculate_contrast(gray)
        noise_level = self._estimate_noise(gray)
        
        # Get image info
        height, width = gray.shape
        file_size_mb = os.path.getsize(image_path) / (1024 * 1024)
        
        # Determine if image is acceptable
        is_acceptable = self._is_image_acceptable(
            blur_score, brightness, contrast, noise_level, (width, height)
        )
        
        return ImageQualityMetrics(
            filename=filename,
            blur_score=blur_score,
            brightness=brightness,
            contrast=contrast,
            noise_level=noise_level,
            resolution=(width, height),
            file_size_mb=file_size_mb,
            is_acceptable=is_acceptable
        )
    
    def _calculate_blur_score(self, gray_image: np.ndarray) -> float:
        """Calculate blur score using Laplacian variance."""
        laplacian = cv2.Laplacian(gray_image, cv2.CV_64F)
        return laplacian.var()
    
    def _calculate_brightness(self, gray_image: np.ndarray) -> float:
        """Calculate average brightness."""
        return np.mean(gray_image)
    
    def _calculate_contrast(self, gray_image: np.ndarray) -> float:
        """Calculate contrast as standard deviation."""
        return np.std(gray_image)
    
    def _estimate_noise(self, gray_image: np.ndarray) -> float:
        """Estimate noise level using local standard deviation."""
        # Use a simple noise estimation method
        # Convert to float to avoid overflow
        img_float = gray_image.astype(np.float64)
        
        # Calculate local variance
        kernel = np.ones((5, 5)) / 25
        local_mean = cv2.filter2D(img_float, -1, kernel)
        local_var = cv2.filter2D(img_float**2, -1, kernel) - local_mean**2
        
        # Noise estimate is median of local standard deviations
        noise_estimate = np.median(np.sqrt(np.maximum(local_var, 0)))
        return noise_estimate
    
    def _is_image_acceptable(self, blur_score: float, brightness: float, 
                           contrast: float, noise_level: float, 
                           resolution: Tuple[int, int]) -> bool:
        """Determine if image meets quality thresholds."""
        width, height = resolution
        
        checks = [
            blur_score >= self.quality_thresholds['min_blur_score'],
            brightness >= self.quality_thresholds['min_brightness'],
            brightness <= self.quality_thresholds['max_brightness'],
            contrast >= self.quality_thresholds['min_contrast'],
            noise_level <= self.quality_thresholds['max_noise'],
            width >= self.quality_thresholds['min_resolution'][0],
            height >= self.quality_thresholds['min_resolution'][1]
        ]
        
        return all(checks)
    
    def assess_dataset_quality(self, image_dir: str) -> Dict[str, Any]:
        """
        Assess quality for all images in a directory.
        
        Args:
            image_dir: Directory containing images
            
        Returns:
            Dictionary with quality assessment results
        """
        quality_results = []
        supported_formats = {'.jpg', '.jpeg', '.tiff', '.tif', '.png'}
        
        for filename in sorted(os.listdir(image_dir)):
            file_ext = os.path.splitext(filename)[1].lower()
            if file_ext in supported_formats:
                image_path = os.path.join(image_dir, filename)
                quality = self.assess_image_quality(image_path)
                quality_results.append(quality)
        
        # Calculate summary statistics
        if not quality_results:
            return {'total_images': 0, 'acceptable_images': 0}
        
        acceptable_count = sum(1 for q in quality_results if q.is_acceptable)
        blur_scores = [q.blur_score for q in quality_results]
        brightness_values = [q.brightness for q in quality_results]
        
        return {
            'total_images': len(quality_results),
            'acceptable_images': acceptable_count,
            'acceptance_rate': acceptable_count / len(quality_results),
            'quality_metrics': {
                'avg_blur_score': np.mean(blur_scores),
                'min_blur_score': np.min(blur_scores),
                'avg_brightness': np.mean(brightness_values),
                'brightness_range': [np.min(brightness_values), np.max(brightness_values)]
            },
            'low_quality_images': [
                q.filename for q in quality_results if not q.is_acceptable
            ],
            'detailed_results': quality_results
        }
    
    def resize_images(self, input_dir: str, output_dir: str, 
                     max_dimension: int = 4000, quality: int = 95):
        """
        Resize images to maximum dimension while maintaining aspect ratio.
        
        Args:
            input_dir: Input directory with original images
            output_dir: Output directory for resized images
            max_dimension: Maximum width or height in pixels
            quality: JPEG quality (1-100)
        """
        os.makedirs(output_dir, exist_ok=True)
        
        supported_formats = {'.jpg', '.jpeg', '.tiff', '.tif', '.png'}
        processed_count = 0
        
        for filename in sorted(os.listdir(input_dir)):
            file_ext = os.path.splitext(filename)[1].lower()
            if file_ext in supported_formats:
                input_path = os.path.join(input_dir, filename)
                output_path = os.path.join(output_dir, filename)
                
                try:
                    with Image.open(input_path) as img:
                        # Calculate new dimensions
                        width, height = img.size
                        if max(width, height) > max_dimension:
                            ratio = max_dimension / max(width, height)
                            new_width = int(width * ratio)
                            new_height = int(height * ratio)
                            
                            # Resize image
                            resized_img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
                            
                            # Save with appropriate format
                            if file_ext in {'.jpg', '.jpeg'}:
                                resized_img.save(output_path, 'JPEG', quality=quality, optimize=True)
                            else:
                                resized_img.save(output_path)
                            
                            processed_count += 1
                        else:
                            # Copy original if already within size limits
                            img.save(output_path)
                            processed_count += 1
                            
                except Exception as e:
                    print(f"Warning: Could not process {filename}: {e}")
        
        print(f"Resized {processed_count} images to {output_dir}")
    
    def convert_to_jpg(self, input_dir: str, output_dir: str, quality: int = 95):
        """
        Convert all images to JPEG format for consistency.
        
        Args:
            input_dir: Input directory with mixed format images
            output_dir: Output directory for JPEG images
            quality: JPEG quality (1-100)
        """
        os.makedirs(output_dir, exist_ok=True)
        
        supported_formats = {'.jpg', '.jpeg', '.tiff', '.tif', '.png', '.bmp'}
        converted_count = 0
        
        for filename in sorted(os.listdir(input_dir)):
            file_ext = os.path.splitext(filename)[1].lower()
            if file_ext in supported_formats:
                input_path = os.path.join(input_dir, filename)
                
                # Change extension to .jpg
                base_name = os.path.splitext(filename)[0]
                output_filename = f"{base_name}.jpg"
                output_path = os.path.join(output_dir, output_filename)
                
                try:
                    with Image.open(input_path) as img:
                        # Convert to RGB if necessary (for PNG with transparency)
                        if img.mode in ('RGBA', 'LA', 'P'):
                            # Create white background
                            rgb_img = Image.new('RGB', img.size, (255, 255, 255))
                            if img.mode == 'P':
                                img = img.convert('RGBA')
                            rgb_img.paste(img, mask=img.split()[-1] if img.mode in ('RGBA', 'LA') else None)
                            img = rgb_img
                        elif img.mode != 'RGB':
                            img = img.convert('RGB')
                        
                        # Save as JPEG
                        img.save(output_path, 'JPEG', quality=quality, optimize=True)
                        converted_count += 1
                        
                except Exception as e:
                    print(f"Warning: Could not convert {filename}: {e}")
        
        print(f"Converted {converted_count} images to JPEG format in {output_dir}")
    
    def remove_low_quality_images(self, image_dir: str, backup_dir: Optional[str] = None):
        """
        Remove or move low-quality images from dataset.
        
        Args:
            image_dir: Directory containing images
            backup_dir: Optional directory to move low-quality images (instead of deleting)
        """
        quality_assessment = self.assess_dataset_quality(image_dir)
        low_quality_images = quality_assessment['low_quality_images']
        
        if not low_quality_images:
            print("No low-quality images found")
            return
        
        if backup_dir:
            os.makedirs(backup_dir, exist_ok=True)
        
        removed_count = 0
        for filename in low_quality_images:
            image_path = os.path.join(image_dir, filename)
            
            if backup_dir:
                # Move to backup directory
                backup_path = os.path.join(backup_dir, filename)
                try:
                    os.rename(image_path, backup_path)
                    removed_count += 1
                except Exception as e:
                    print(f"Warning: Could not move {filename}: {e}")
            else:
                # Delete file
                try:
                    os.remove(image_path)
                    removed_count += 1
                except Exception as e:
                    print(f"Warning: Could not delete {filename}: {e}")
        
        action = "moved to backup" if backup_dir else "deleted"
        print(f"{action.capitalize()} {removed_count} low-quality images")
    
    def prepare_for_odm(self, input_dir: str, output_dir: str, 
                       max_dimension: int = 4000) -> Dict[str, Any]:
        """
        Prepare images for OpenDroneMap processing.
        
        Performs quality assessment, format conversion, and resizing as needed.
        
        Args:
            input_dir: Directory with original images
            output_dir: Directory for ODM-ready images
            max_dimension: Maximum image dimension
            
        Returns:
            Preparation summary
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # Step 1: Assess quality
        print("Assessing image quality...")
        quality_assessment = self.assess_dataset_quality(input_dir)
        
        # Step 2: Convert to JPEG and resize
        print("Converting and resizing images...")
        temp_dir = os.path.join(output_dir, 'temp')
        self.convert_to_jpg(input_dir, temp_dir, quality=95)
        self.resize_images(temp_dir, output_dir, max_dimension=max_dimension)
        
        # Clean up temp directory
        import shutil
        shutil.rmtree(temp_dir, ignore_errors=True)
        
        # Step 3: Final quality check
        final_quality = self.assess_dataset_quality(output_dir)
        
        summary = {
            'input_images': quality_assessment['total_images'],
            'output_images': final_quality['total_images'],
            'initial_quality': quality_assessment,
            'final_quality': final_quality,
            'processing_steps': [
                'Quality assessment',
                'Format conversion to JPEG',
                f'Resizing to max {max_dimension}px',
                'Final quality check'
            ]
        }
        
        print(f"Prepared {summary['output_images']} images for ODM processing")
        return summary