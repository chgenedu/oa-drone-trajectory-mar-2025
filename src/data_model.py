"""Data models for the camera and user specification."""
from dataclasses import dataclass

@dataclass
class Camera:
    """
    Data model for a simple pinhole camera.
    
    References: 
    - https://github.com/colmap/colmap/blob/3f75f71310fdec803ab06be84a16cee5032d8e0d/src/colmap/sensor/models.h#L220
    - https://en.wikipedia.org/wiki/Pinhole_camera_model
    """
    # added by CH
    fx: float
    fy: float
    cx: float
    cy: float
    sensor_size_x_mm: float
    sensor_size_y_mm: float
    image_size_x_px: float
    image_size_y_px: float


@dataclass
class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """
    # added by CH
    overlap: float
    sidelap: float
    height: float
    scan_dimension_x: float
    scan_dimension_y: float
    exposure_time_ms: float


@dataclass
class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """
    pass
