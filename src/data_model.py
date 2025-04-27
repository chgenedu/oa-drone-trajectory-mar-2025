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
    # fx: focal length along x axis (in pixels)
    # fy: focal length along y axis (in pixels)
    # cx: optical center of the image along the x axis (in pixels)
    # cy: optical center of the image along the y axis (in pixels)
    # sensor_size_x_mm: Size of the sensor along the x axis (in mm)
    # sensor_size_y_mm: Size of the sensor along the y axis (in mm)
    # image_size_x_px: Number of pixels in the image along the x axis
    # image_size_y_px: Number of pixels in the image along the y axis
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
    # overlap: the ratio (in 0 to 1) of scene shared between two consecutive images.
    # sidelap: the ratio (in 0 to 1) of scene shared between two images in adjacent rows.
    # height: the height of the scan above the ground (in meters).
    # scan_dimension_x: the horizontal size of the rectangle to be scanned
    # scan_dimension_y: the vertical size of the rectangle to be scanned
    # exposure_time_ms: the exposure time for each image (in milliseconds).
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
    # x: position of the camera (meters)
    # y: position of the camera (meters)
    # speed: movement speed of the camera itself (meters per second)
    # camera_angle_x_deg: the camera's gimbal angle along the horizontal direction
    #   as measured from the perpendicular line between the camera and the ground.
    # camera_angle_y_deg: the camera's gimbal angle along the vertical direction
    #   as measured from the perpendicular line between the camera and the ground.
    x: float
    y: float
    speed: float
    camera_angle_x_deg: float = 0
    camera_angle_y_deg: float = 0
