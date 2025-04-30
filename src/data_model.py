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
    # z: position of the camera (meters)
    # speed: movement speed of the camera itself (meters per second)
    # camera_angle_x_deg: the camera's gimbal angle along the world's fixed horizontal direction
    #   as measured from the perpendicular line between the camera and the ground.
    # camera_angle_y_deg: the camera's gimbal angle along the world's fixed vertical direction
    #   as measured from the perpendicular line between the camera and the ground.
    x: float
    y: float
    z: float
    speed: float
    camera_angle_x_deg: float = 0
    camera_angle_y_deg: float = 0


@dataclass
class Coordinate:
    """
    Coordinates are positions of a point
    """
    # x: position of a point (meters)
    # y: position of a point (meters)
    # z: position of a point (meters)
    x: float
    y: float
    z: float


@dataclass
class FootprintCoord:
    """
    Information about a the four corners of the footprint of a photo area.
    The coordinate is relative to the camera's coordinate.
    """
    # pt_1: corner 1 coordinate (meters)
    # pt_2: corner 2 coordinate (meters)
    # pt_3: corner 3 coordinate (meters)
    # pt_4: corner 4 coordinate (meters)
    # pt_c: coordinate of the point where the camera is aiming at (meters)
    # Note: all coordinates are relative to the camera's current coordinate.
    pt_1: Coordinate
    pt_2: Coordinate
    pt_3: Coordinate
    pt_4: Coordinate
    pt_c: Coordinate
