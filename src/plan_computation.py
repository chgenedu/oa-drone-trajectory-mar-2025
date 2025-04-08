import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance


def compute_distance_between_images(camera: Camera, dataset_spec: DatasetSpec) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """
    [footprint_x, footprint_y] = compute_image_footprint_on_surface(camera, dataset_spec.height)
    horizontal_distance = (1 - dataset_spec.overlap) * footprint_x
    vertical_distance = (1 - dataset_spec.sidelap) * footprint_y
    return np.array([horizontal_distance, vertical_distance])


# Bonus question for week 4
# Assume the camera's field of view/2 + angle_deg <= 90 degrees
def compute_distance_between_images_with_angle(camera: Camera, 
                                               dataset_spec: DatasetSpec,
                                               angle_deg_x: float,
                                               angle_deg_y: float) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap,
    and camera angle.
    We assume that we only need to adjust the footprint based on the camera angle.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        angle_deg_x (float): camera's gimbal angle in x direction (in degrees)
        angle_deg_y (float): camera's gimbal angle in y direction (in degrees)

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """
    [footprint_x, footprint_y] = compute_image_footprint_on_surface(camera, dataset_spec.height)

    angle_x = math.radians(angle_deg_x) # convert deg to rad
    angle_y = math.radians(angle_deg_y) # convert deg to rad

    # fov_x is field of view in the x direction.
    # unused:  fov_x = 2 * math.atan((camera.image_size_x_px/2) / camera.fx)
    [fov_deg_x, fov_deg_y] = fov(camera)
    [fov_x, fov_y] = [math.radians(fov_deg_x), math.radians(fov_deg_y)]

    # If camera_angle >= 0, We have two cases:
    # 1: If camera_angle >= FOV/2,
    # then footprint = (tan(camera_angle + FOV/2) - tan(camera_angle - FOV/2)) * height
    # 2: If 0 <= camera_angle < FOV/2,
    # then footprint = (tan(camera_angle + FOV/2) + tan(FOV/2 - camera_angle)) * height
    # If camera angle < 0, we still get the same equations since tan() is an odd function.
    # Then we use wolframalpha.com to simplify these functions and we found out that
    # both of them can be simplified to the same form, regardless of whether
    # the camera_angle is greater than FOV/2:
    # footprint = 2 * sin(FOV) / (cos(FOV) + cos(2*camera_angle))

    footprint_x = 2*math.sin(fov_x) / (math.cos(fov_x) + math.cos(2*angle_x)) * dataset_spec.height
    footprint_y = 2*math.sin(fov_y) / (math.cos(fov_y) + math.cos(2*angle_y)) * dataset_spec.height

    horizontal_distance = (1 - dataset_spec.overlap) * footprint_x
    vertical_distance = (1 - dataset_spec.sidelap) * footprint_y
    return np.array([horizontal_distance, vertical_distance])


# Helper function for compute_distance_between_images_with_angle()
# calculate the field of view of a camera
def fov(camera: Camera) -> np.ndarray:
    """Compute the field of view of a camera.

    Args:
        camera (Camera): Camera model used for image capture.

    Returns:
        float: The field of view of the camera in the horizontal (x) direction (in degrees)
        float: The field of view of the camera in the vertical (y) direction (in degrees)
    """
    fov_x = 2 * math.atan((camera.image_size_x_px/2) / camera.fx)
    fov_y = 2 * math.atan((camera.image_size_y_px/2) / camera.fy)
    return np.array([math.degrees(fov_x), math.degrees(fov_y)])


def compute_speed_during_photo_capture(camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        float: The speed at which the drone should move during photo capture.
    """
    raise NotImplementedError()


def generate_photo_plan_on_grid(camera: Camera, dataset_spec: DatasetSpec) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    raise NotImplementedError()
