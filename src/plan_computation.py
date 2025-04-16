import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance, fov


def compute_distance_between_images(camera: Camera, dataset_spec: DatasetSpec) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction. (in meters)
        float: The distance between images in the vertical direction. (in meters)
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
    and camera angles.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        angle_deg_x (float): camera's gimbal angle toward the x direction (in degrees)
        angle_deg_y (float): camera's gimbal angle toward the y direction (in degrees)

    Returns:
        float: The distance between images in the horizontal direction. (in meters)
        float: The distance between images in the vertical direction. (in meters)
    """
    [footprint_x, footprint_y] = compute_image_footprint_on_surface(camera, dataset_spec.height)

    angle_x = math.radians(angle_deg_x) # convert deg to rad
    angle_y = math.radians(angle_deg_y) # convert deg to rad

    # fov_x is field of view in the x direction.
    # unused:  fov_x = 2 * math.atan((camera.image_size_x_px/2) / camera.fx)
    [fov_deg_x, fov_deg_y] = fov(camera)
    [fov_x, fov_y] = [math.radians(fov_deg_x), math.radians(fov_deg_y)]

    # Below is an explanation of the calculation formula for the horizontal footprint.
    # The formula for vertical footprint is analogous.
    #
    # If camera_angle_x >= 0, We have two cases:
    # If camera_angle_x >= FOV/2,
    # then horizontal_footprint = (tan(camera_angle_x+FOV/2) - tan(camera_angle_x-FOV/2)) * height
    # If 0 <= camera_angle_x < FOV/2,
    # then horizontal_footprint = (tan(camera_angle_x+FOV/2) + tan(FOV/2-camera_angle_x)) * height
    # But tan() is an odd function. By using this fact, we know that tan(FOV/2 - camera_angle_x)
    # is the same as -tan(camera_angle_x - FOV/2), which means that the two formulas above
    # are the same.
    #
    # Furthermore, by using the fact that tan() is an odd function again,
    # we know that when camera angle_x < 0, we still get the same equations.
    # So either of the above equations would work whenever camera_angle_x satisfies
    # the condition that FOV/2 + abs(camera_angle_x) < 90 degrees.
    #
    # Conclusion (after a slight rearrangement):
    # Assuming FOV/2 + abs(camera_angle_x) < 90 degrees, then
    # horizontal_footprint = (tan(FOV/2+camera_angle_x) + tan(FOV/2-camera_angle_x)) * height

    footprint_x = (math.tan(fov_x/2 + angle_x) + math.tan(fov_x/2 - angle_x)) * dataset_spec.height
    footprint_y = (math.tan(fov_y/2 + angle_y) + math.tan(fov_y/2 - angle_y)) * dataset_spec.height

    horizontal_distance = (1 - dataset_spec.overlap) * footprint_x
    vertical_distance = (1 - dataset_spec.sidelap) * footprint_y
    return np.array([horizontal_distance, vertical_distance])


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
