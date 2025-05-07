import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance

# additional imports
from src.camera_utils import (compute_image_footprint_on_surface_with_angle,
                              compute_image_footprint_on_surface_coord,
                              closest_distance_to_footprint)


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
                                               angle_x_deg: float,
                                               angle_y_deg: float
                                               ) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions 
    for specified overlap and sidelap, and camera angles.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        angle_x_deg (float): camera's gimbal angle toward the x direction (in degrees)
        angle_y_deg (float): camera's gimbal angle toward the y direction (in degrees)

    Returns:
        float: The distance between images in the horizontal direction. (in meters)
        float: The distance between images in the vertical direction. (in meters)
    """

    [footprint_x, footprint_y] = compute_image_footprint_on_surface_with_angle(camera, dataset_spec.height,
                                                                               angle_x_deg, angle_y_deg)

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
        float: The speed at which the drone should move during photo capture, in meters per second.
    """
    # distance during active photo capture (meters)
    distance = compute_ground_sampling_distance(camera, dataset_spec.height) * allowed_movement_px
    time = dataset_spec.exposure_time_ms * 0.001 # exposure time (seconds)
    return distance / time


def compute_speed_during_photo_capture_with_angle(camera: Camera,
                                                  dataset_spec: DatasetSpec,
                                                  angle_x_deg: float = 0,
                                                  angle_y_deg: float = 0,
                                                  allowed_movement_px: float = 1,
                                                  ) -> float:
    """Compute the speed of drone during an active photo capture to prevent 
    more than 1px of motion blur. This function accounts for non-zero camera angles,
    but only one of the angles maybe non-zero.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        float: The speed at which the drone should move during photo capture, in meters per second.
    """

    # This function will help account for non-zero angle
    footprint_coord = compute_image_footprint_on_surface_coord(camera,
                                                               dataset_spec.height,
                                                               angle_x_deg,
                                                               angle_y_deg)
    distance_to_footprint = closest_distance_to_footprint(footprint_coord)
    
    # distance travelled during active photo capture with angle
    distance = compute_ground_sampling_distance(camera, distance_to_footprint) * allowed_movement_px

    time = dataset_spec.exposure_time_ms * 0.001 # exposure time (seconds)
    return distance / time


def generate_photo_plan_on_grid(camera: Camera, 
                                dataset_spec: DatasetSpec,
                                angle_x_deg: float = 0,
                                angle_y_deg: float = 0,
                                allowed_movement_px: float = 1,
                                ) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        angle_x_deg (float, optional): camera's gimbal angle toward the x direction (in degrees)
        angle_y_deg (float, optional): camera's gimbal angle toward the y direction (in degrees)
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    scan_dimension_x = dataset_spec.scan_dimension_x
    scan_dimension_y = dataset_spec.scan_dimension_y

    max_computed_distances = compute_distance_between_images_with_angle(camera, dataset_spec,
                                                                            angle_x_deg,
                                                                            angle_y_deg)
    # default allowed_movment_px = 1
    if angle_x_deg == 0 and angle_y_deg == 0:
        computed_speed = compute_speed_during_photo_capture(camera, dataset_spec, allowed_movement_px)
    else:
        computed_speed = compute_speed_during_photo_capture_with_angle(camera,
                                                                       dataset_spec,
                                                                       angle_x_deg=angle_x_deg,
                                                                       angle_y_deg=angle_y_deg,
                                                                       allowed_movement_px=allowed_movement_px)

    # calculate the number of photos in each direction
    # we need at least two waypoints in each direction at the start and end point of the scan area
    num_x = 1 + math.ceil(scan_dimension_x / max_computed_distances[0])
    num_y = 1 + math.ceil(scan_dimension_y / max_computed_distances[1])

    # actual distance between photos, based on actual number of photos to take
    distance_between_photos_x = scan_dimension_x / (num_x - 1)
    distance_between_photos_y = scan_dimension_y / (num_y - 1)

    # Calculate adjustments of waypoints in x and y directions based on angles
    adj_x = dataset_spec.height * math.tan(math.radians(angle_x_deg))
    adj_y = dataset_spec.height * math.tan(math.radians(angle_y_deg))

    result: T.List[Waypoint] = []
    for j in range(0, num_y):
        y = j * distance_between_photos_y # y position of camera
        y -= adj_y # adjust y position based on camera gimbal angle y
        for i in range(0, num_x):
            # use lawnmower pattern, so x travels in reverse direction if j is odd
            x = i * distance_between_photos_x if j%2==0 else (num_x - 1 - i) * distance_between_photos_x
            x -= adj_x # adjust x position based on camera gimbal angle x
            waypoint = Waypoint(x=x, y=y, z=dataset_spec.height,
                                speed=computed_speed, 
                                camera_angle_x_deg=angle_x_deg, camera_angle_y_deg=angle_y_deg)
            result.append(waypoint)

    return result


def time_between_waypoints(wp1: Waypoint, wp2: Waypoint, max_speed: float, max_acc: float) -> float:
    """Compute the time of travel between two waypoints.
    This function assumes the the drone is starting and ending with the same speed as in wp1.

    Args:
        wp1 (Waypoint): Waypoint of the starting position.
        wp2 (Waypoint): Waypoint of the ending position.
        max_speed (float): maximum speed (in meters per second)
        max_acc (float): maximum acceleration (in meters per second squared)

    Returns:
        float: the minimum flight time (in seconds).

    """

    # Distance between waypoints.
    d = math.sqrt((wp2.x - wp1.x)**2 + (wp2.y - wp1.y)**2)
    v1 = wp1.speed

    # d_acc is the distance needed to accelerate from initial speed to max_speed
    # Calculated with basic kinematic equation d_acc = v1 * t + (1/2) * a * t**2
    # with substitution of t using the relation of a * t = max_speed - v1.
    # where:
    #   d_acc = distance for accelerating from initial speed to max_speed
    #   v1 = initial speed
    #   a = acceleration
    #   t = time for acceleration
    d_acc = (v1 * (max_speed - v1) / max_acc
                 + (1/2) * (max_speed - v1)**2 / max_acc)

    # t_acc is the time needed for acceleration to max_speed
    t_acc = (max_speed - v1) / max_acc

    # If the drone is to attain max_speed and return back to initial speed,
    # the distance between waypoints must be at least 2 * d_acc
    # so there is enough distance to accelerate and decelerate.
    # Any additional distance would be travelled at max_speed.
    if d >= 2 * d_acc:
        total_time = 2 * t_acc + (d - 2 * d_acc) / max_speed
    else:
        # In this case the highest speed during acceleratino
        # can be calculated by solving the kinematic equation
        # where we set the distance for acceleration to be d/2.
        # d/2 = v1 * t + (1/2) a * t ** 2
        # where:
        #   t = (v2 - v1) / a
        #   v2 = highest speed attained
        #   v1 = initial speed
        #   a = acceleration
        v2 = math.sqrt(max_acc * d + v1 ** 2)
        total_time = 2 * (v2 - v1) / max_acc
    return total_time


def total_time_for_flightplan(flightplan: T.List[Waypoint],
                              max_speed: float,
                              max_acc: float) -> float:
    """Compute the time of travel for a flightplan (a list of Waypoints).

    Args:
        flightplan (T.List[Waypoint]): A flightplan represented as a list of Waypoints.
        max_speed (float): The maximum drone speed.
        max_acc (float): The maximum drone acceleration.

    Returns:
        float: the total flight time (in seconds).

    """
    total_time = 0.0
    for i in range(1, len(flightplan)):
        # Calculate and accumulate time between consecutive waypoints
        time = time_between_waypoints(flightplan[i-1], flightplan[i],
                                      max_speed, max_acc)
        total_time += time
    return total_time


def waypoint_count(flightplan: T.List[Waypoint]) -> tuple[int, int, int]:
    """Count the number of total waypoints, number of waypoints per row, and total number of rows

    Args:
        flightplan (T.List[Waypoint]): A flightplan represented as a list of Waypoints.

    Returns:
        int: Total number of waypoints
        int: Number of waypoints per row
        int: Number of rows

    """
    n = len(flightplan) # Total number of waypoints
    num_per_row = 0 # number of waypoints per row
    num_rows = 0 # number of rows

    # find number of waypoints in the first row
    i = 0
    for j in range(1, len(flightplan)):
        if flightplan[j].x > flightplan[i].x:
            i = j
        else:
            break
    num_per_row = i + 1

    # compute number rows
    num_rows = math.ceil(n / num_per_row)

    return n, num_per_row, num_rows
