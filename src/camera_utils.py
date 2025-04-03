"""Utility functions for the camera model.
"""
import numpy as np

from src.data_model import Camera

def compute_focal_length_in_mm(camera: Camera) -> np.ndarray:
    """Computes the focal length in mm for the given camera

    Args:
        camera (Camera): the camera model.

    Returns:
        np.ndarray: [fx, fy] in mm.
    """
    raise NotImplementedError() 
    # Note(Ayush): Solution provided by project leader.
    # pixel_to_mm_x = camera.sensor_size_x_mm / camera.image_size_x_px
    # pixel_to_mm_y = camera.sensor_size_y_mm / camera.image_size_y_px

    # return np.array([camera.fx * pixel_to_mm_x, camera.fy * pixel_to_mm_y])

def project_world_point_to_image(camera: Camera, point: np.ndarray) -> np.ndarray:
    """Project a 3D world point into the image coordinates.

    Args:
        camera (Camera): the camera model
        point (np.ndarray): the 3D world point

    Returns:
        np.ndarray: [u, v] pixel coordinates corresponding to the point.
    """
    x = camera.fx * point[0] / point[2]
    y = camera.fy * point[1] / point[2]
    u = x + camera.cx
    v = y + camera.cy
    return np.array([u, v])


def compute_image_footprint_on_surface(camera: Camera, distance_from_surface: float) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).

    Returns:
        np.ndarray: [footprint_x, footprint_y] in meters.
    """
    # reversing the calculation in project_world_point_to_image() for an image,
    # using the two opposing corners of the image.
    # (u0, v0) is the (x, y) coordinate in px of the upper left corner of the image.
    # (u1, v1) is the (x, y) coordinate in px of the lower right corner of the image.
    u0 = 0
    v0 = 0
    u1 = u0 + camera.image_size_x_px
    v1 = v0 + camera.image_size_y_px

    point_0 = reproject_image_point_to_world(camera, distance_from_surface, np.array([u0, v0]))
    point_1 = reproject_image_point_to_world(camera, distance_from_surface, np.array([u1, v1]))

    # The footprint is the difference in worldpoint X and difference in worldpoint Y
    # of the two points.
    return np.array([point_1[0] - point_0[0], point_1[1] - point_0[1]])


def compute_ground_sampling_distance(camera: Camera, distance_from_surface: float) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
    
    Returns:
        float: the GSD in meters (smaller among x and y directions).
    """
    # footprint along each direction per pixel is the ground sampling distance in that direction
    footprint = compute_image_footprint_on_surface(camera, distance_from_surface)

    # calculate gsd in x and y directions
    gsd_x = footprint[0]/camera.image_size_x_px
    gsd_y = footprint[1]/camera.image_size_y_px

    # return the smaller of the two gsd values
    return min(gsd_x, gsd_y)


def reproject_image_point_to_world(camera: Camera,
                                   distance_from_surface: float,
                                   point: np.ndarray) -> np.ndarray:
    """Reproject an image point captured by the camera back to a world point at a given distance.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
        point (float): image point [u, v] (in px).

    Returns:
        np.ndarray: world point [x, y, z] (in m).
    """
    # image point (u, v)
    u = point[0]
    v = point[1]

    # calculate world point (x, y) corresponding to image point
    x = (u - camera.cx) / camera.fx * distance_from_surface
    y = (v - camera.cy) / camera.fy * distance_from_surface

    return np.array([x, y, distance_from_surface])
