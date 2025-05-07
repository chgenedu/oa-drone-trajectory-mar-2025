"""Utility functions for the camera model.
"""
import numpy as np
import math

from src.data_model import Camera, Coordinate, FootprintCoord

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


def compute_image_footprint_on_surface_with_angle(camera: Camera,
                                                  distance_from_surface: float,
                                                  angle_x_deg: float = 0,
                                                  angle_y_deg: float = 0) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
        angle_x_deg (float): camera's gimbal angle toward the x direction (in degrees)
        angle_y_deg (float): camera's gimbal angle toward the y direction (in degrees)        

    Returns:
        np.ndarray: [footprint_x, footprint_y] in meters.
    """

    footprint = compute_image_footprint_on_surface_coord(camera, distance_from_surface,
                                                               angle_x_deg, angle_y_deg)
    footprint_x = footprint.pt_2.x - footprint.pt_1.x
    footprint_y = footprint.pt_2.y - footprint.pt_1.y

    return np.array([footprint_x, footprint_y])


def compute_image_footprint_on_surface_coord(camera: Camera,
                                             distance_from_surface: float,
                                             angle_x_deg: float = 0,
                                             angle_y_deg: float = 0) -> FootprintCoord:
    """Compute the footprint's coordinate relative to the camera's position,
    where the footprint is the image captured by the camera 
    at a given distance from the surface.

    It is assumed that only one of angle_x_deg and angle_y_deg may be non-zero.
    If both are non-zero, then an exception will be raised.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
        angle_x_deg (float): camera's gimbal angle toward the x direction (in degrees)
        angle_y_deg (float): camera's gimbal angle toward the y direction (in degrees)

    Returns:
        FootprintCoord: the coordinates of the footprint relative to the camera's position.
        Note: Since the camera is aiming downward toward the ground, the z coordinate of 
        a point on the ground will be equal to -distance_from_surface, which is a negative value.

        Note: A simplifying assumption is made that the footprint will remain a rectangular region,
        But with non-zero gimbal angle, there will be a distortion so the footprint may not actually 
        be rectangular in reality.
    """
    # Check to make sure that at most one angle is non-zero
    if angle_x_deg != 0 and angle_y_deg != 0:
        raise NotImplementedError("Not Implemented: Currenly only one of the camera angles may be non-zero.")

    # convert to radians
    angle_x = math.radians(angle_x_deg) # convert deg to rad
    angle_y = math.radians(angle_y_deg) # convert deg to rad

    # fov_x is field of view in the x direction.
    [fov_deg_x, fov_deg_y] = fov(camera)
    [fov_x, fov_y] = [math.radians(fov_deg_x), math.radians(fov_deg_y)]

    # For either x or y direction, there are two edges of the rectangular region.
    # We will use x-direction to explain the calculation below. (y-direction is analogous.)
    # All coordinates will be relative to camera's position.
    # One edge will have coordinate greater than the other edge.
    # The edge with greater coordinate value will have coordinate equal to
    # x_1 = math.tan(angle_x + fov_x/2) * h
    # The edge with lesser coordinate value will have coordinate equal to
    # x_0 = math.tan(angle_x - fov_x/2) * h
    # The coordinate on the ground where the camera aims at will be equal to
    # math.tan(angle_x) * h

    h = distance_from_surface
    # Bottom Left corner of the footprint
    point_bottomleft = Coordinate(math.tan(angle_x - fov_x/2)*h, math.tan(angle_y - fov_y/2)*h, -h)
    # Top Right corner
    point_topright = Coordinate(math.tan(angle_x + fov_x/2)*h, math.tan(angle_y + fov_y/2)*h, -h)
    # Top Left corner
    point_topleft = Coordinate(math.tan(angle_x - fov_x/2)*h, math.tan(angle_y + fov_y/2)*h, -h)
    # Bottom Right corner
    point_bottomright = Coordinate(math.tan(angle_x + fov_x/2)*h, math.tan(angle_y - fov_y/2)*h, -h)
    # The point where camera is aiming at
    point_c = Coordinate(math.tan(angle_x)*h, math.tan(angle_y)*h, -h)

    return FootprintCoord(point_bottomleft, point_topright, point_topleft, point_bottomright, point_c)


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


# This is a helper function created for Week 4 Bonus function
# compute_distance_between_images_with_angle() in src/plan_computation.py.
# This function calculates the field of view of a camera.
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


# Helper function to compute the closest distance to the footprint from the camera.
def closest_distance_to_footprint(footprint: FootprintCoord) -> float:
    """Compute the closest distance to a footprint.
    
    Args:
        footprint (FootprintCoord): The information about the coordinates of the footprint, 
            relative to camera's current position.

    Returns:
        float: The closest distance between a the footprint and the camera's current position.
    """

    # height h is the vertical distance from the camera to the footprint.
    h = math.fabs(footprint.pt_c.z)

    # There are 3 cases.
    # Case 1: The camera is directly over the footprint.
    # Then the closest distance is simply the z distance (in absolute value).
    # This happens when pt_1.x and pt_0.x have opposite signs or if one of them is zero
    # (and similarly for y coordinate).
    if footprint.pt_1.x * footprint.pt_2.x <=0 and footprint.pt_1.y * footprint.pt_2.y <= 0:
        return math.fabs(h)
    # Case 2A: If x coordinate of the camera is between the footprint's x_boundaries
    # but y coordinate is not, then the closest ground distance is the y distance
    # of the closest edge parallel to the y axis.
    # We can then use the Pythagorean Theorem to compute the distance from that ground point
    # on the edge of the footprint to the camera.
    elif footprint.pt_1.x * footprint.pt_2.x <=0 and footprint.pt_1.y * footprint.pt_2.y > 0:
        ground_distance_y = min(math.fabs(footprint.pt_1.y), math.fabs(footprint.pt_2.y))
        return math.sqrt(ground_distance_y ** 2 + h ** 2)
    # Case 2B: Simily to Case 2A except it is for y coordinates.
    elif footprint.pt_1.y * footprint.pt_2.y <=0 and footprint.pt_1.x * footprint.pt_2.x > 0:
        ground_distance_x = min(math.fabs(footprint.pt_1.x), math.fabs(footprint.pt_2.x))
        return math.sqrt(ground_distance_x ** 2 + h ** 2)
    # Case 3: In this case the closest point on footprint must be one of the corners.
    # So we take the minimum of the ground distances to the corners and then use
    # Pythagorean Theorem to find the distance from camera to the closest ground point.
    else:
        # distances to the 4 corners
        def d(pt: Coordinate) -> float: # helper function
            return math.sqrt(pt.x ** 2 + pt.y ** 2)
        corner_1 = d(footprint.pt_1)
        corner_2 = d(footprint.pt_2)
        corner_3 = d(footprint.pt_3)
        corner_4 = d(footprint.pt_4)
        closest_corner =  min(corner_1, corner_2, corner_3, corner_4)
        return math.sqrt(closest_corner ** 2 + h ** 2)
