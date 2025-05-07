"""Utility to visualize photo plans.
"""

import typing as T
import random

import plotly.graph_objects as go
import plotly.express as px

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface_coord

def plot_photo_plan(photo_plans: T.List[Waypoint]) -> go.Figure:
    """Plot the photo plan on a 2D grid.

    Args:
        photo_plans (T.List[Waypoint]): List of waypoints for the photo plan.

    Returns:
        T.Any: Plotly figure object.
    """
    plot_dot_size = 10
    plot_line_width = 3
    plot_range_min = 'auto'
    plot_range_max = 'auto'

    # create layout of the figure
    fig = go.Figure()
    fig.update_layout(
        title="Flight plan",
        xaxis_title="x-axis",
        yaxis_title="y-axis",
        width = 1000,
        height = 1000,
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis=dict(scaleanchor='y', scaleratio=1,
                   showline=True,
                   gridcolor='lightgray', dtick=50,
                   zerolinecolor='gray', zerolinewidth=1,
                   range=[plot_range_min, plot_range_max]
                   ),
        yaxis=dict(scaleanchor='x', scaleratio=1,
                   showline=True,
                   gridcolor='lightgray', dtick=50,
                   zerolinecolor='gray', zerolinewidth=1,
                   range=[plot_range_min, plot_range_max]
                   )
    )

    # create the graph for the flightplan
    # x, y coordinates of the waypoints
    x = []
    y = []
    for p in photo_plans:
        x.append(p.x)
        y.append(p.y)

    trace_path = go.Scatter(x=x, y=y,
                            mode="lines", 
                            line=dict(color="Red", width=plot_line_width),
                            name="Flight Path")
    trace_waypoint = go.Scatter(x=x, y=y,
                                mode="markers", 
                                marker=dict(color="Red", size=plot_dot_size),
                                name="Waypoint")

    fig.add_trace(trace_path)
    fig.add_trace(trace_waypoint)

    return fig


def plot_photo_plan_with_area(photo_plans: T.List[Waypoint],
                              camera: Camera,
                              dataset_spec: DatasetSpec,
                              title: str = "",
                              fill_alpha: float = 0.1,
                              line_alpha: float = 1.0,
                              jitter_amount: float=0,
                              darken_outline_at_ends: bool = True) -> go.Figure:
    """Plot the photo plan on a 2D grid, with the photo area indicated by rectangles.
    Since photo area overlap, we add jitter to make the rectangular area easier to see.

    Args:
        photo_plans (T.List[Waypoint]): List of waypoints for the photo plan.
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        title (str): The title string for the figure.
        fill_alpha (float): alpha value for the fill of rectangular region representing photo area.
        line_alpha (float): alpha value for the outline of rectangular region representing photo area.
        jitter_amount (float): photo area is offsetted by a randomly uniform amount
            between -jitter_amount to +jitter_amount, so that it is easier to see.
        darken_outline_at_ends (boolean): darken the outline of the area for the first and last area
            so they are easier to see.

    Returns:
        T.Any: Plotly figure object.
    """

    # add height information to the title
    title += f" (z={dataset_spec.height})"

    # begin with a figure of the flight plan
    fig = plot_photo_plan(photo_plans)
    fig.update_layout(title=title)

    num_colors = 30 # number of colors to use for the photo areas
    # Create color hsv color scale
    # note: px.colors.sample_colorscale() outputs a string like "rgb(255, 0, 0)"
    color_hsv = px.colors.sample_colorscale("hsv", [i / num_colors for i in range(num_colors)])

    # after string manipulation, each entry is a string like "rgba(255, 0, 0, 0.1)"
    fillcolor_with_alpha = ["rgba" + c[3:-1] + ", " + str(fill_alpha) + ")" for c in color_hsv]
    linecolor_with_alpha = ["rgba" + c[3:-1] + ", " + str(line_alpha) + ")" for c in color_hsv]

    # overlay translucent rectangles for photo areas for each waypoint
    for i, p in enumerate(photo_plans):        
        # Create jitter offset to display the rectangles representing the photo areas
        jitter = random.uniform(-jitter_amount, jitter_amount)
        footprint_coord = compute_image_footprint_on_surface_coord(camera,
                                                                    dataset_spec.height,
                                                                    p.camera_angle_x_deg,
                                                                    p.camera_angle_y_deg)
        # add rectangles for photo areas
        fig.add_shape(
            type="rect",
            x0=p.x + footprint_coord.pt_1.x + jitter,
            y0=p.y + footprint_coord.pt_1.y + jitter,
            x1=p.x + footprint_coord.pt_2.x + jitter,
            y1=p.y + footprint_coord.pt_2.y + jitter,
            line=dict(color=linecolor_with_alpha[i%num_colors], width = 1),
            fillcolor=fillcolor_with_alpha[i%num_colors]
        )

    # add cross hair to indicate centers of the photo
    photocenter_x = []
    photocenter_y = []
    for _, p in enumerate(photo_plans):
        photocenter_x.append(p.x + footprint_coord.pt_c.x)
        photocenter_y.append(p.y + footprint_coord.pt_c.y)
    # add crosshair at location of center of photos
    trace_center_of_photo = go.Scatter(x=photocenter_x, y=photocenter_y,
                    mode="markers",
                    marker=dict(size=10, symbol="cross-thin",
                                line=dict(width=1, color="Black")),
                    name="Center of Photo")
    fig.add_trace(trace_center_of_photo)

    # darken the area outline, crosshair, and waypoint of the first and last photo if requested
    if darken_outline_at_ends:
        for i, p in zip ( [0, -1], [photo_plans[0], photo_plans[-1]]):
            # darken outline of rectangle
            fig.add_shape(
                type="rect",
                x0=p.x + footprint_coord.pt_1.x + jitter,
                y0=p.y + footprint_coord.pt_1.y + jitter,
                x1=p.x + footprint_coord.pt_2.x + jitter,
                y1=p.y + footprint_coord.pt_2.y + jitter,
                line=dict(color="Black", width = 1),
                fillcolor=fillcolor_with_alpha[i%num_colors],
                showlegend=False
            )
            # darken the waypoint marker
            fig.add_trace(
                go.Scatter(x=[p.x], y=[p.y],
                    mode="markers",
                    marker=dict(size=10, symbol="circle-open",
                                color="Black",
                                line=dict(width=2, color="Green")),
                    showlegend=False
                )
            )
            # darken the cross hair for the center of photo area
            fig.add_trace(
                go.Scatter(x=[photocenter_x[i]], y=[photocenter_y[i]],
                    mode="markers",
                    marker=dict(size=20, symbol="cross-thin",
                                line=dict(width=2, color="Black")),
                    showlegend=False
                )
            )

    return fig
