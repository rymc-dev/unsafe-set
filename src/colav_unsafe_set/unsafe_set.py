from .objects import (
    DynamicObstacle,
    DynamicObject,
    DynamicObstacleWithMetrics,
)
from .indices_of_interest import *
from .risk_assessment import calc_cpa
from typing import List
from scipy.spatial import ConvexHull
import numpy as np


def create_unsafe_set(
    agent_vessel: DynamicObject,
    dynamic_obstacles: List[DynamicObstacle],
    dsf: float,
):
    """create_unsafe_set"""
    dynamic_obstacle_with_metrics = _calc_dynamic_obstacles_tcpa_dcpa(
        agent_vessel=agent_vessel, dynamic_obstacles=dynamic_obstacles
    )
    I1 = calc_I1(
        agent_vessel=agent_vessel,
        dynamic_obstacles=dynamic_obstacle_with_metrics,
        dsf=dsf,
    )
    I2 = calc_I2(I1=I1, dynamic_obstacles=dynamic_obstacle_with_metrics, dsf=dsf)
    I3 = calc_I3(
        dynamic_obstacle_with_metrics,
        dsf=dsf,
    )

    uIoI = unionise_indices_of_interest(I1, I2, I3)
    if uIoI == []:
        return []

    return _gen_uIoI_convhull(uIoI)


def _gen_uIoI_convhull(uIoI: List[DynamicObstacleWithMetrics]):
    """Generate the convex hull of the unionised indices of interest."""
    unsafe_set_vertices = []
    for dynamic_obstacle in uIoI:
        unsafe_set_vertices.extend(
            _generate_circle_vertices(
                centroid=[
                    dynamic_obstacle.dynamic_obstacle.object.configuration.pose.position.x,
                    dynamic_obstacle.dynamic_obstacle.object.configuration.pose.position.y,
                ],
                radius=dynamic_obstacle.dynamic_obstacle.object.safety_radius,
            )
        )
    if unsafe_set_vertices == []:
        return []

    hull = ConvexHull(unsafe_set_vertices)
    return hull.vertices


def _generate_circle_vertices(centroid, radius, num_points=10):
    """Generate 2D circle vertices in the XY plane."""
    x_c, y_c = centroid  # Extract centroid coordinates
    theta = np.linspace(0, 2 * np.pi, num_points)  # Generate angles

    # Compute x, y coordinates of the circle
    x = x_c + radius * np.cos(theta)
    y = y_c + radius * np.sin(theta)

    # Stack into a list of 2-length arrays
    circle_vertices = np.column_stack((x, y))

    return circle_vertices.tolist()  # Convert to a list of lists


def _calc_dynamic_obstacles_tcpa_dcpa(agent_vessel, dynamic_obstacles):

    dynamic_obstacles_with_metrics = []
    for dynamic_obstacle in dynamic_obstacles:
        [dcpa, tcpa] = calc_cpa(agent_vessel, dynamic_obstacle.object)
        dynamic_obstacles_with_metrics.append(
            DynamicObstacleWithMetrics(
                dynamic_obstacle=dynamic_obstacle, dcpa=dcpa, tcpa=tcpa
            )
        )
    return dynamic_obstacles_with_metrics
