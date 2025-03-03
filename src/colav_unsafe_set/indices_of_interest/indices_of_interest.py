from colav_unsafe_set.objects import (
    DynamicObstacle,
    DynamicObject,
    DynamicObstacleWithMetrics,
)
from scipy.spatial.distance import euclidean
from typing import List


def calc_I1(
    agent_vessel: DynamicObject,
    dynamic_obstacles: List[DynamicObstacleWithMetrics],
    dsf: float,
) -> List[DynamicObstacleWithMetrics]:
    """Calculate the set of obstacles within the distance safety threshold."""
    I1 = []
    for dynamic_obstacle in dynamic_obstacles:
        dist = euclidean(
            [
                agent_vessel.configuration.pose.position.x,
                agent_vessel.configuration.pose.position.y,
                agent_vessel.configuration.pose.position.z,
            ],
            [
                dynamic_obstacle.dynamic_obstacle.object.configuration.pose.position.x,
                dynamic_obstacle.dynamic_obstacle.object.configuration.pose.position.y,
                dynamic_obstacle.dynamic_obstacle.object.configuration.pose.position.z,
            ],
        ) - (
            agent_vessel.safety_radius
            + dynamic_obstacle.dynamic_obstacle.object.safety_radius
        )

        if dist <= dsf:
            I1.append(dynamic_obstacle)

    return I1


def calc_I2(
    I1: List[DynamicObstacle],
    dynamic_obstacles: List[DynamicObstacleWithMetrics],
    dsf: float,
) -> List[DynamicObstacleWithMetrics]:
    """Calculate the set of obstacles that are within the distance safety threshold."""
    I2 = []

    for operand_obstacle in I1:
        if not _validate_obstacle_list_subset([operand_obstacle], dynamic_obstacles):
            raise ValueError("Operand obstacle not in dynamic obstacles list.")

        for arg_obstacle in dynamic_obstacles:
            if operand_obstacle == arg_obstacle:
                continue

            dist = euclidean(
                [
                    operand_obstacle.dynamic_obstacle.object.configuration.pose.position.x,
                    operand_obstacle.dynamic_obstacle.object.configuration.pose.position.y,
                    operand_obstacle.dynamic_obstacle.object.configuration.pose.position.z,
                ],
                [
                    arg_obstacle.dynamic_obstacle.object.configuration.pose.position.x,
                    arg_obstacle.dynamic_obstacle.object.configuration.pose.position.y,
                    arg_obstacle.dynamic_obstacle.object.configuration.pose.position.z,
                ],
            ) - (
                operand_obstacle.dynamic_obstacle.object.safety_radius
                + arg_obstacle.dynamic_obstacle.object.safety_radius
            )

            if dist <= dsf:
                I2.append(operand_obstacle)

    return I2


def calc_I3(
    dynamic_obstacles_with_metrics: List[DynamicObstacleWithMetrics],
    dsf: float,
) -> List[DynamicObstacleWithMetrics]:
    """Calculate the set of obstacles with TCPA within the distance safety threshold."""
    I3 = []

    for dynamic_obstacle_with_metrics in dynamic_obstacles_with_metrics:
        if dynamic_obstacle_with_metrics.dcpa <= dsf:
            I3.append(dynamic_obstacle_with_metrics)

    return I3


def _validate_obstacle_list_subset(
    dynamic_obstacle_subset: List[DynamicObstacleWithMetrics],
    dynamic_obstacle_list: List[DynamicObstacleWithMetrics],
) -> bool:
    """Validate if all obstacles in the subset are in the main list."""
    if not dynamic_obstacle_subset:
        return True

    for subset_obstacle in dynamic_obstacle_subset:
        if subset_obstacle not in dynamic_obstacle_list:
            return False

    return True
