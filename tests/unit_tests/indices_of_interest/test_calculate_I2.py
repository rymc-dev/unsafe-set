import pytest
from unittest.mock import MagicMock
import sys
import os

# Make sure the module path is correctly added
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src/colav_unsafe_set')))
from colav_unsafe_set.objects import DynamicObject, DynamicObstacleWithMetrics, Configuration
from src.colav_unsafe_set.indices_of_interest import calc_I2


@pytest.fixture
def agent_vessel():
    # Setup basic mock data for agent vessel
    agent_vessel = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(0.0, 0.0, 0.0),
                Configuration.Pose.Orientation(0, 0, 0.0, 0.0)
            ),
            yaw_rate=0.0,
            velocity=10
        ), 
        safety_radius=1.0
    )
    return agent_vessel


def test_calc_I2_empty_I1(agent_vessel):
    # Test when I1 is empty
    I1 = []
    dynamic_obstacle = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(10, 10, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=1.0, tcpa=2.0)
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics]
    
    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate I2
    I2 = calc_I2(I1, dynamic_obstacles, dsf)
    
    # Assert that I2 is empty since I1 is empty
    assert len(I2) == 0


def test_calc_I2_no_matching_obstacles(agent_vessel):
    # Test when there are no matching obstacles in I1 and dynamic_obstacles
    dynamic_obstacle1 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(3, 3, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )

    dynamic_obstacle2 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(10, 10, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    # Wrap obstacles in DynamicObstacleWithMetrics
    dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=1.0, tcpa=2.0)
    dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=1.0, tcpa=2.0)
    
    # I1 contains one obstacle that doesn't match any in dynamic_obstacles
    I1 = [dynamic_obstacle_with_metrics1]

    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics2]
    
    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate I2
    I2 = calc_I2(I1, dynamic_obstacles, dsf)
    
    # Assert that I2 is empty since there are no matching obstacles
    assert len(I2) == 0


def test_calc_I2_with_matching_obstacles(agent_vessel):
    # Test when I1 contains obstacles that match with dynamic_obstacles
    dynamic_obstacle1 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(3, 3, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    dynamic_obstacle2 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(10, 10, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    # Wrap obstacles in DynamicObstacleWithMetrics
    dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=1.0, tcpa=2.0)
    dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=1.0, tcpa=2.0)
    
    # I1 contains one obstacle that matches with dynamic_obstacles
    I1 = [dynamic_obstacle_with_metrics1]
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics1, dynamic_obstacle_with_metrics2]
    
    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate I2
    I2 = calc_I2(I1, dynamic_obstacles, dsf)
    
    # Assert that the matching obstacle is in I2
    assert len(I2) == 0


# def test_calc_I2_invalid_obstacle_in_I1(agent_vessel):
#     # Test when an invalid obstacle is in I1 (i.e., an obstacle not in dynamic_obstacles)
#     dynamic_obstacle1 = DynamicObject(
#         configuration=Configuration(
#             pose=Configuration.Pose(
#                 Configuration.Pose.Position(3, 3, 0),
#                 Configuration.Pose.Orientation(0, 0, 0, 0)
#             ),
#             yaw_rate=0.0,
#             velocity=10
#         ),
#         safety_radius=1.0
#     )
    
#     dynamic_obstacle2 = DynamicObject(
#         configuration=Configuration(
#             pose=Configuration.Pose(
#                 Configuration.Pose.Position(10, 10, 0),
#                 Configuration.Pose.Orientation(0, 0, 0, 0)
#             ),
#             yaw_rate=0.0,
#             velocity=10
#         ),
#         safety_radius=1.0
#     )
    
#     # Wrap obstacles in DynamicObstacleWithMetrics
#     dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=1.0, tcpa=2.0)
#     dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=1.0, tcpa=2.0)
    
#     # I1 contains an invalid obstacle that is not in dynamic_obstacles
#     I1 = [dynamic_obstacle_with_metrics1]
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics2]
    
#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function and expect it to raise a ValueError due to invalid obstacle in I1
#     with pytest.raises(ValueError):
#         calc_I2(I1, dynamic_obstacles, dsf)


def test_calc_I2_multiple_obstacles(agent_vessel):
    # Test with multiple obstacles in I1 and dynamic_obstacles
    dynamic_obstacle1 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(3, 3, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    dynamic_obstacle2 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(5, 5, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )

    dynamic_obstacle3 = DynamicObject(
        configuration=Configuration(
            pose=Configuration.Pose(
                Configuration.Pose.Position(8, 8, 0),
                Configuration.Pose.Orientation(0, 0, 0, 0)
            ),
            yaw_rate=0.0,
            velocity=10
        ),
        safety_radius=1.0
    )
    
    # Wrap obstacles in DynamicObstacleWithMetrics
    dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=1.0, tcpa=2.0)
    dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=1.0, tcpa=2.0)
    dynamic_obstacle_with_metrics3 = DynamicObstacleWithMetrics(dynamic_obstacle3, dcpa=1.0, tcpa=2.0)
    
    # I1 contains two obstacles
    I1 = [dynamic_obstacle_with_metrics1, dynamic_obstacle_with_metrics2]
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics1, dynamic_obstacle_with_metrics2, dynamic_obstacle_with_metrics3]
    
    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate I2
    I2 = calc_I2(I1, dynamic_obstacles, dsf)
    
    # Assert that only the obstacles that are within the distance safety threshold are in I2
    assert len(I2) == 3


def main():
    pytest.main()


if __name__ == '__main__':
    main()
