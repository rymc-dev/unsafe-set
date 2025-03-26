import pytest
from unittest.mock import MagicMock
from colav_unsafe_set.objects import DynamicObject, DynamicObstacleWithMetrics, Configuration
from src.colav_unsafe_set.indices_of_interest import calc_I3

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

def test_calc_I3_outside_safety_threshold(agent_vessel):
    # Create a mock dynamic obstacle that is outside the safety threshold
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
    
    dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=6.0, tcpa=2.0)
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics]

    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate the unsafe set
    I3 = calc_I3(dynamic_obstacles, dsf)
    
    # Assert that the obstacle is not within the safety threshold
    assert len(I3) == 0

def test_calc_I3_within_safety_threshold(agent_vessel):
    # Create a mock dynamic obstacle that is within the safety threshold
    dynamic_obstacle = DynamicObject(
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
    
    dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=4.0, tcpa=2.0)
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics]

    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate the unsafe set
    I3 = calc_I3(dynamic_obstacles, dsf)
    
    # Assert that the obstacle is within the safety threshold
    assert len(I3) == 1

def test_calc_I3_at_safety_threshold(agent_vessel):
    # Create a mock dynamic obstacle that is exactly at the safety threshold
    dynamic_obstacle = DynamicObject(
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
    
    dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=5.0, tcpa=2.0)
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics]

    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate the unsafe set
    I3 = calc_I3(dynamic_obstacles, dsf)
    
    # Assert that the obstacle is within the safety threshold
    assert len(I3) == 1

def test_calc_I3_multiple_obstacles(agent_vessel):
    # Create multiple mock dynamic obstacles
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
    dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=4.0, tcpa=2.0)
    dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=6.0, tcpa=2.0)
    
    # List of obstacles for the test
    dynamic_obstacles = [dynamic_obstacle_with_metrics1, dynamic_obstacle_with_metrics2]

    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate the unsafe set
    I3 = calc_I3(dynamic_obstacles, dsf)
    
    # Assert that only the first obstacle is within the safety threshold
    assert len(I3) == 1

def test_calc_I3_no_obstacles(agent_vessel):
    # Test when no obstacles are provided
    dynamic_obstacles = []

    # Define the safety threshold
    dsf = 5.0
    
    # Call the function to calculate the unsafe set
    I3 = calc_I3(dynamic_obstacles, dsf)
    
    # Assert that the unsafe set is empty
    assert len(I3) == 0
