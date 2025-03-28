# import pytest
# from unittest.mock import MagicMock
# import sys
# import os

# # Make sure the module path is correctly added
# # sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src/colav_unsafe_set')))
# from colav_unsafe_set.objects import Agent, DynamicObstacleWithMetrics, DynamicObstacle
# from src.colav_unsafe_set.indices_of_interest import calc_I1


# @pytest.fixture
# def agent_vessel():
#     # Setup basic mock data for agent vessel
#     agent_vessel = Agent(
#         position=[float(0), float(0), float(0)],
#         orientation=[float(0), float(0), float(0), float(0.0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1)
#     )
#     return agent_vessel

# def test_calc_I1_outside_safety_threshold(agent_vessel):
#     # Create a mock dynamic obstacle that is outside the safety threshold
#     dynamic_obstacle = DynamicObstacle(
#         tag='mock_obstacle',
#         position=[float(10), float(10), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )
    
#     dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=1.0, tcpa=2.0)
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics]

#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that the obstacle is not within the unsafe set
#     assert len(I1) == 0


# def test_calc_I1_within_safety_threshold(agent_vessel):
#     # Create a mock dynamic obstacle that is within the safety threshold
#     dynamic_obstacle = DynamicObstacle(
#         tag='mock_obstacle',
#         position=[float(3), float(3), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )
    
#     dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=1.0, tcpa=2.0)
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics]

#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that the obstacle is within the unsafe set
#     assert len(I1) == 1


# def test_calc_I1_at_safety_threshold(agent_vessel):
#     # Create a mock dynamic obstacle that is exactly at the safety threshold
#     dynamic_obstacle = DynamicObstacle(
#         tag='mock_obstacle',
#         position=[float(5), float(5), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )
    
#     dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=1.0, tcpa=2.0)
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics]

#     # Define the safety threshold
#     dsf = 5.1
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that the obstacle is within the unsafe set
#     assert len(I1) == 1


# def test_calc_I1_multiple_obstacles(agent_vessel):
#     # Create multiple mock dynamic obstacles
#     dynamic_obstacle1 = DynamicObstacle(
#         tag='mock_obstacle1',
#         position=[float(3), float(3), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )
#     dynamic_obstacle2 = DynamicObstacle(
#         tag='mock_obstacle2',
#         position=[float(10), float(10), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )
    
#     # Wrap obstacles in DynamicObstacleWithMetrics
#     dynamic_obstacle_with_metrics1 = DynamicObstacleWithMetrics(dynamic_obstacle1, dcpa=1.0, tcpa=2.0)
#     dynamic_obstacle_with_metrics2 = DynamicObstacleWithMetrics(dynamic_obstacle2, dcpa=1.0, tcpa=2.0)
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics1, dynamic_obstacle_with_metrics2]

#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that only the first obstacle is within the unsafe set
#     assert len(I1) == 1


# def test_calc_I1_no_obstacles(agent_vessel):
#     # Test when no obstacles are provided
#     dynamic_obstacles = []

#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that the unsafe set is empty
#     assert len(I1) == 0


# def test_calc_I1_zero_safety_radius(agent_vessel):
#     # Modify the agent's safety radius to 0
#     agent_vessel.safety_radius = 0.0
#     dynamic_obstacle = DynamicObstacle(
#         tag='mock_obstacle',
#         position=[float(3), float(3), float(0)],
#         orientation=[float(0), float(0), float(0), float(0)],
#         yaw_rate=float(0.0),
#         velocity=float(10),
#         safety_radius=float(1.0)
#     )

#     dynamic_obstacle_with_metrics = DynamicObstacleWithMetrics(dynamic_obstacle, dcpa=1.0, tcpa=2.0)
    
#     # List of obstacles for the test
#     dynamic_obstacles = [dynamic_obstacle_with_metrics]

#     # Define the safety threshold
#     dsf = 5.0
    
#     # Call the function to calculate the unsafe set
#     I1 = calc_I1(agent_vessel, dynamic_obstacles, dsf)
    
#     # Assert that the obstacle is within the unsafe set (since agent has no safety buffer)
#     assert len(I1) == 1

# def main():
#     pytest.main()


# if __name__ == '__main__':
#     main()
