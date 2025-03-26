# import pytest
# from unittest.mock import MagicMock, patch
# from src.colav_unsafe_set import create_unsafe_set
# from src.colav_unsafe_set.unsafe_set import _gen_uIoI_convhull, _generate_circle_vertices, _calc_dynamic_obstacles_tcpa_dcpa
# from src.colav_unsafe_set.indices_of_interest import (
#     calc_I1,
#     calc_I2,
#     calc_I3,
#     unionise_indices_of_interest
# )
# from src.colav_unsafe_set.objects import (
#     Configuration,
#     DynamicObject,
#     DynamicObstacle,
#     DynamicObstacleWithMetrics
# )
# from scipy.spatial import ConvexHull
# import numpy as np


# @pytest.fixture
# def agent_vessel():
#     # Mock a dynamic object for the agent vessel
#     return MagicMock(spec=DynamicObject)


# @pytest.fixture
# def dynamic_obstacle():
#     # Mock a dynamic obstacle
#     return MagicMock(spec=DynamicObstacle)


# @pytest.fixture
# def dynamic_obstacle_with_metrics(dynamic_obstacle):
#     # Create a DynamicObstacleWithMetrics object
#     return DynamicObstacleWithMetrics(
#         dynamic_obstacle=dynamic_obstacle, dcpa=1.0, tcpa=2.0
#     )


# def test_create_unsafe_set_no_dynamic_obstacles(agent_vessel):
#     """Test if create_unsafe_set returns an empty list when no obstacles exist."""
#     dynamic_obstacles = []
#     dsf = 1.0
#     result = create_unsafe_set(agent_vessel, dynamic_obstacles, dsf)

#     # Assert that the result is an empty list
#     assert result == []


# def test_create_unsafe_set_with_dynamic_obstacles(agent_vessel, dynamic_obstacle):
#     """Test the function with some dynamic obstacles."""
#     dynamic_obstacles = [dynamic_obstacle]
#     dsf = 1.0

#     # Mocking the functions that are called within create_unsafe_set
#     with patch('_calc_dynamic_obstacles_tcpa_dcpa', return_value=[dynamic_obstacle]) as mock_calc_tcpa_dcpa, \
#          patch('calc_I1', return_value=[dynamic_obstacle]), \
#          patch('calc_I2', return_value=[dynamic_obstacle]), \
#          patch('calc_I3', return_value=[dynamic_obstacle]), \
#          patch('unionise_indices_of_interest', return_value=[dynamic_obstacle]):
        
#         result = create_unsafe_set(agent_vessel, dynamic_obstacles, dsf)
        
#     # Assert that the result is a list
#     assert isinstance(result, list)

# def test_generate_circle_vertices():
#     """Test if circle vertices are correctly generated."""
#     centroid = [0, 0]
#     radius = 1.0
#     result = _generate_circle_vertices(centroid, radius)

#     # Assert the result is a list with correct number of points (default is 10)
#     assert len(result) == 10
#     assert isinstance(result[0], list)
#     assert len(result[0]) == 2  # Each point should be a 2D point (x, y)


# def test_gen_uIoI_convhull_with_no_vertices():
#     """Test _gen_uIoI_convhull with no vertices."""
#     result = _gen_uIoI_convhull([])

#     # Assert that the result is an empty list
#     assert result == []


# def test_gen_uIoI_convhull_with_some_vertices(dynamic_obstacle_with_metrics):
#     """Test _gen_uIoI_convhull with some valid dynamic obstacles."""
#     result = _gen_uIoI_convhull([dynamic_obstacle_with_metrics])

#     # Assert that the result is a list (ConvexHull vertices indices)
#     assert isinstance(result, list)


# def test_calc_dynamic_obstacles_tcpa_dcpa(dynamic_obstacle):
#     """Test _calc_dynamic_obstacles_tcpa_dcpa."""
#     dynamic_obstacles = [dynamic_obstacle]
#     agent_vessel = MagicMock(spec=DynamicObject)

#     with patch('yourmodule.calc_cpa', return_value=[1.0, 2.0]) as mock_calc_cpa:
#         result = _calc_dynamic_obstacles_tcpa_dcpa(agent_vessel, dynamic_obstacles)

#     # Assert that the result contains the correct DynamicObstacleWithMetrics object
#     assert isinstance(result[0], DynamicObstacleWithMetrics)
#     assert result[0].dcpa == 1.0
#     assert result[0].tcpa == 2.0


# def test_create_unsafe_set_with_no_obstacles_in_union(agent_vessel, dynamic_obstacle, dynamic_obstacle_with_metrics):
#     """Test create_unsafe_set with no obstacles after unionisation."""
#     # Mock the functions to return empty results for unionise_indices_of_interest
#     with patch('yourmodule._calc_dynamic_obstacles_tcpa_dcpa', return_value=[dynamic_obstacle_with_metrics]), \
#          patch('yourmodule.unionise_indices_of_interest', return_value=[]):
        
#         result = create_unsafe_set(agent_vessel, [dynamic_obstacle], 1.0)

#     # Assert that the result is an empty list (since there are no unionised obstacles)
#     assert result == []


# def test_create_unsafe_set_with_valid_union(agent_vessel, dynamic_obstacle_with_metrics):
#     """Test create_unsafe_set with valid union."""
#     # Mock the necessary functions
#     with patch('yourmodule._calc_dynamic_obstacles_tcpa_dcpa', return_value=[dynamic_obstacle_with_metrics]), \
#          patch('yourmodule.calc_I1', return_value=[dynamic_obstacle_with_metrics]), \
#          patch('yourmodule.calc_I2', return_value=[dynamic_obstacle_with_metrics]), \
#          patch('yourmodule.calc_I3', return_value=[dynamic_obstacle_with_metrics]), \
#          patch('yourmodule.unionise_indices_of_interest', return_value=[dynamic_obstacle_with_metrics]):
        
#         result = create_unsafe_set(agent_vessel, [dynamic_obstacle_with_metrics], 1.0)

#     # Assert that the result is a list (it should contain vertices of the convex hull)
#     assert isinstance(result, list)


def main():
    pytest.main()


if __name__ == '__main__':
    main()
