import pytest
from colav_unsafe_set.indices_of_interest import calc_I1, calc_I2, calc_I3
from colav_unsafe_set.objects import (
    DynamicObstacle,
    DynamicOB,
    DynamicObject,
    Configuration,
)


# obstacles = [
#     # Test 1: No velocity, same position
#     (
#         DynamicObstacle(
#             id = 'obstacle_1',
#         DYnamicOBstacleWith = DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=0.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=0.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=0.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=0.0,
#             ),
#             safety_radius=5.0,
#         ),
#         0.0,  # expected dcpa
#         0.0,  # expected tcpa
#     ),
#     # Test 2: Same velocity, different direction
#     (
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=100.0, y=100.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         141.4213562373095,  # expected dcpa
#         14.14213562373095,  # expected tcpa
#     ),
#     # Test 3: Different velocities, intersecting paths
#     (
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=100.0, y=100.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=15.0,
#             ),
#             safety_radius=5.0,
#         ),
#         141.4213562373095,  # expected dcpa
#         9.20753323377405,  # expected tcpa
#     ),
#     # Test 4: Different velocities, no interception (moving in parallel)
#     (
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=100.0, y=200.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         100.0,  # expected dcpa
#         float("inf"),  # expected tcpa (no interception)
#     ),
#     # Test 5: High-speed approach, small time to collision
#     (
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=50.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=1000.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=50.0,
#             ),
#             safety_radius=5.0,
#         ),
#         1000.0,  # expected dcpa
#         20.0,  # expected tcpa
#     ),
#     # Test 6: Moving perpendicular to each other (no interception)
#     (
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=0.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         DynamicObject(
#             configuration=DynamicObject.Configuration(
#                 pose=DynamicObject.Pose(
#                     position=DynamicObject.Position(x=100.0, y=0.0, z=0.0),
#                     orientation=DynamicObject.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
#                 ),
#                 yaw_rate=0.0,
#                 velocity=10.0,
#             ),
#             safety_radius=5.0,
#         ),
#         100.0,  # expected dcpa
#         float("inf"),  # expected tcpa (no interception)
#     ),
# ]


# @pytest.mark.parametrize(
#     "input_value, expected_output",
#     [
#         (10, 100),  # Example test case for calc_I1
#         (5, 25),    # Example test case for calc_I2
#         (2, 4)      # Example test case for calc_I3
#     ]
# )
# def test_calc_I1(input_value, expected_output):
# def test_calc_I1():
#     # You should replace calc_I1 with your actual function and its logic
#     # result = calc_I1(input_value)  # Assume calc_I1 processes input_value
#     assert 0 == 0
