from unsafe_set_gen.risk_assesment import calc_dcpa_and_tcpa
from unsafe_set_gen.objects import DynamicObject, Configuration
import pytest

test_ids = [
    "No velocity, same position",
    "Same velocity, different direction",
    "Different velocities, intersecting paths",
    "Different velocities, no interception (moving in parallel)",
    "High-speed approach, small time to collision",
    "Moving perpendicular to each other (no interception)",
]
testdata = [
    # Test 1: No velocity, same position
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=0.0),
                ),
                yaw_rate=0.0,
                velocity=0.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=0.0),
                ),
                yaw_rate=0.0,
                velocity=0.0,
            ),
            safety_radius=5.0,
        ),
        0.0,  # expected dcpa
        0.0,  # expected tcpa
    ),
    # Test 2: Same velocity, different direction
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=100.0, y=100.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        141.4213562373095,  # expected dcpa
        14.14213562373095,  # expected tcpa
    ),
    # Test 3: Different velocities, intersecting paths
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=100.0, y=100.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=15.0,
            ),
            safety_radius=5.0,
        ),
        141.4213562373095,  # expected dcpa
        9.20753323377405,  # expected tcpa
    ),
    # Test 4: Different velocities, no interception (moving in parallel)
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=100.0, y=200.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        100.0,  # expected dcpa
        float("inf"),  # expected tcpa (no interception)
    ),
    # Test 5: High-speed approach, small time to collision
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=50.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=1000.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=50.0,
            ),
            safety_radius=5.0,
        ),
        1000.0,  # expected dcpa
        20.0,  # expected tcpa
    ),
    # Test 6: Moving perpendicular to each other (no interception)
    (
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=100.0, y=0.0, z=0.0),
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                yaw_rate=0.0,
                velocity=10.0,
            ),
            safety_radius=5.0,
        ),
        100.0,  # expected dcpa
        float(-0.0),  # expected tcpa (no interception)
    ),
]


@pytest.mark.parametrize(
    "agent_object, target_object, expected_dcpa, expected_tcpa", testdata, ids=test_ids
)
def test_calc_dcpa_and_tcpa(
    agent_object: DynamicObject,
    target_object: DynamicObject,
    expected_dcpa: float,
    expected_tcpa: float,
):
    dcpa, tcpa = calc_dcpa_and_tcpa(agent_object, target_object)

    print (f"actual: {dcpa}, expected: {expected_dcpa}")
    print (f"actual: {tcpa}, expected: {expected_tcpa}")

    assert dcpa == pytest.approx(expected_dcpa, rel=1e-9)
    assert tcpa == pytest.approx(expected_tcpa, rel=1e-9)
