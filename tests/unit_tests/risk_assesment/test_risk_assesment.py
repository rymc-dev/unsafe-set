from src.colav_unsafe_set.risk_assessment import calc_cpa
from src.colav_unsafe_set.objects import DynamicObject, DynamicObstacle, Configuration
import pytest
from pytest import approx

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
        float('nan'),  # expected dcpa
        float('inf'),  # expected tcpa
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
                velocity=10.0,  # Object 1 moving at 10.0
            ),
            safety_radius=5.0,
        ),
        DynamicObject(
            configuration=Configuration(
                pose=Configuration.Pose(
                    position=Configuration.Pose.Position(x=100.0, y=100.0, z=0.0),  # Starting point moved
                    orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=-0.707, w=0.707),  # Object 2 now moves towards Object 1
                ),
                yaw_rate=0.0,
                velocity=10.0,  # Object 2 moving at 10.0
            ),
            safety_radius=5.0,
        ),
        0.021361075105734158,  # expected DCPA (they will intersect)
        10.001510684305536,  # expected TCPA (they will intersect in 14.14 seconds)
    ),
    # Test 4: Different velocities, no interception
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
        223.60679774997897,  # expected dcpa
        22.360679774997898,  # expected tcpa (no interception)
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
        10.0,  # expected tcpa (no interception)
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
    dcpa, tcpa = calc_cpa(agent_object, target_object)

    print(f"Actual DCPA: {dcpa}, Expected DCPA: {expected_dcpa}")
    print(f"Actual TCPA: {tcpa}, Expected TCPA: {expected_tcpa}")
    import math

    if math.isnan(expected_dcpa):
        assert math.isnan(dcpa)
    elif math.isinf(expected_dcpa):
        assert math.isinf(dcpa)
    else:
        assert float(dcpa) == approx(float(expected_dcpa), rel=1e-9)

    if math.isnan(expected_tcpa):
        assert math.isnan(tcpa)
    elif math.isinf(expected_tcpa):
        assert math.isnan(dcpa)
    else:    
        assert float(tcpa) == approx(float(expected_tcpa), rel=1e-9)


def main():
    test_calc_dcpa_and_tcpa()
    
if __name__ == '__main__':
    main()