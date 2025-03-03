import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from colav_unsafe_set.objects import (
    DynamicObject,
    DynamicObstacle,
    Configuration,
)
from colav_unsafe_set import create_unsafe_set

agent_vessel = DynamicObject(
    configuration=Configuration(
        pose=Configuration.Pose(
            position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
            orientation=Configuration.Pose.Orientation(x=0.0, y=0.0, z=0.0, w=0.0),
        ),
        yaw_rate=0.0,
        velocity=0.0,
    ),
    safety_radius=5.0,
)

dynamic_obstacles = list(
    [
        DynamicObstacle(
            id="1",
            object=DynamicObject(
                configuration=Configuration(
                    pose=Configuration.Pose(
                        position=Configuration.Pose.Position(x=0.0, y=0.0, z=0.0),
                        orientation=Configuration.Pose.Orientation(
                            x=0.0, y=0.0, z=0.0, w=0.0
                        ),
                    ),
                    yaw_rate=0.0,
                    velocity=0.0,
                ),
                safety_radius=5.0,
            ),
        ),
        DynamicObstacle(
            id="2",
            object=DynamicObject(
                configuration=Configuration(
                    pose=Configuration.Pose(
                        position=Configuration.Pose.Position(x=10.0, y=0.0, z=0.0),
                        orientation=Configuration.Pose.Orientation(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        ),
                    ),
                    yaw_rate=0.1,
                    velocity=10.0,
                ),
                safety_radius=3.0,
            ),
        ),
    ]
)

dsf = 1.0


def main():
    polyshape = create_unsafe_set(
        agent_vessel=agent_vessel, dynamic_obstacles=dynamic_obstacles, dsf=dsf
    )
    print(polyshape)


if __name__ == "__main__":
    main()
