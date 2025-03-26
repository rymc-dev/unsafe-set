import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from colav_unsafe_set import create_unsafe_set

def main():
    from colav_unsafe_set.objects import DynamicObstacle, Agent

    agent = Agent(
        position=(float(10), float(10), float(10)),
        orientation=(float(0), float(0), float(0), float(1)),
        velocity=float(15.0),
        yaw_rate=float(0.2),
        safety_radius=float(5)
    )
    dynamic_obstacles = [   
        DynamicObstacle(
            tag='obstacle 1',
            position=(float(30), float(20), float(0)),
            orientation=(float(0), float(0), float(0), float(1)),
            velocity=float(20.0),
            yaw_rate=float(0.1),
            safety_radius=float(10.0)
        ),
        DynamicObstacle(
            tag='obstacle 1',
            position=(float(5), float(7), float(0)),
            orientation=(float(0), float(0), float(0), float(1)),
            velocity=float(10.0),
            yaw_rate=float(0.1),
            safety_radius=float(7.0)
        )    
    ]

    vertices = create_unsafe_set(agent=agent, dynamic_obstacles=dynamic_obstacles, dsf=float(10))
    print(vertices)


if __name__ == "__main__":
    main()
