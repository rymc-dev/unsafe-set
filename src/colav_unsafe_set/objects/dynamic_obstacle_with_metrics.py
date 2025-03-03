from .dynamic_obstacle import DynamicObstacle

class DynamicObstacleWithMetrics:
    def __init__(self, dynamic_obstacle: DynamicObstacle, dcpa: float, tcpa: float):
        self.dynamic_obstacle = dynamic_obstacle
        self.dcpa = dcpa
        self.tcpa = tcpa

    def __eq__(self, other):
        if isinstance(other, DynamicObstacleWithMetrics):
            return (
                self.dynamic_obstacle == other.dynamic_obstacle
                and self.dcpa == other.dcpa
                and self.tcpa == other.tcpa
            )
        return False

    def __hash__(self):
        return hash((self.dynamic_obstacle, self.dcpa, self.tcpa))