class Configuration:
    """Represents the dynamic state of an object at a given time instance."""

    class Pose:
        """Represents a 3D pose, consisting of position (x, y, z) and orientation (quaternion)."""

        class Position:
            """3D position represented in meters."""

            def __init__(self, x: float, y: float, z: float):
                self.x = x
                self.y = y
                self.z = z

            def __eq__(self, other):
                if isinstance(other, Configuration.Pose.Position):
                    return self.x == other.x and self.y == other.y and self.z == other.z
                return False

            def __hash__(self):
                return hash((self.x, self.y, self.z))

        class Orientation:
            """Quaternion-based orientation."""

            def __init__(self, x: float, y: float, z: float, w: float):
                self.x = x
                self.y = y
                self.z = z
                self.w = w

            def __eq__(self, other):
                if isinstance(other, Configuration.Pose.Orientation):
                    return (
                        self.x == other.x
                        and self.y == other.y
                        and self.z == other.z
                        and self.w == other.w
                    )
                return False

            def __hash__(self):
                return hash((self.x, self.y, self.z, self.w))

        def __init__(
            self,
            position: "Configuration.Pose.Position",
            orientation: "Configuration.Pose.Orientation",
        ):
            """Initializes the Pose with a given position and orientation."""
            self.position = position
            self.orientation = orientation

        def __eq__(self, other):
            if isinstance(other, Configuration.Pose):
                return (
                    self.position == other.position
                    and self.orientation == other.orientation
                )
            return False

        def __hash__(self):
            return hash((self.position, self.orientation))

    def __init__(self, pose: Pose, yaw_rate: float, velocity: float):
        """
        Initializes the configuration of a dynamic object.

        :param pose: The object's pose, consisting of position and orientation.
        :param yaw_rate: The rate of change of the object's heading (rad/s).
        :param velocity: The object's speed in meters per second.
        """
        self.pose = pose
        self.yaw_rate = yaw_rate
        self.velocity = velocity

    def __eq__(self, other):
        if isinstance(other, Configuration):
            return (
                self.pose == other.pose
                and self.yaw_rate == other.yaw_rate
                and self.velocity == other.velocity
            )
        return False

    def __hash__(self):
        return hash((self.pose, self.yaw_rate, self.velocity))