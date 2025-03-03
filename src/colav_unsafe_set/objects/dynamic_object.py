from .configuration import Configuration

class DynamicObject:
    """
    Represents a dynamic object with a configuration and a safety radius.

    :param configuration: The object's dynamic state (position, orientation, velocity, etc.).
    :param safety_radius: The object's safety buffer radius in meters.
    """

    def __init__(self, configuration: Configuration, safety_radius: float):
        self.configuration = configuration
        self.safety_radius = safety_radius

    def __eq__(self, other):
        if isinstance(other, DynamicObject):
            return (
                self.configuration == other.configuration
                and self.safety_radius == other.safety_radius
            )
        return False

    def __hash__(self):
        return hash((self.configuration, self.safety_radius))





