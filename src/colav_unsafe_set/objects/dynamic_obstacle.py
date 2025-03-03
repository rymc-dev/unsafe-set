from .dynamic_object import DynamicObject

class DynamicObstacle:
    def __init__(self, id: str, object: DynamicObject):
        self.id = id
        self.object = object

    def __eq__(self, other):
        if isinstance(other, DynamicObstacle):
            return self.id == other.id and self.object == other.object
        return False

    def __hash__(self):
        return hash((self.id, self.object))
