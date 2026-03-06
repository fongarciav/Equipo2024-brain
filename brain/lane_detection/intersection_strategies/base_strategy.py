from abc import ABC, abstractmethod


class IntersectionStrategy(ABC):
    """Base strategy for lane-based intersection events."""

    @abstractmethod
    def execute(self, intersection_state: dict):
        """Execute an action when a debounced intersection event is triggered."""
        pass
