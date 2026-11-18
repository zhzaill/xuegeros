"""Standardized XuegerOS agent control primitives.

The package deliberately keeps the safety and planning core independent of ROS.
The ROS node is an adapter around these primitives, so the same policy can be
used by a conversation plugin, a simulator, and the vehicle runtime.
"""

from .policy import SafetyDecision, SafetyPolicy
from .protocol import ActionProposal, WorldSnapshot

__all__ = [
    "ActionProposal",
    "SafetyDecision",
    "SafetyPolicy",
    "WorldSnapshot",
]
