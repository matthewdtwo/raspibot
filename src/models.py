from typing import Literal, NamedTuple, Optional
from pydantic import BaseModel, Field
from dataclasses import dataclass


class Movement(BaseModel):
    type: Literal["rotation", "linear"]
    unit: Literal["mm", "deg"]
    amount: int = Field(description="positive for forward / clockwise. negative for backward / anti-clockwise rotation limited to 180 degrees")

class ActualMovement(NamedTuple):
    """Represents the actual movement data"""
    measured: int
    optical: int
    warning: Optional[str] = None

@dataclass
class MovementResult:
    type: Literal["rotation", "linear"]
    target: int
    actual: ActualMovement
    unit: Literal["mm", "deg"]


class MovementParams(NamedTuple):
    """Parameters for movement calculations"""
    wheel_circumference_mm: float
    pulses_per_mm_L: float
    pulses_per_mm_R: float
    target_pulses_L: int
    target_pulses_R: int


class PIDState(NamedTuple):
    """State for PID controller"""
    Kp: float = 0.25
    Ki: float = 0.0
    Kd: float = 0.02
    K_steer: float = 0.035
    dt: float = 0.05
    tolerance_mm: float = 3.0
    integral_limit_factor: int = 4    