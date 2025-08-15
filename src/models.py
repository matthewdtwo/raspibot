from typing import List, Literal, NamedTuple, Optional
from pydantic import BaseModel, Field
from dataclasses import dataclass, field


class Movement(BaseModel):
    type: Literal["rotation", "linear"]
    unit: Literal["mm", "deg"]
    amount: int = Field(description="positive for forward / clockwise. negative for backward / counter-clockwise. Rotation limited to +/-180 degrees.")

class Response(BaseModel):
    movement: Movement
    observation: str = Field(description="Observe the environment and explain your thought process for your next moves")

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


@dataclass
class Observation:
    snapshot_path: str
    observation: str

@dataclass
class RobotState:
    observations: List[Observation] = field(default_factory=list)
    previous_movements: List[MovementResult] = field(default_factory=list)
    current_report: str = ""


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