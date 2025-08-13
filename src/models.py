from typing import Literal
from pydantic import BaseModel, Field
from dataclasses import dataclass


class Movement(BaseModel):
    type: Literal["rotation", "linear"]
    unit: Literal["mm", "deg"] = Field(description="positive for forward / clockwise. negative for backward / anti-clockwise")
    amount: int

@dataclass
class MovementResult:
    type: Literal["rotation", "linear"]
    target: int
    actual: int
    unit: Literal["mm", "deg"]