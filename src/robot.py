from typing import List, Literal
from pydantic import BaseModel, Field

from camera import Camera
from llm import LLMs, Movement
from models import MovementResult
from move_controller import MoveController
from web_interface import WebInterface

class RobotState:
    snapshot_descriptions: List[str] = []
    previous_movements: List[Movement] = []
    current_report: str = ""

class Robot:
    def __init__(self, move_controller: MoveController, llm: LLMs, camera: Camera, web_interface: WebInterface):
    
        self.move_controller = move_controller
        self.llm = llm
        self.camera = camera
        self.web_interface = web_interface
        self.state = RobotState()

        print("Robot initialized")
        print()

    def _take_action(self, action: Movement) -> MovementResult:
        if action.type == "rotation":
            if action.amount > 0 and action.amount <= 180:
                return self.move_controller.rotate_cw(action.amount)
            elif action.amount < 0 and action.amount >= -180:
                return self.move_controller.rotate_ccw(action.amount)
            else:
                print("Invalid rotation amount")
        if action.type == "linear":
            pass
            # if action.amount > 0:
            #     return self.move_controller.move_forward(action.amount)
            # elif action.amount < 0:
            #     return self.move_controller.move_backward(action.amount)
            # else:
            #     print("Invalid linear movement amount")

        raise Exception("Invalid movement action")

    def explore(self) -> RobotState:
        print("Beginning exploration")

        initial_snapshot_path = self.camera.take_snapshot()

        description = self.llm.describe_image(initial_snapshot_path, self.state.snapshot_descriptions)
        self.state.snapshot_descriptions.append(description)
    
        if len(self.state.snapshot_descriptions) > 5:
            # summarize if we have more than 5 descriptions to keep context smaller
            summary = self.llm.summarize(self.state.snapshot_descriptions)
            self.state.snapshot_descriptions = [summary]



        print("Deciding action")

        action = self.llm.decide_next_action(description)
        self.state.previous_movements.append(action)

        self._take_action(action)

        return self.state