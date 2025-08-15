from time import sleep


from camera import Camera
from llm import LLMs
from models import Movement, MovementResult, Observation, RobotState
from move_controller import MoveController
from web_interface import WebInterface



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
                print(f"Invalid rotation amount: {action}")
        if action.type == "linear":
            print(action)
            # raise Exception("unable to move forward")
            if action.amount > 0:
                return self.move_controller.move_forward(action.amount)
            elif action.amount < 0:
                return self.move_controller.move_backward(abs(action.amount))
            else:
                print("Invalid linear movement amount")

        raise Exception("Invalid movement action")

    # def _loop(self): 
    #     if self.web_interface:
    #         self.web_interface.update_status(active=True, current_action="Exploring")

    #     initial_snapshot_path = self.camera.take_snapshot()


    #     if self.web_interface:
    #         self.web_interface.update_status(active=True, current_action="Analyzing snapshot")

    #     description = self.llm.describe_image(initial_snapshot_path, self.state.
    #     snapshot_descriptions)

    #     if self.web_interface:
    #         self.web_interface.log_snapshot(initial_snapshot_path, description)

    #     self.state.snapshot_descriptions.append(description)
    
    #     if len(self.state.snapshot_descriptions) >= 5:
    #         if self.web_interface:
    #             self.web_interface.update_status(active=True, current_action="Summarizing snapshots")

    #         # summarize if we have more than 5 descriptions to keep context smaller
    #         summary = self.llm.summarize(self.state.snapshot_descriptions)
    #         self.state.snapshot_descriptions = [summary]

    #     print("Thinking")
    #     if self.web_interface:
    #         self.web_interface.update_status(active=True, current_action="Thinking")

    #     if len(self.state.previous_movements) > 0:
    #         description += "\n\nPrevious Movements: "
    #         description += "\n".join(f"- {mv}" for mv in self.state.previous_movements)

    #     action = self.llm.decide_next_action(description)
    #     self.state.previous_movements.append(action)

    #     if self.web_interface:
    #         self.web_interface.update_status(active=True, current_action="Taking action")

    #         self.web_interface.log_message("action", f"Decided to perform action: {action}")

    #     result = self._take_action(action)

    #     if self.web_interface:
    #         self.web_interface.log_message("action", f"Result: {result}")

    #     sleep(2) # sleep so the motion can finish before taking the next snapshot.


    def _loop2(self):
        self.web_interface.update_status(active=True, current_action="Processing observation")

        response = self.llm.process(self.state)

        self.state.observations[-1].observation = response.observation
        self.web_interface.update_status(active=True, current_action="Moving")
        self.web_interface.log_message("action", f"Movement Request: {response.movement}")

        self.web_interface.log_message("AI", f"{response.observation}")

        # take action from response and append to state
        movement_result = self._take_action(response.movement)

        self.web_interface.update_status(active=True, current_action="Move completed")
        self.web_interface.log_message("action", f"Movement Result: Encoders: {movement_result.actual.measured} {movement_result.unit} (optical: {movement_result.actual.optical}){f' Warning: {movement_result.actual.warning}' if movement_result.actual.warning else ''}")


        self.state.previous_movements.append(movement_result)


        # take next snapshot
        path = self.camera.take_snapshot()

        self.web_interface.log_snapshot(path, "New snapshot")

        self.state.observations.append(Observation(snapshot_path=path, observation=""))


        sleep(2)





    def explore(self, steps=5) -> RobotState:
        print("Beginning exploration")

        self.web_interface.update_status(active=True, current_action="Taking snapshot")

        # take first snapshot
        path = self.camera.take_snapshot()
        self.state.observations = [Observation(snapshot_path=path, observation="")]

        self.web_interface.log_snapshot(path, "Initial snapshot")

        for _ in range(steps):
            self._loop2()


        sleep(5) # stay up briefly
        self.web_interface.log_message("report", f"Final report: {self.state.current_report}")
        self.web_interface.update_status(active=False, status="Finished...")

        return self.state