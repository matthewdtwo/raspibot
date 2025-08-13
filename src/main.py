from camera import Camera
from llm import LLMs
from move_controller import MoveController
from robot import Robot
from web_interface import WebInterface
import time


if __name__ == "__main__":
    web_interface = WebInterface(port=5000)
    web_thread = web_interface.run_threaded(debug=False)
    
    time.sleep(2)

    robot = Robot(move_controller=MoveController(), llm=LLMs(), camera=Camera(persistent_snapshots=True), web_interface=web_interface)

    report = robot.explore(steps=10)

    # pretty print the report

    for i, description in enumerate(report.snapshot_descriptions):
        print(f"Snapshot {i + 1}: {description}")

    print("Previous Movements:")
    for movement in report.previous_movements:
        print(f" - {movement}")

    print(f"Current Report: {report.current_report}")