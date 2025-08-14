from camera import Camera
from llm import LLMs
from move_controller import MoveController
from robot import Robot
from web_interface import WebInterface
from utils import reset_snapshots
import time
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Robot exploration script')
    parser.add_argument('--reset', action='store_true', help='Remove all files in the snapshots directory on startup')
    parser.add_argument('--steps', type=int, default=20, help='Number of exploration steps')
    args = parser.parse_args()
    
    if args.reset:
        reset_snapshots()
    
    web_interface = WebInterface(port=5000)
    web_thread = web_interface.run_threaded(debug=False)
    
    time.sleep(2) # wait for web interface to start up.

    web_interface.update_status(active=False, current_action="Initializing...")

    camera = Camera(persistent_snapshots=True)
    web_interface.set_camera(camera)
    
    robot = Robot(move_controller=MoveController(), llm=LLMs(), camera=camera, web_interface=web_interface)

    report = robot.explore(steps=args.steps)

    print(f"Current Report: {report.current_report}")