from robot import Robot
from camera import Camera
from llm import LLMs


if __name__ == "__main__":
    print("🤖 Robot Exploration System Starting...")
    print("=" * 50)
    
    with Robot() as robot:
        print("✅ Robot initialized successfully")
        
        camera = Camera()
        print("📷 Camera initialized successfully")
        
        robot_tools = [robot.move_backward, robot.move_forward, robot.rotate_cw, robot.rotate_ccw]

        llm = LLMs(tools=robot_tools, camera=camera, debug=True)

        response = llm.agent.invoke(
            {"messages": [{"role": "user", "content": "Explore the environment."}] },
            {"recursion_limit":  15}
        )

        print(response)