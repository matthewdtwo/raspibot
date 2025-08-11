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
            {"messages": [{"role": "user", "content": "Thoroughly explore and map the entire room. Take snapshots from multiple angles, rotate to see all sides, move around to different positions, and compile a detailed report of everything you observe. Continue exploring until you have a complete understanding of the space."}] },
            {"recursion_limit":  100}
        )

        print(response)