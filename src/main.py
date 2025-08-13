from robot import Robot
from camera import Camera
from llm import LLMs
from web_interface import WebInterface
import time


if __name__ == "__main__":
    print("🤖 Robot Exploration System Starting...")
    print("=" * 50)
    
    # Start web interface
    web_interface = WebInterface(port=5000)
    web_thread = web_interface.run_threaded(debug=False)
    
    print("🌐 Web interface started on http://localhost:5000")
    print("📱 Open this URL in your browser to monitor the robot")
    
    # Give the web interface time to start
    time.sleep(2)
    
    # Initialize robot with web interface
    with Robot(web_interface=web_interface) as robot:
        web_interface.update_status(active=True, current_action="Initializing")
        web_interface.log_message('system', 'Robot initialized successfully')
        print("✅ Robot initialized successfully")
        
        camera = Camera(persistent_snapshots=True)
        web_interface.log_message('system', 'Camera initialized successfully')
        print("📷 Camera initialized successfully")
        
        robot_tools = [robot.move_backward, robot.move_forward, robot.rotate_cw, robot.rotate_ccw]

        llm = LLMs(tools=robot_tools, camera=camera, debug=True, web_interface=web_interface)
        web_interface.log_message('system', 'LLM agent initialized with tools')
        web_interface.update_status(current_action="Ready for exploration")

        # Log the user request
        user_message = "Thoroughly explore and map the entire room. Take snapshots from multiple angles, rotate to see all sides, move around to different positions, and compile a detailed report of everything you observe. Continue exploring until you have a complete understanding of the space."
        web_interface.log_message('user', user_message)
        
        web_interface.update_status(current_action="Starting exploration mission")
        
        response = llm.invoke_with_streaming(
            {"messages": [{"role": "user", "content": user_message}] },
            {"recursion_limit":  100}
        )

        web_interface.log_message('agent', str(response))
        web_interface.update_status(current_action="Exploration complete")
        print("🎯 Exploration mission complete!")
        print("🌐 Web interface continues running. Press Ctrl+C to stop.")
        
        # Keep the program running so the web interface stays active
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down...")
            web_interface.update_status(active=False, current_action="Shutdown")
            web_interface.log_message('system', 'System shutdown initiated')