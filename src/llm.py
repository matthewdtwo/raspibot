from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage
from langgraph.prebuilt import create_react_agent
from langchain_core.tools import tool

from config import OLLAMA_HOST

from base64 import b64encode
import os


class LLMs:
    _vision_system_prompt = """You're part of a robotic exploration system. Your job is to describe the image to the best of your abilities so that a control agent system can determine the next steps for the robot."""

    _agent_prompt = """You're a robot exploring an environment. Your job is to use the tools available to navigate and identify things in your environment and compile them into a report.

Important guidelines:
- Take multiple snapshots from different angles to build a complete picture
- Rotate to see all sides of the room/environment 
- Move forward and backward to explore different areas
- Continue exploring until you have a comprehensive understanding of the space
- Use tools frequently to gather information - don't stop after just a few observations
- Build a detailed mental map of the environment before concluding
- Always use proper tool calls to continue your exploration"""

    def __init__(self, vision_model="llama3.2-vision:latest", tool_model="qwen3:8b", tools = [], camera=None, debug=False):
        self._vision_model = ChatOllama(model=vision_model, base_url=OLLAMA_HOST)
        self._tool_model = ChatOllama(model=tool_model, base_url=OLLAMA_HOST)
        self._camera = camera
        self._debug = debug

        # Create a combined tool that takes a snapshot and analyzes it
        @tool
        def take_snapshot_and_analyze() -> str:
            """Take a snapshot of the environment and return an analysis of what's visible"""
            if self._camera is None:
                return "Error: No camera available"
            
            # Take snapshot
            image_path = self._camera.take_snapshot()
            
            # Analyze it
            description = self._get_image_description(image_path)
            
            return f"Snapshot taken and analyzed. Description: {description}"
        
        # Add the combined vision tool to the tools list
        tools.append(take_snapshot_and_analyze)

        self.agent = create_react_agent(
            tools=tools,
            model=self._tool_model,
            prompt=self._agent_prompt,
            debug=self._debug
        )

    def _get_image_description(self, image_path: str):
        """Uses a vision LLM to get a description of the image file"""
        # Read the image file and encode it to base64
        with open(image_path, 'rb') as image_file:
            image_data = image_file.read()
            b64_image = b64encode(image_data).decode('utf-8')

        message = HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": self._vision_system_prompt + " Please describe what you see in this image."
                },
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{b64_image}"
                    }
                }
            ]
        )

        response = self._vision_model.invoke([message])
        
        # Clean up the temporary file after processing
        try:
            os.unlink(image_path)
        except OSError:
            pass  # File might already be deleted
            
        return response.content



