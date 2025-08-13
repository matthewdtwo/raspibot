from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.prebuilt import create_react_agent
from langchain_core.tools import tool

from config import OLLAMA_HOST

from base64 import b64encode
import os


class LLMs:
    _vision_system_prompt = """You're a ground based robotic system tasked with exploring the environment. """

    _agent_prompt = """ You're a ground based robotic system tasked with exploring the environment. You have tools available to you, and can use them to move around and take snapshots. Navigate the environment to build a mental map, then report your findings once you feel you have navigated enough of the space."""

    def __init__(self, vision_model="llama3.2-vision:latest", tool_model="qwen3:8b", tools = [], camera=None, debug=False, web_interface=None):
        self._vision_model = ChatOllama(model=vision_model, base_url=OLLAMA_HOST)
        self._tool_model = ChatOllama(model=tool_model, base_url=OLLAMA_HOST)
        self._camera = camera
        self._debug = debug
        self._web_interface = web_interface
        
        # Track previous snapshot descriptions for context
        self._previous_descriptions = []
        self._max_description_history = 5  # Keep last 5 descriptions for context


        
        # Add the combined vision tool to the tools list

        self.agent = create_react_agent(
            tools=tools,
            model=self._tool_model,
            prompt=self._agent_prompt,
            debug=self._debug
        )

    def invoke_with_streaming(self, input_data, config=None):
        """Invoke the agent with streaming debug output to web interface"""
        if not self._web_interface:
            return self.agent.invoke(input_data, config)
        
        # Log the initial user message
        if 'messages' in input_data and input_data['messages']:
            user_msg = input_data['messages'][0]
            if hasattr(user_msg, 'content'):
                self._web_interface.log_message('user', user_msg.content)
        
        # Stream the agent execution
        self._web_interface.log_message('system', 'Agent execution started...')
        
        try:
            # Invoke the agent and capture the streaming output
            for chunk in self.agent.stream(input_data, config):
                # Log each step of the agent's reasoning
                if 'agent' in chunk:
                    agent_data = chunk['agent']
                    if 'messages' in agent_data:
                        for message in agent_data['messages']:
                            if hasattr(message, 'content') and message.content:
                                # Log agent thoughts/reasoning
                                self._web_interface.log_message('agent', message.content)
                            
                            # Log tool calls
                            if hasattr(message, 'tool_calls') and message.tool_calls:
                                for tool_call in message.tool_calls:
                                    tool_info = f"🔧 Calling tool: {tool_call['name']}"
                                    if 'args' in tool_call:
                                        tool_info += f" with args: {tool_call['args']}"
                                    self._web_interface.log_message('tool', tool_info)
                
                # Log tool execution results
                if 'tools' in chunk:
                    tools_data = chunk['tools']
                    if 'messages' in tools_data:
                        for message in tools_data['messages']:
                            if hasattr(message, 'content') and message.content:
                                tool_result = f"🔧 Tool result: {message.content}"
                                self._web_interface.log_message('tool', tool_result)
            
            # Get the final result
            final_result = self.agent.invoke(input_data, config)
            self._web_interface.log_message('system', 'Agent execution completed.')
            return final_result
            
        except Exception as e:
            error_msg = f"Agent execution error: {str(e)}"
            self._web_interface.log_message('system', error_msg)
            raise

    def _get_image_description(self, image_path: str):
        """Uses a vision LLM to get a description of the image file"""
        # Read the image file and encode it to base64
        with open(image_path, 'rb') as image_file:
            image_data = image_file.read()
            b64_image = b64encode(image_data).decode('utf-8')

        # Build context from previous descriptions
        context_text = "Please describe what you see in this image. "        
        if self._previous_descriptions:
            context_text += "\n\nFor context, here are your previous observations from earlier snapshots in this exploration:"
            for i, prev_desc in enumerate(self._previous_descriptions[-self._max_description_history:], 1):
                context_text += f"\n\nSnapshot {i}: {prev_desc}"
            context_text += "\n\nNow describe this current image, noting any changes, new objects, or different perspectives compared to your previous observations. Be consistent with your terminology and object identification."
        
        system_message = SystemMessage(
            content=self._vision_system_prompt
        )

        message = HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": context_text
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
        description = response.content
        
        # Add this description to the history
        self._previous_descriptions.append(description)
        
        # Keep only the most recent descriptions to avoid context getting too long
        if len(self._previous_descriptions) > self._max_description_history:
            self._previous_descriptions = self._previous_descriptions[-self._max_description_history:]
        
        # Only clean up temporary files (ones that start with temp path)
        if 'tmp' in image_path or 'temp' in image_path:
            try:
                os.unlink(image_path)
            except OSError:
                pass  # File might already be deleted
            
        return description