# package imports
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, AIMessage, SystemMessage, BaseMessage
from base64 import b64encode
from typing import List, Union
from pydantic import BaseModel

# local imports
from config import OLLAMA_HOST, TEMPERATURE
from models import Response, RobotState

class LLMs:
    _system_prompt = """You're controlling a small, wheeled robotic exploration system. Your objective is exploring the environment. It is equipped with a motion control system for movement and a vision system for capturing images. If you get stuck, try backing up by using a negative movement command."""

    _initial_prompt = """Thoroughly describe what you see, and plan a route to explore the current environment and understand where you are. Describe the potential obstacles and how you might navigate around them. Identify their rough position relative to your view."""

    def __init__(self, model="gemma3:27b-it-qat"):
        self._llm = ChatOllama(model=model, base_url=OLLAMA_HOST, temperature=TEMPERATURE)
        print("LLM initialized")

    def _send_message(self, messages: List[BaseMessage], output_type=None) -> Union[BaseModel, str]:
        if output_type:
            response = self._llm.with_structured_output(output_type).invoke(messages)
        else:
            response = self._llm.invoke(messages).content
        return response  # type: ignore

    def _prepare_image_from_path(self, img_path: str) -> dict:
        with open(img_path, "rb") as image_file:
            encoded_string = b64encode(image_file.read()).decode("utf-8")
            image_part = {"type": "image_url", "image_url": f"data:image/jpeg;base64,{encoded_string}"}
            return image_part

    def _history_from_state(self, state: RobotState) -> List[BaseMessage]:
        messages: List[BaseMessage] = [SystemMessage(content=self._system_prompt)]

        if not state.observations:
            return messages

        messages.append(HumanMessage(content=[self._prepare_image_from_path(state.observations[0].snapshot_path), {"type": "text", "text": self._initial_prompt}]))

        obs_count = len(state.observations)
        move_count = len(state.previous_movements)
        print(f"observations: {obs_count}\nPrevious movements: {move_count}")

        for i in range(move_count):
            if i >= obs_count:
                break
            movement = state.previous_movements[i]
            obs_for_action = state.observations[i]
            messages.append(AIMessage(content=f"Observation: {obs_for_action.observation} - Action: Requested movement: {movement.type} - {movement.target} {movement.unit}"))
            post_obs_index = i + 1
            if post_obs_index < obs_count:
                post_obs = state.observations[post_obs_index]
                messages.append(HumanMessage(content=[self._prepare_image_from_path(post_obs.snapshot_path), {"type": "text", "text": f"Actual movement: Encoders: {movement.actual.measured} {movement.unit}, Optical: {movement.actual.optical} {movement.unit}. Warning: {movement.actual.warning}. What changed between the two pictures and what should you do next?"}]))

        for message in messages:
            if not isinstance(message, HumanMessage):
                print(f"{type(message)} - {message.content}")
            else:
                if len(message.content) == 2:
                    print(f"{type(message)} {message.content[1]}")

        if len(messages) > 6:
            system = messages[0]
            tail = messages[1:][-5:]
            messages = [system] + tail

        return messages

    def process(self, state: RobotState) -> Response:
        messages = self._history_from_state(state)
        response = self._send_message(messages=messages, output_type=Response)
        print("#############################")
        print(response)
        print("#############################")
        print()
        return response  # type: ignore