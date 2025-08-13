# package imports
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, SystemMessage, BaseMessage
from base64 import b64encode
from typing import List, Union
from pydantic import BaseModel

# local imports
from config import OLLAMA_HOST, TEMPERATURE
from models import Movement

class LLMs:
    _system_prompt = """You're controlling a small, wheeled robotic exploration system. Your objective is exploring the environment. It is equipped with a motion control system for movement and a vision system for capturing images. No flattery."""
    _description_prompt = """Briefly describe the image. Identify the terrain, and any objects of interest."""
    _action_prompt = """What action should the robot take?"""

    def __init__(self, model="gemma3:12b"):
        self._llm = ChatOllama(model=model, base_url=OLLAMA_HOST, temperature=TEMPERATURE)
        print("LLM initialized")

    def _send_message(self, messages: List[BaseMessage], output_type=None) -> Union[BaseModel, str]:
        if output_type:
            response = self._llm.with_structured_output(output_type).invoke(messages)
        else:
            response = self._llm.invoke(messages).content
        return response # type: ignore
    

    def describe_image(self, image_path, previous_descriptions = []) -> str:
        # get image from path and base64 encode it.
        with open(image_path, "rb") as image_file:
            encoded_string = b64encode(image_file.read()).decode("utf-8")

            image_part = {
                "type": "image_url",
                "image_url": f"data:image/jpeg;base64,{encoded_string}",
            }

            text_prompt = ""

            if len(previous_descriptions) > 0:
                text_prompt += "Here are your previous observations: Use them to maintain consistent terminology when describing the scene. Only describe the changes in the new image. "
                text_prompt += " ".join(previous_descriptions)

            text_prompt += self._description_prompt

            text_part = {
                "type": "text",
                "text": text_prompt
            }                

            messages = [
                SystemMessage(content=self._system_prompt),
                HumanMessage(content=[image_part, text_part])
            ]

            return self._send_message(messages) # type: ignore
        
    def decide_next_action(self, observation: str) -> Movement:

        if observation not in [None, ""]:
            content = f"""Environment observation: {observation} \n\n{self._action_prompt}"""
        else:
            content = self._action_prompt

        messages = [
            SystemMessage(content=self._system_prompt),
            HumanMessage(content=content)
        ]

        return self._send_message(messages, Movement) # type: ignore

    def summarize(self, descriptions: List[str]) -> str:
        content = f"""Previous observations: {descriptions} \n\nPlease summarize the key points."""

        messages = [
            SystemMessage(content=self._system_prompt),
            HumanMessage(content=content)
        ]

        return self._llm.invoke(messages).content # type: ignore