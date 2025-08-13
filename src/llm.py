# package imports
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, SystemMessage, BaseMessage
from base64 import b64encode
from typing import List

# local imports
from config import OLLAMA_HOST
from models import Movement


class LLMs:
    _system_prompt = """You're a robotic exploration system tasked with exploring the environment."""
    _description_prompt = """Describe the image."""

    def __init__(self, model="gemma3:12b"):
        self.llm = ChatOllama(model=model, base_url=OLLAMA_HOST)
        print("LLM initialized")
    

    def describe_image(self, image_path, previous_descriptions = None) -> str:
        # get image from path and base64 encode it.
        with open(image_path, "rb") as image_file:
            encoded_string = b64encode(image_file.read()).decode("utf-8")

            image_part = {
                "type": "image_url",
                "image_url": f"data:image/jpeg;base64,{encoded_string}",
            }

            text_prompt = ""

            if previous_descriptions:
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

            return self.llm.invoke(messages).content # type: ignore

    def decide_next_action(self, observation: str) -> Movement:
        
        content = f"""Environment observation: {observation} \n\nWhat action do you wish to perform?"""

        messages = [
            SystemMessage(content=self._system_prompt),
            HumanMessage(content=content)
        ]

        return self.llm.with_structured_output(Movement).invoke(messages) # type: ignore
    

    def summarize(self, descriptions: List[str]) -> str:
        content = f"""Previous observations: {descriptions} \n\nPlease summarize the key points."""

        messages = [
            SystemMessage(content=self._system_prompt),
            HumanMessage(content=content)
        ]

        return self.llm.invoke(messages).content # type: ignore