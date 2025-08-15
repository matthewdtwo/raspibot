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
    _system_prompt = """You're controlling a small, wheeled robotic exploration system. Your objective is exploring the environment. It is equipped with a motion control system for movement and a vision system for capturing images."""

    _initial_prompt = """Thoroughly describe what you see, and plan a route to explore the current environment and understand where you are. Describe the potential obstacles and how you might navigate around them. Identify their rough position relative to your view."""

    def __init__(self, model="gemma3:27b-it-qat"):
        self._llm = ChatOllama(model=model, base_url=OLLAMA_HOST, temperature=TEMPERATURE)
        print("LLM initialized")

    def _send_message(self, messages: List[BaseMessage], output_type=None) -> Union[BaseModel, str]:
        if output_type:
            response = self._llm.with_structured_output(output_type).invoke(messages)
        else:
            response = self._llm.invoke(messages).content
        return response # type: ignore
    

    def _prepare_image_from_path(self, img_path: str) -> dict:
        with open(img_path, "rb") as image_file:
            encoded_string = b64encode(image_file.read()).decode("utf-8")

            image_part = {
                "type": "image_url",
                "image_url": f"data:image/jpeg;base64,{encoded_string}",
            }

            return image_part

    def _history_from_state(self, state: RobotState) -> List[BaseMessage]:

        messages: List[BaseMessage] = [SystemMessage(content=self._system_prompt)]

        # SystemMessage
        # HumanMessage(picture, initial_prompt)
        # AIMessage(observation, action)
        # HumanMessage(picture, action_results)

        if len(state.observations) == 1:
            # no initial observation yet
            messages.append(HumanMessage(content=[self._prepare_image_from_path(state.observations[0].snapshot_path), {"type": "text", "text": self._initial_prompt}]))
        else:
            print(f"observations: {len(state.observations)}\nPrevious movements: {len(state.previous_movements)}")

            for observation, movement in zip(state.observations, state.previous_movements):
                movement_request = f"Requested movement: {movement.type} - {movement.target} {movement.unit}"
                # observation and movement request.target and request.type and request.unit go into AI message.
                messages.append(AIMessage(content=f"Observation: {observation.observation} - Action: {movement_request}"))
                messages.append(HumanMessage(content=[self._prepare_image_from_path(observation.snapshot_path), {"type": "text", "text": f"Actual movement: Encoders: {movement.actual.measured} {movement.unit}, Optical: {movement.actual.optical} {movement.unit}. Warning: {movement.actual.warning}. What changed between the two pictures and what should you do next?"}]))



        for message in messages:
            if not isinstance(message, HumanMessage):
                print(f"{type(message)} - {message.content}")
            else:
                if(len(message.content)) == 2:
                    print(f"{type(message)} {{image}} - {message.content[1]}")
    

        if len(messages) > 5:
            messages = messages[-5:]

        return messages

    def process(self, state: RobotState) -> Response:
        messages = self._history_from_state(state)
        
        response = self._send_message(messages=messages, output_type=Response)

        print("#############################")
        print(response)
        print("#############################")


        print()

        return response # type: ignore


        


    # def describe_image(self, image_path, previous_descriptions = []) -> str:
    #     # get image from path and base64 encode it.
    #     with open(image_path, "rb") as image_file:
    #         encoded_string = b64encode(image_file.read()).decode("utf-8")

    #         image_part = {
    #             "type": "image_url",
    #             "image_url": f"data:image/jpeg;base64,{encoded_string}",
    #         }

    #         text_prompt = ""

    #         if len(previous_descriptions) > 0:
    #             text_prompt += "Here are your previous observations: Use them to maintain consistent terminology when describing the scene. Only describe the changes in the new image. "
    #             text_prompt += " ".join(previous_descriptions)

    #         text_prompt += self._description_prompt

    #         text_part = {
    #             "type": "text",
    #             "text": text_prompt
    #         }                

    #         messages = [
    #             SystemMessage(content=self._system_prompt),
    #             HumanMessage(content=[image_part, text_part])
    #         ]

    #         return self._send_message(messages) # type: ignore
        
    # def decide_next_action(self, observation: str) -> Movement:

    #     if observation not in [None, ""]:
    #         content = f"""Environment observation: {observation} \n\n{self._action_prompt}"""
    #     else:
    #         content = self._action_prompt

    #     messages = [
    #         SystemMessage(content=self._system_prompt),
    #         HumanMessage(content=content)
    #     ]

    #     return self._send_message(messages, Movement) # type: ignore

    # def summarize(self, descriptions: List[str]) -> str:
    #     content = f"""Previous observations: {descriptions} \n\nPlease summarize the key points."""

    #     messages = [
    #         SystemMessage(content=self._system_prompt),
    #         HumanMessage(content=content)
    #     ]

    #     return self._llm.invoke(messages).content # type: ignore