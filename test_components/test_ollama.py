from langchain_ollama import ChatOllama
from picamera2 import Picamera2
from libcamera import Transform
import io
import time
from base64 import b64encode

model_name = "llama3.2-vision:latest"

llm = ChatOllama(
    model=model_name,
    temperature=0.7,
    base_url="http://vengeance.matthewtwo.com:11434"
)

print(llm.invoke("Hello, world"))


picam2 = Picamera2()
config = picam2.create_still_configuration(transform=Transform(vflip=1, hflip=1))
picam2.configure(config)

picam2.start()
time.sleep(1)

data = io.BytesIO()
picam2.capture_file(data, format='jpeg')
data.seek(0)

b64_image = b64encode(data.getvalue()).decode('utf-8')

_vision_system_prompt = """You're part of a robotic exploration system. Your job is to describe the image to the best of your abilities so that a control agent system can determine the next steps for the robot."""


from langchain_core.messages import HumanMessage

message = HumanMessage(
    content=[
        {
            "type": "text",
            "text": _vision_system_prompt + " Please describe what you see in this image."
        },
        {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{b64_image}"
            }
        }
    ]
)

response = llm.invoke([message])

print(response.content)