from langchain_ollama import ChatOllama

model_name = "llama3.2-vision:latest"

llm = ChatOllama(
    model=model_name,
    temperature=0.7,
    base_url="http://vengeance.matthewtwo.com:11434"
)

print(llm.invoke("Hello, world"))