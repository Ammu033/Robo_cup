import random
import ollama
import os

the_path = os.path.dirname(__file__)
if the_path == "":
    the_path = "."

test_img = random.choice([os.path.join(the_path, i) for i in os.listdir(the_path) if os.path.splitext(i)[-1] in {".jpg"}])
print("Using the test image '%s'" % test_img)

prompt = "Is there any trash on the floor in this image? Add the token >>RUBBISH<< to your response if there is trash in the image."

client = ollama.Client(host = "http://192.168.69.253:11434")
ollama_output = client.generate(
    model = "llava:7b", 
    prompt = prompt,
    images = [test_img], 
    keep_alive = "0m"
)["response"]

print(ollama_output)