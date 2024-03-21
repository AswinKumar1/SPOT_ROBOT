import openai
import re
import argparse
from spot_wrapper import *
import math
import numpy as np
import os
import json
import time

parser = argparse.ArgumentParser()
parser.add_argument("--prompt", type=str, default="~/chatgpt_spotrobot-master/gpt_spot/prompts/airsim_basic.txt")
parser.add_argument("--sysprompt", type=str, default="~/chatgpt_spotrobot-master/gpt_spot/system_prompts/airsim_basic.txt")
args = parser.parse_args()

with open("config.json", "r") as f:
    config = json.load(f)

print("Initializing ChatGPT...")
openai.api_key = config["OPENAI_API_KEY"]

with open(args.sysprompt, "r") as f:
    sysprompt = f.read()

chat_history = [
    {
        "role": "system",
        "content": sysprompt
    },
    {
        "role": "user",
        "content": "move spot robot"
    },
    {
        "role": "assistant",
        "content": """```python
aw.move_spot()
```

This code uses the `move_spot()` function to move the spot robot ahead. just call the funciton. you can `."""
    }
]


def ask(prompt):
    chat_history.append(
        {
            "role": "user",
            "content": prompt,
        }
    )
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=chat_history,
        temperature=0
    )
    chat_history.append(
        {
            "role": "assistant",
            "content": completion.choices[0].message.content,
        }
    )
    return chat_history[-1]["content"]


print(f"Done.")

code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)


def extract_python_code(content):
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)

        if full_code.startswith("python"):
            full_code = full_code[7:]
        print("function called: ", full_code)
        return full_code
    else:
        return None


class colors:  # You may need to change color settings
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


#standprint(f"Initializing AirSim...")
aw = SpotWrapper()
print(f"Done.")

with open(args.prompt, "r") as f:
    prompt = f.read()

ask(prompt)
print("Welcome to the SpotRobot chatbot! I am ready to help you with your SpotRobot questions and commands.")

executed_commands = {}

while True:
    question = input(colors.YELLOW + "AirSim> " + colors.ENDC)

    if question == "!quit" or question == "!exit":
        break

    if question == "!clear":
        os.system("cls")
        continue

    commands = question.split(",")
    
    for ques in commands:
        response = ask(ques.strip())
        print(f"\n{response}\n")
        code = extract_python_code(response)
        print(code)
        if code is not None:
            if code not in executed_commands:  # Check if the command is already executed
                print("Please wait while I command Spot Robot...")
                exec(code)
                executed_commands[code] = True  # Add the executed command to the dictionary
                print("Done!\n")
        executed_commands = {}
		
		
		
		
		
