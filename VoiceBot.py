"""Voice2Text"""
import openai
import re
import argparse
from robotiq_wrapper import *
import os
import json
import speech_recognition as sr


# Initialize the recognizer
r = sr.Recognizer()

parser = argparse.ArgumentParser()
parser.add_argument("--prompt", type=str, default="prompts/aspida_basic.txt")
parser.add_argument("--sysprompt", type=str, default="system_prompts/aspida_basic.txt")
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
        "content": "go to the Kitchen"
    },
    {
        "role": "assistant",
        "content": """
        ```python
        x, y = aw.get_position("Kitchen")
        aw.navigation_to_target(x, y)
        aw.focus_on_kitchen()
        ```
        This code uses the `navigation_to_target()` function to move the robot to the Kitchen. 
        It does this by getting the x, y coordinates of the target named "Kitchen" and pass them as arguments in the `navigation_to_target()` function. 
        The robot will then navigate to the specified location and use corresponding focus_on_kitchen function to focus on the kitchen.
        """
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

        return full_code
    else:
        return None


class colors:  # You may need to change color settings
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


print(colors.YELLOW + "Initializing ASPiDA...")
aw = ASPiDAWrapper()
print(colors.YELLOW + "Done.")

with open(args.prompt, "r") as f:
    prompt = f.read()

ask(prompt)
print(colors.YELLOW + "Welcome to the ASPiDA VoiceBot! I am ready to help you with your ASPiDA questions and commands.")


# Function to activate the microphone and recognize speech
def get_command_from_microphone():
    print("Press Enter to start recording...")
    input()  # Wait for the user to press Enter to start recording
    with sr.Microphone() as source:
        print("Recording...")
        audio_data = r.record(source, duration=10)

    try:
        print("Recognizing...")
        text = r.recognize_google(audio_data, language="es-ES")  # language="el-GR")
        return text
    except sr.UnknownValueError:
        #print("Sorry, I could not understand your command.")
        return ""
    except sr.RequestError as e:
        #print(f"Could not request results from Google Speech Recognition service; {e}")
        return ""


while True:

    text = get_command_from_microphone()

    if text == "":
        continue

    question = text #input(colors.YELLOW + "ASPiDA> " + colors.ENDC + text)

    #subprocess.call(['xdotool', 'windowactivate', '--sync', window_id, 'type', "ASPiDA> {}\n".format(text)])

    if question == "!quit" or question == "!exit":
        break

    if question == "!clear":
        os.system("cls")
        continue


    response = ask(question)

    print(f"\n{response}\n")

    code = extract_python_code(response)
    if code is not None:
        print("Please wait while I run the code for ASPiDA...")
        exec(extract_python_code(response))
        print("Done!\n")

