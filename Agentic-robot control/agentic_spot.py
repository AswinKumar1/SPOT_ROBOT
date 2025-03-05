import os
import io
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
from langchain import hub
from langchain.agents import AgentExecutor, create_tool_calling_agent, tool
from langchain_core.tools import Tool
from langchain_openai import ChatOpenAI
from dotenv import load_dotenv
from spot_wrapper import SpotWrapper  # Import SpotWrapper
from langchain_core.prompts import ChatPromptTemplate

# Load environment variables
load_dotenv()
os.environ["PLAYDIALOG_API_KEY"] = os.getenv("PLAYDIALOG_API_KEY")
os.environ["PLAYDIALOG_USER_ID"] = os.getenv("PLAYDIALOG_USER_ID")
os.environ["SERPAPI_API_KEY"] = os.getenv("SERPAPI_API_KEY")  

q = queue.Queue()

# Initialize SpotWrapper
spot = SpotWrapper(spot_init=True)  # Initialize the robot

# Define your tools using the @tool decorator

@tool
def move_robot_forward(v_x=1.0, duration_s=5.0):
    """Makes the robot move forward"""
    try:
        # stream_audio("Moving the robot forward")
        v_x_value = float(1.0)
        duration_s_value = float(2.0)
        spot.move_spot(v_x_value, duration_s_value)
        return f"Robot moved forward with v_x={v_x_value} for {duration_s_value} seconds."
    except ValueError as e:
        return f"Error: {e}"

@tool
def move_robot_backward(v_x=1.0, duration_s=5.0):
    """Makes the robot move backward"""
    try:
        # Extract numeric values from the string arguments and convert them to floats
        v_x_value = -float(1.0)
        duration_s_value = float(2.0)
        # stream_audio("Moving the robot backward")
        spot.move_spot(v_x_value, duration_s_value)
        return f"Robot moved backward with v_x={v_x_value} for {duration_s_value} seconds."
    except ValueError as e:
        # stream_audio("Error: Invalid input values for v_x or duration_s")
        return f"Error: {e}"

@tool
def stop_robot():
    """Stops the robot from moving"""
    # stream_audio("Stopping the robot")
    spot.stop_spot()
    return "Robot stopped."

@tool
def stand_robot():
    """Makes the robot stand up"""
    # stream_audio("Making the robot stand now")
    spot.stand_spot()
    return "Robot is standing."

@tool
def sit_robot():
    """Makes the robot sit down"""
    # stream_audio("Making the robot sit down")
    spot.sit_spot()
    return "Robot is sitting."

# List of tools available to the agent
tools = [
    move_robot_forward,
    move_robot_backward,
    stop_robot,
    stand_robot,
    sit_robot
]

# Create a new prompt template
prompt = ChatPromptTemplate.from_messages(
    [
        ("system", "You are a helpful assistant who can execute multiple tasks sequentially in a robot."),
        ("human", "{input}"),
        ("placeholder", "{agent_scratchpad}"),
    ]
)

# Create the agent using the tool_calling_agent
llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)
agent = create_tool_calling_agent(llm, tools, prompt)

# Set up the agent executor
agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)

# Function to stream audio
def stream_audio(text):
    client = OpenAI()
    speech_file_path = "speech.mp3"
    response = client.audio.speech.create(
        model="tts-1",  
        voice="alloy",  
        input=text,
    )

    response.stream_to_file(speech_file_path)
    play_audio(speech_file_path)

# Function to play audio
def play_audio(file_path):
    from pydub import AudioSegment
    import simpleaudio as sa

    audio = AudioSegment.from_mp3(file_path)
    play_obj = sa.play_buffer(audio.raw_data, num_channels=audio.channels, bytes_per_sample=audio.sample_width, sample_rate=audio.frame_rate)
    play_obj.wait_done()  
    os.remove(file_path)

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

def main():
    model = Model(lang="en-us")
    with sd.RawInputStream(samplerate=16000, blocksize=2000, dtype="int16", channels=1, callback=callback):
        print("#" * 80)
        print("Listening for commands...")
        print("#" * 80)

        rec = KaldiRecognizer(model, 16000)
        while True:
            try:
                data = q.get(timeout=10)
            except queue.Empty:
                print("No audio data received. Waiting...")
                continue

            if rec.AcceptWaveform(data):
                result = rec.Result()

                try:
                    recognized_text = json.loads(result)["text"].strip().lower()
                except (json.JSONDecodeError, KeyError):
                    recognized_text = ""

                if recognized_text:
                    print(f"Recognized command: {recognized_text}")
                    print("Processing input command query...")
                    response = agent_executor.invoke({"input": recognized_text})
                    # stream_audio(response['output'])  # Uncomment this if you want to stream the response as audio
                else:
                    print("No valid command. Waiting for a new command...")

if __name__ == "__main__":
    main()
