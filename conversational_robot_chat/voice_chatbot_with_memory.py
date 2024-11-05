import cv2
import numpy as np
import base64
import os
import queue
import sys
import sounddevice as sd
import soundfile as sf
from vosk import Model, KaldiRecognizer
import json
from datetime import datetime
import requests
import re
from pymongo import MongoClient
from langchain.memory import ConversationBufferWindowMemory
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate

# Global variables
q = queue.Queue()

# List of URLs and an index to track the current URL
urls = [url1, url2]
current_url_index = 0

def int_or_str(text):
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

def get_frame_from_video_stream(url):
    print(f"Attempting to open video stream {url}")
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"Error: Unable to open video stream {url}")
        return None
    ret, frame = cap.read()
    cap.release()
    if ret:
        print("Successfully captured frame from video stream")
        return frame
    else:
        print("Error: Unable to capture frame from video stream")
        return None

def save_image(image):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = f"captured_frame_{timestamp}.jpeg"
    cv2.imwrite(file_path, image)
    return file_path

def image_to_base64(image):
    _, buffer = cv2.imencode('.png', image)
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    return image_base64

def extract_words_from_json(json_data):
    words = []
    for item in json_data:
        try:
            data = json.loads(item)
            response = data['response']
            response_words = response.split()
            words.extend(response_words)
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
        except KeyError as e:
            print(f"KeyError: {e} not found in JSON data")
    return words

# Alter the window size to keep track of the past interactions
memory = ConversationBufferWindowMemory(k=3)

def call_llava_api(image_base64, prompt):
    history = memory.load_memory_variables({})["history"]
    full_prompt = f"{history}\nUser: {prompt}\nBot:"
    url = "http://localhost:11434/api/generate"
    payload = {
        "model": "new-cards-llm",
        "prompt": full_prompt + " Respond within 20 words, crisp and clear as if you are reporting back the situation.",
        "images": [image_base64]
    }
    headers = {
        'Content-Type': 'application/json'
    }
    try:
        response = requests.post(url, json=payload)
        response_lines = response.text.strip().split('\n')
        full_response = ''.join(json.loads(line)['response'] for line in response_lines if 'response' in json.loads(line))
        response.raise_for_status()

        # Update memory with the current interaction
        memory.save_context({"input": prompt}, {"output": full_response})   
        return full_response
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")

def extract_sentences(text, num_sentences=2):
    sentences = text.split('. ')
    return '. '.join(sentences[:num_sentences])

def clean_split_words(text):
    cleaned_text = re.sub(r'\b(\w+)\s+(\w+)\b', lambda m: m.group(1) + m.group(2) if len(m.group(2)) < 4 else m.group(0), text)
    return cleaned_text

def generate_and_play_audio(text):
    try:
        # Prepares the command to run the mimic TTS
        audio_file = "output_audio.wav"
        voice_file = "/path/to/mimic1/voices/cmu_us_slt.flitevox"
        command = f'/path/to/mimic1/mimic -t "{text}" -voice {voice_file} -o {audio_file}'
        os.system(command)
        audio_data, samplerate = sf.read(audio_file)  
        sd.play(audio_data, samplerate=samplerate)
        sd.wait()  # Wait until audio is finished playing
    except Exception as e:
        print(f"Error generating or playing audio: {e}")

def check_obstacle():
    client = MongoClient("mongodb://localhost:27017/")
    db = client.robot_messaging
    robots_collection = db.robots

    robot_data = robots_collection.find_one({"robot_name": "spot_3"})
    if robot_data and robot_data.get("obstacle", "n") == "n":
        return True
    return False

def main():
    global current_url_index
    model = Model(lang="en-us")
    with sd.RawInputStream(samplerate=16000, blocksize=2000, dtype="int16", channels=1, callback=callback):
        print("#" * 80)
        print("Listening for commands...")
        print("#" * 80)
        rec = KaldiRecognizer(model, 16000)
        recognized_text = ""  
        last_recognized_text = ""
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
                    print(recognized_text)
                except (json.JSONDecodeError, KeyError):
                    recognized_text = ""
            else:
                print("No valid input. Waiting for commands...")

            if recognized_text and recognized_text != "exit" and recognized_text!= last_recognized_text:
                print("Recognized text:", recognized_text)
                last_recognized_text = recognized_text
                frame = get_frame_from_video_stream(urls[current_url_index])
                if frame is None:
                    print("Error: Unable to capture frame from video stream.")
                elif "switch" in recognized_text or "stream" in recognized_text:
                        current_url_index = (current_url_index + 1) % len(urls)
                        print("Switching stream now.")
                        generate_and_play_audio(f"Switching stream now.")
                else:
                    image_base64 = image_to_base64(frame)
                    prompt = recognized_text
                    result = call_llava_api(image_base64, prompt)
                    result = extract_sentences(result)
                    print("Response from API:", result)
                    generate_and_play_audio(result)
                  

if __name__ == "__main__":
    main()

