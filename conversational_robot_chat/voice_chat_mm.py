import cv2
import numpy as np
import base64
import os
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import playsound
from gtts import gTTS
import json
from datetime import datetime
import requests
import re
from pymongo import MongoClient

q = queue.Queue()

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

def call_llava_api(image_base64, prompt):
    url = "http://localhost:11434/api/generate"
    payload = {
        "model": "cards-llm",
        "prompt": prompt,
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
    output_file_path = 'output.mp3'
    try:
        tts = gTTS(text=text, lang='en')
        tts.save(output_file_path)
        playsound.playsound(output_file_path)
    except Exception as e:
        print(f"Error generating or playing audio: {e}")
    finally:
        if os.path.exists(output_file_path):
            os.remove(output_file_path)

def check_obstacle():
    client = MongoClient("mongodb://localhost:27017/")
    db = client.robot_messaging
    robots_collection = db.robots

    robot_data = robots_collection.find_one({"robot_name": "spot_3"})
    
    if robot_data and robot_data.get("obstacle", "y") == "n":
        return True
    return False

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

                if recognized_text and recognized_text != "exit":
                    print("Recognized text:", recognized_text)

                    if "millimeter" in recognized_text or "radar" in recognized_text:
                        if check_obstacle():
                            generate_and_play_audio("No obstacle detected, the path is clear.")
                        else:
                            generate_and_play_audio("Obstacle detected, please proceed with caution.")
                    else:
                        frame = get_frame_from_video_stream("http://192.168.1.42:8000/video_feed")

                        if frame is None:
                            print("Error: Unable to capture frame from video stream.")
                        else:
                            image_base64 = image_to_base64(frame)
                            prompt = recognized_text
                            result = call_llava_api(image_base64, prompt)
                            result = extract_sentences(result) + '. over.'
                            print("Response from API:", result)
                            generate_and_play_audio(result)
                else:
                    print("No valid input. Waiting for commands...")

if __name__ == "__main__":
    main()
