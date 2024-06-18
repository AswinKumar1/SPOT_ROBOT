import cv2
import numpy as np
import base64
import os
import threading
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import playsound
from gtts import gTTS
from gradio_client import Client, file
import json
import time
from datetime import datetime

api_name = "/answer_question_1"

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
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"Error: Unable to open video stream {url}")
        return None

    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame from video stream")
        return None
    
    # Release the capture
    cap.release()
    return frame

def save_image(image):
    # Define the file path for saving the image with a timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = f"captured_frame_{timestamp}.jpeg"
    cv2.imwrite(file_path, image)
    return file_path

def call_gradio_api(image, prompt):
    # Initialize the Gradio Client with the API URL
    client = Client("http://127.0.0.1:7860/")

    # Send a request to the API using the `predict` method
    result = client.predict(
        file(image),  # Image data
        prompt,  # Prompt text
        api_name=api_name  # Specify the API endpoint
    )

    # Return the result from the API
    return result

def extract_sentences(text, num_sentences=2):
    sentences = text.split('. ')
    return '. '.join(sentences[:num_sentences])

def generate_and_play_audio(text):
    output_file_path = 'output.mp3'
   
    try:
        tts = gTTS(text=text, lang='en')
        tts.save(output_file_path)
       
        play_thread = threading.Thread(target=playsound.playsound, args=(output_file_path,))
        play_thread.start()
        play_thread.join()
    except KeyboardInterrupt:
        print("Process interrupted. Exiting.")
    finally:
        if os.path.exists(output_file_path):
            os.remove(output_file_path)

def main():
    model = Model(lang="en-us")

    with sd.RawInputStream(samplerate=16000, blocksize=4000, dtype="int16",channels=1, callback=callback):
        print("#" * 80)
        print("Listening for commands...")
        print("#" * 80)

        rec = KaldiRecognizer(model, 16000)
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = rec.Result()
               
                try:
                    recognized_text = json.loads(result)["text"].strip().lower()
                except (json.JSONDecodeError, KeyError):
                    recognized_text = ""
               
                if recognized_text and recognized_text != "exit":
                    print("Recognized text:", recognized_text)

                    # Capture frame from video stream
                    frame = get_frame_from_video_stream("https://192.168.1.31:8000/video_feed")
                    if frame is not None:
                        image = save_image(frame)
                        print(f"Image saved at {image}")

                        # Define the prompt
                        prompt = recognized_text

                        # Call Gradio API
                        result = call_gradio_api(image, prompt)
                        if result:
                            try:
                                result = extract_sentences(result)
                                print("Response from API:", result)
                                generate_and_play_audio(result)
                            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                                print("Error:", e)
                    else:
                        print("No frame captured from video stream")
                else:
                    print("No valid input. Waiting for commands...")

if __name__ == "__main__":
    main()
