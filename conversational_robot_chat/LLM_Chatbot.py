import pyrealsense2 as rs
import numpy as np
import cv2
import base64
import requests
import json
import os
import threading
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from google.cloud import texttospeech
import playsound
from gtts import gTTS

# Load the configuration from the JSON file
with open('key.json', 'r') as config_file:
    config = json.load(config_file)

# Set the path to your Google Cloud service account key file
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = config['google_cloud_key_file']

# Initialize the Google Text-to-Speech client
client = texttospeech.TextToSpeechClient()

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

def get_frame_from_realsense():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        return color_image
    finally:
        # Stop streaming
        pipeline.stop()

def image_to_base64(image):
    _, buffer = cv2.imencode('.png', image)
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    return image_base64

def call_llava_api(image_base64, prompt):
    url = "http://localhost:11434/api/generate"
    payload = {
        "model": "spot-vlm",
        "prompt": prompt,
        "images": [image_base64]
    }

    headers = {
        'Content-Type': 'application/json'
    }

    try:
        response = requests.post(url, data=json.dumps(payload), headers=headers)
        print("Response status code:", response.status_code)
        response.raise_for_status()  # Raise an exception for HTTP errors
        return response.content  # Return raw response content
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")

def extract_sentences(text, num_sentences=3):
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

                    # Capture frame from RealSense camera
                    frame = get_frame_from_realsense()
                    if frame is not None:
                        # Convert captured frame to base64
                        image_base64 = image_to_base64(frame)

                        # Define the prompt
                        prompt = recognized_text

                        # Call Llava API
                        result = call_llava_api(image_base64, prompt)
                        if result:
                            try:
                                responses = result.decode('utf-8').strip().split('\n')
                                final_response = ' '.join([json.loads(resp)['response'] for resp in responses if resp.strip()])
                                final_response = extract_sentences(final_response, num_sentences=3)
                                print("Response from API:", final_response)
                                generate_and_play_audio(final_response)
                            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                                print("Error:", e)
                    else:
                        print("No frame captured from RealSense camera")
                else:
                    print("No valid input. Waiting for commands...")

if __name__ == "__main__":
    main()
