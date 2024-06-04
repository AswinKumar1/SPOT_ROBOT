import pyrealsense2 as rs
import numpy as np
import cv2
import os
import threading
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import playsound
from gtts import gTTS
from gradio_client import Client
import json

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

def save_image(image):
    # Define the file path for saving the image
    file_path = "temp.jpeg"

    # Save the image
    cv2.imwrite(file_path, image)

    # Return the file path
    return file_path

def call_gradio_api(image, prompt):
    # Initialize the Gradio Client with the API URL
    client = Client("http://127.0.0.1:7860/")

    # Send a request to the API using the `predict` method
    result = client.predict(
        image,  # Image data
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

                    # Capture frame from RealSense camera
                    frame = get_frame_from_realsense()
                    if frame is not None:
                        # Convert captured frame to jpeg image
                        image = save_image(frame)

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
                        print("No frame captured from RealSense camera")
                else:
                    print("No valid input. Waiting for commands...")

if __name__ == "__main__":
    main()
