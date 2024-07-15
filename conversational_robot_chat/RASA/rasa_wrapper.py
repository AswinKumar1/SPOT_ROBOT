import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import requests
import math
from spotWrapper import SpotWrapper

# Initialize the Spot robot wrapper
spot = SpotWrapper()

# Define a global queue
q = queue.Queue()

# Function to send text to Rasa and get the response
def get_rasa_response(command):
    url = "http://localhost:5005/webhooks/rest/webhook"  # Update if your Rasa server URL is different
    payload = {"sender": "user", "message": command}
    try:
        response = requests.post(url, json=payload)
        if response.status_code == 200:
            print("Rasa response:", response.json())  # Debug print to check Rasa response
            return response.json()
        else:
            print(f"Error connecting to Rasa: {response.status_code} - {response.text}")
            return []
    except requests.exceptions.RequestException as e:
        print(f"Failed to connect to Rasa server: {e}")
        return []

# Function to process Rasa responses and control the robot
def process_rasa_response(rasa_response):
    for resp in rasa_response:
        if 'text' in resp:
            print("Rasa text response:", resp['text'])  # Output the response from Rasa

        if 'action' in resp:
            action = resp['action']
            print(f"Executing action: {action}")  # Debug print to check action being executed
            try:
                if action == "action_stand_spot":
                    spot.stand_spot()
                elif action == "action_sit_spot":
                    spot.sit_spot()
                elif action == "action_move_forward":
                    distance = resp.get('entities', [{}])[0].get('value', 1.0)  # Ensure entity access is safe
                    spot.move_spot(distance, 0, 10)  # Use a fixed duration or parameterize
                elif action == "action_move_backward":
                    distance = resp.get('entities', [{}])[0].get('value', 1.0)
                    spot.move_spot(-distance, 0, 10)
                elif action == "action_turn_left":
                    spot.turn_spot(math.radians(90), 2)
                elif action == "action_turn_right":
                    spot.turn_spot(math.radians(-90), 2)
                elif action == "action_stop_spot":
                    spot.stop_spot()
                else:
                    print(f"Unknown action: {action}")
            except Exception as e:
                print(f"Failed to execute action {action}: {e}")

# Callback function to put the data into a queue
def callback(indata, frames, time, status):
    q.put(bytes(indata))

# Main function for speech recognition and processing
def main():
    model = Model(lang="en-us")  # Change language model if needed
    rec = KaldiRecognizer(model, 16000)

    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1, callback=callback):
        print("#" * 80)
        print("Listening for commands...")
        print("#" * 80)

        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = rec.Result()
                try:
                    recognized_text = json.loads(result)["text"].strip().lower()
                    print("Recognized command:", recognized_text)

                    if recognized_text.startswith("spot"):
                        command = recognized_text[5:].strip()
                        if command:
                            rasa_response = get_rasa_response(command)
                            if rasa_response:
                                process_rasa_response(rasa_response)
                except json.JSONDecodeError:
                    print("Failed to decode the recognition result.")
                except KeyError:
                    print("Failed to retrieve text from the recognition result.")
                except Exception as e:
                    print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
