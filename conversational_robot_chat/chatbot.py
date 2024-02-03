import openai
import json
import os
import re
import threading
from google.cloud import texttospeech
import playsound
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# Load the configuration from the JSON file
with open('key.json', 'r') as config_file:
    config = json.load(config_file)

# Set the path to your Google Cloud service account key file and OpenAI API key
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = config['google_cloud_key_file']
openai.api_key = config['openai_api_key']
gpt_model = config["model"]

# Initialize the Google Text-to-Speech client
client = texttospeech.TextToSpeechClient()

q = queue.Queue()
is_listening = True  # Global flag to control listening state
start_word = 'SPOT'

def callback(indata, frames, time, status):
    global is_listening
    if status:
        print(status, file=sys.stderr)
    if is_listening:
        q.put(bytes(indata))

def generate_and_play_audio(text):
    global is_listening
    synthesis_input = texttospeech.SynthesisInput(ssml=text)
    voice1 = texttospeech.VoiceSelectionParams(
        language_code='en-US',
        name='en-US-Wavenet-D',
        ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3
    )
    try:
        response1 = client.synthesize_speech(
            input=synthesis_input,
            voice=voice1,
            audio_config=audio_config
        )
        output_file_path = 'output.mp3'
        with open(output_file_path, 'wb') as out_file:
            out_file.write(response1.audio_content)
        
        # Removing "<speak>" and "</speak>" from the beginning and end of the OpenAI response
        cleaned_response = re.sub(r'^<speak>', '', text, flags=re.IGNORECASE)
        cleaned_response = re.sub(r'</speak>$', '', cleaned_response, flags=re.IGNORECASE)

        is_listening = False  # Stop listening while playing audio
        playsound.playsound(output_file_path, block=True)  # Simplified audio playback
        #is_listening = True
    finally:
        is_listening = True  # Resume listening after audio playback
        os.remove(output_file_path)

OPT_RESPONSE_LINES = 3    # Optimal response lines. 
MAX_RESPONSE_LINES = 6    # Maximum no. of lines in response.

def get_confirmation(q, rec):  
    check_response = "Do you want me to keep going?"
    generate_and_play_audio(f'<speak>{check_response}</speak>')
    is_waiting_for_response = True 
    # Pass the recording queue into a function to delay the response recognition period.  
    while is_waiting_for_response:
        print("Inside the waiting for response loop")
        #generate_and_play_audio(f'<speak>{check_response}</speak>')
        response = recognize_speech(q, rec)
        if response is not None:
            print("Inside the response is not none condition")
            is_waiting_for_response = False 
            return response
            

def recognize_speech(q, rec):   # Checks if the User wants to continue the response with a yes or no/stop. 
    while True:
        try:
            data = q.get()  # Collect User's response
        except queue.Empty:
        # Handles the case where there's no data or queue being empty
            print("No valid input received. Waiting for commands...")
            continue
        except Exception as e:
            print(f"Error in recognize_speech: {e}")
            continue
        
        try:
            if rec.AcceptWaveform(data):
                result = rec.Result()
                recognized_text = json.loads(result)["text"].strip().lower()
                # Checks the response for a word starting with "Y" or stop words. 
                if recognized_text.startswith("y") and recognized_text != "exit":
                    print("Recognized text:", recognized_text)
                    return True
                elif "no" in recognized_text or "stop" in recognized_text:
                    return False
                else:
                    print("No valid input. Waiting for commands...")
                    return None
        except Exception as e:
            print(f"Error processing recognized text: {e}")
            continue


def play_openai_output(rec, user_question):
    if user_question:
        # To remove HTML-like tags from the recognized text using regular expression
        cleaned_question = re.sub(r'<\s*speak\s*>\s*|\s*<\s*/\s*speak\s*>\s*', '', user_question, flags=re.IGNORECASE)
        print(f"Cleaned Question: {cleaned_question}")  # Added this line for debugging
        
        response = openai.ChatCompletion.create(
            model=gpt_model,
            messages=[
                {"role": "system", "content": "You are a SPOT robot belonging to CARDS labarotary."},
                {"role": "user", "content": cleaned_question},
            ],
        )
        openai_output = response['choices'][0]['message']['content']
        # Output only first 3 sentences of the responses. 
        response_lines = openai_output.split('.')[:OPT_RESPONSE_LINES]
        limited_response = '\n'.join(response_lines)
        generate_and_play_audio(f'<speak>{limited_response}</speak>')
        # If the response is beyond OPT_RESPONSE_LINES check with the user for continuation. 
        if get_confirmation(q, rec):
            new_response_lines = openai_output.split('\n')[OPT_RESPONSE_LINES:MAX_RESPONSE_LINES]
            generate_and_play_audio(f'<speak>{new_response_lines}</speak>')
        

    
if __name__ == "__main__":
    try:
        model = Model(lang="en-us")

        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1, callback=callback):
            print("#" * 80)
            print("Listening for commands...")
            print("#" * 80)

            rec = KaldiRecognizer(model, 16000)
            while True:
                try:
                    # Set a timeout for the get operation
                    data = q.get(timeout=5)  # Timeout is kept for 3 seconds for now. 
                except queue.Empty:
                    # Handles the case where there's no data within the timeout
                    print("No valid input within the timeout. Waiting for commands...")
                    continue

                if rec.AcceptWaveform(data):
                    result = rec.Result()
                    try:
                        recognized_text = json.loads(result)["text"].strip().lower()
                    except (json.JSONDecodeError, KeyError):
                        recognized_text = ""

                    if recognized_text.startswith(start_word.lower()) and recognized_text != "exit":
                        if recognized_text[len(start_word):] == "":
                            print("Inside wake_up_prompt loop")
                            wake_up_prompt = "Hello, I'm here and ready to assist. Just say SPOT to start a conversation."
                            generate_and_play_audio(f'<speak>{wake_up_prompt}</speak>')
                        else:
                            recognized_text = recognized_text[len(start_word):]
                            print("Recognized text:", recognized_text)
                            play_openai_output(rec, recognized_text)
                    else:
                        print("No valid input. Waiting for commands...")

    except KeyboardInterrupt:
        print("\nDone")
    except Exception as e:
        print(type(e).__name__ + ": " + str(e))

