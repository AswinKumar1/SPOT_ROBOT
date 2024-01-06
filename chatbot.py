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
    finally:
        is_listening = True  # Resume listening after audio playback
        os.remove(output_file_path)

MAX_RESPONSE_LINES = 3

def get_confirmation(q, rec):
    check_response = "Do you want me to keep going?"
    generate_and_play_audio(f'<speak>{check_response}</speak>')
    # Pass the recording queue into a function to see if the user wants to continue the prompt response with a YES or NO. 
    response = recognize_speech(q, rec)  
    return response.lower().startswith("y")

def recognize_speech(q, rec):
    while True:
        try:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = rec.Result()
                recognized_text = json.loads(result)["text"].strip().lower()

                if recognized_text and recognized_text != "exit":
                    print("Recognized text:", recognized_text)
                    return recognized_text
                else:
                    print("No valid input. Waiting for commands...")
        except Exception as e:
            print("Error in recognize_speech:", str(e))


def play_openai_output(user_question):
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
        response_lines = openai_output.split('.')[:MAX_RESPONSE_LINES]
        limited_response = '\n'.join(response_lines)
        generate_and_play_audio(f'<speak>{limited_response}</speak>')
        # If the response is beyond MAX_RESPONSE_LINES check with the user for continuation. 
        if get_confirmation(q, rec):
            new_response_lines = openai_output.split('\n')[MAX_RESPONSE_LINES:]
            generate_and_play_audio(f'<speak>{new_response_lines}</speak>')
        else:
            sys.exit()

    
if __name__ == "__main__":
    try:
        model = Model(lang="en-us")

        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1, callback=callback):
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

                    if recognized_text.startswith(start_word.lower()) and recognized_text != "exit":
                        print("Recognized text:", recognized_text)
                        play_openai_output(recognized_text)
                    else:
                        print("No valid input. Waiting for commands...")

    except KeyboardInterrupt:
        print("\nDone")
    except Exception as e:
        print(type(e).__name__ + ": " + str(e))
