# Integration of Real-Time Speech Command and Visual Question Answering System (voice_chatbot_v2.py)

## Overview
This project integrates real-time speech command recognition with a visual question answering system using Gradio and an Intel RealSense camera. The system listens for voice commands, captures an image from the camera, processes the input through a machine learning model, and responds with synthesized speech.

## Components
### Gradio Interface (Colleague's Code):
-  Sets up a Gradio web interface for users to input an image and a text prompt. 
- Uses a pre-trained language model to generate responses based on the input.

## Real-Time Speech Command and Image Capture (My Code):
- Listens for voice commands using sounddevice and vosk.
- Captures an image from an Intel RealSense camera.
- Sends the image and recognized text to the Gradio API.
- Converts the API response to speech using gTTS and plays it.

## Workflow
### Voice Command Recognition:
- The system continuously listens for voice commands.
- Recognized commands are processed to form a text prompt.

## Image Capture:
- Upon recognizing a valid command, the system captures an image using the RealSense camera.

## API Interaction:
- The captured image and text prompt are sent to the Gradio API for processing.
- The API processes the input and returns a text response.

## Audio Response:
- The text response is converted to audio and played back to the user.

## Dependencies:
- PyTorch
- Gradio
- pyrealsense2
- OpenCV
- sounddevice
- vosk
- playsound
- gTTS
