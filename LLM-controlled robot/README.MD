# SPOT-AI Robot Control and Interaction

This project demonstrates the integration of a Boston Dynamics Spot robot with a fine-tuned LLAVA v1.6 model, allowing for voice-controlled commands and image-based interactions. The system captures real-time images from the Spot's camera, processes commands using voice recognition, and interacts with the Spot robot via a custom LLAVA model (`spot-vlm`).

## Features

- Voice recognition for command input using the VOSK library
- Real-time image capture from the Spot robot using an Intel RealSense camera
- Integration with Google Cloud Text-to-Speech for audio responses
- Fine-tuned LLAVA v1.6 model for interpreting and generating responses to commands
- Command execution to control the Spot robot's movements and actions

### Prerequisites

- Python 3.7+
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [VOSK](https://github.com/alphacep/vosk-api)
- [Google Cloud Text-to-Speech](https://cloud.google.com/text-to-speech/docs/setup)
- [gTTS](https://gtts.readthedocs.io/en/latest/)
- [playsound](https://github.com/TaylorSMarks/playsound)
- Docker

### Installation

1. Clone the repository:

   ```sh
   git clone https://github.com/yourusername/spot-ai-control.git
   cd spot-ai-control
  ```

2. Install required Python packages:

   ```sh
pip install pyrealsense2 numpy opencv-python requests sounddevice vosk gtts playsound
   ```

3. Use Google Text-to-Speech module

4. Build and run the Docker container for LLAVA:

   ```sh
docker build -t spot-vlm .
docker run -d -p 11434:11434 spot-vlm
   ```

5. Start the main script:

```sh
python spot_control.py
```

The system will listen for voice commands. Speak a command to control the Spot robot. For example:

- "Move the black spot forward by 3 meters."
- "Turn the red spot to the left for 2 seconds."
- "What do you see in front of you?"

The system captures an image from the RealSense camera, processes the command using the LLAVA API, and generates an appropriate response.

### Commands: 

Here are some example commands you can use:

+ move_spot(self, 0.5, 1) - Moves the Spot robot forward.
+ move_spot(self, -0.5, 1) - Moves the Spot robot backward.
+ sit_spot() - Makes the Spot robot sit.
+ stand_spot() - Makes the Spot robot stand.
+ turn_spot(self, 0.5, 3) - Turns the Spot robot to the right.
+ turn_spot(self, -0.5, 3) - Turns the Spot robot to the left.
+ stop_spot() - Stops the Spot robot.
+ lease_spot() - Leases control of the Spot robot.
+ rotate_spot() - Rotates the Spot robot continuously.
+ search_spot() - Rotates the Spot robot to search for nearby objects.
+ battery_spot() - Provides the current battery level of the Spot robot.
+ call_stitch_and_save() - Captures an image and describes the scene in front of the Spot robot.

### Acknowledgements
+ Boston Dynamics Spot Robot
+ Intel RealSense
+ VOSK Speech Recognition
+ Google Text-to-Speech
+ LLAVA Model
