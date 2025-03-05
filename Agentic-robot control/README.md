# Voice-Controlled Robot Agent

## Overview
This project implements a voice-controlled robot agent using the Vosk speech recognition engine, OpenAI's GPT-4o-mini model, and Boston Dynamics' Spot robot. The system allows users to issue voice commands to control the robot, enabling movement, stopping, standing, and sitting functionalities.

## Features
- **Voice Command Recognition**: Uses Vosk for real-time speech-to-text processing.
- **AI-Powered Command Execution**: Utilizes OpenAI's GPT-4o-mini to process and execute commands.
- **Robot Control**: Interacts with Spot via `SpotWrapper` to perform various actions.
- **Text-to-Speech Feedback**: Converts responses into audio feedback for better user interaction.
- **Sequential Task Execution**: Allows multiple commands to be processed and executed in sequence.

## Dependencies
Ensure you have the following dependencies installed:

- Python 3.x
- `vosk` for speech recognition
- `sounddevice` for audio processing
- `langchain`, `langchain_openai`, `langchain_core` for AI-based decision-making
- `spot_wrapper` for Spot robot interaction
- `dotenv` for environment variable management
- `pydub` and `simpleaudio` for audio playback
- `openai` for text-to-speech functionality

## Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/your-repo/voice-controlled-robot.git
   cd voice-controlled-robot
   ```
2. Install the required dependencies:
   ```sh
   pip install -r requirements.txt
   ```
3. Set up environment variables in a `.env` file:
   ```ini
   PLAYDIALOG_API_KEY=your_api_key
   PLAYDIALOG_USER_ID=your_user_id
   SERPAPI_API_KEY=your_serpapi_key
   ```

## Usage
1. Ensure that your Spot robot is powered on and connected.
2. Run the script:
   ```sh
   python agentic_spot.py
   ```
3. Speak commands into the microphone. Examples of commands:
   - "Stand up"
   - "Move forward"
   - "Move backward"
   - "Stop"
   - "Sit down"
4. The robot will execute the commands and provide voice feedback.

## AI Agent Execution
One of the key improvements in this project is the use of `tool_calling_agent` instead of `react_agent` for executing commands. The normal `react_agent` was unable to successfully execute commands sequentially. By utilizing `tool_calling_agent`, we ensure that multiple commands are processed and executed in the correct order without interruptions, making the robot's behavior more predictable and reliable. This approach significantly improves the efficiency of handling sequential tasks, such as moving forward and then stopping at the right time.

## Robot Control Functions
The following functions interact with the Spot robot:
- `move_robot_forward()`: Moves the robot forward.
- `move_robot_backward()`: Moves the robot backward.
- `stop_robot()`: Stops all movement.
- `stand_robot()`: Makes the robot stand up.
- `sit_robot()`: Makes the robot sit down.

## Troubleshooting
- **No audio detected?** Ensure your microphone is working and accessible.
- **Robot not responding?** Check Spot's connection and API keys.
- **Speech not recognized?** Ensure clear pronunciation and minimal background noise.



