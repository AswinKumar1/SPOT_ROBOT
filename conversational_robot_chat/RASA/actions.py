import sys
import os
import math
import time
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

# Ensure the 'actions' directory is in the Python path
sys.path.append('/home/blue/rasa/actions/spot_wrapper')

from spotWrapper import SpotWrapper

class ActionLeaseSpot(Action):
    def name(self) -> str:
        return "action_lease_spot"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        dispatcher.utter_message(text="Black Robot lease control in place")
        return []

class ActionStandSpot(Action):
    def name(self) -> str:
        return "action_stand_spot"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        spot.stand_spot()
        dispatcher.utter_message(text="Making the robot stand.")
        return []

class ActionSitSpot(Action):
    def name(self) -> str:
        return "action_sit_spot"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        spot.sit_spot()
        dispatcher.utter_message(text="Making the robot sit.")
        return []

class ActionBatterySpot(Action):
    def name(self) -> str:
        return "action_battery_spot"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        # Implement battery check logic here
        dispatcher.utter_message(text="Checked the battery status.")
        return []

class ActionMoveForward(Action):
    def name(self) -> str:
        return "action_move_forward"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        distance = float(tracker.get_slot("distance"))
        spot = SpotWrapper()
        spot.move_spot(distance, 0, distance)  # Assuming duration is proportional to distance
        dispatcher.utter_message(text=f"Moving the robot forward by {distance} meters.")
        return []

class ActionMoveBackward(Action):
    def name(self) -> str:
        return "action_move_backward"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        distance = float(tracker.get_slot("distance"))
        spot = SpotWrapper()
        spot.move_spot(-distance, 0, distance)  # Assuming duration is proportional to distance
        dispatcher.utter_message(text=f"Moving the robot backward by {distance} meters.")
        return []

class ActionTurnLeft(Action):
    def name(self) -> str:
        return "action_turn_left"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        spot.turn_spot(math.radians(90), 4)  # Assuming duration for 90 degrees turn
        dispatcher.utter_message(text="Turning the robot to the left.")
        return []

class ActionTurnRight(Action):
    def name(self) -> str:
        return "action_turn_right"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        spot.turn_spot(math.radians(-90), 4)  # Assuming duration for 90 degrees turn
        dispatcher.utter_message(text="Turning the robot to the right.")
        return []

class ActionStopSpot(Action):
    def name(self) -> str:
        return "action_stop_spot"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        spot = SpotWrapper()
        spot.stop_spot()
        dispatcher.utter_message(text="Stopping the robot.")
        return []

class ActionGreet(Action):
    def name(self) -> str:
        return "action_greet"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="Hello! How can I assist you today?")
        return []

class ActionGoodbye(Action):
    def name(self) -> str:
        return "action_goodbye"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="Goodbye! Have a great day!")
        return []

class ActionAffirm(Action):
    def name(self) -> str:
        return "action_affirm"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="Got it!")
        return []

class ActionDeny(Action):
    def name(self) -> str:
        return "action_deny"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="Okay, no problem.")
        return []

class ActionBotChallenge(Action):
    def name(self) -> str:
        return "action_bot_challenge"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="I am a bot, here to assist you!")
        return []

class ActionMoodGreat(Action):
    def name(self) -> str:
        return "action_mood_great"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="That's great to hear!")
        return []

class ActionMoodUnhappy(Action):
    def name(self) -> str:
        return "action_mood_unhappy"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: dict):
        dispatcher.utter_message(text="I'm sorry to hear that.")
        return []
