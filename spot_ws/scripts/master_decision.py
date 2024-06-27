from pymongo import MongoClient, UpdateOne
import playsound
import os
import threading
from gtts import gTTS
import time
from geopy.distance import geodesic  # Add this import for distance calculation

# MongoDB connection configuration
mongo_host = '192.168.1.64'  # Replace with your MongoDB host IP
mongo_port = 27017  # Default MongoDB port
mongo_db = 'robot_messaging'  # Replace with your MongoDB database name
robots_collection_name = 'robots'  # MongoDB collection name for robots
announcements_collection_name = 'robots_announcement'  # MongoDB collection name for announcements

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

def find_closest_robot(bunker_location, robots):
    closest_robot = None
    min_distance = float('inf')

    for robot in robots:
        robot_location = (robot['gps']['lat'], robot['gps']['long'])
        distance = geodesic(bunker_location, robot_location).meters

        if distance < min_distance:
            min_distance = distance
            closest_robot = robot['robot_name']
    
    return closest_robot

def check_bunker_and_dispatch():
    try:
        # Connect to MongoDB
        client = MongoClient(mongo_host, mongo_port)
        db = client[mongo_db]
        robots_collection = db[robots_collection_name]
        announcements_collection = db[announcements_collection_name]

        # Query MongoDB for any robot with a high-priority bunker
        bunker_query = {"bunker": {"$exists": True, "$ne": ""}}
        robots_with_bunker = list(robots_collection.find(bunker_query))

        if robots_with_bunker:
            # Generate and play "Bunker Detected" announcement
            generate_and_play_audio("Bunker Detected")

            bunker_robot = robots_with_bunker[0]
            bunker_location = (bunker_robot['gps']['lat'], bunker_robot['gps']['long'])

            # Query for spot robots and their locations
            spot_query = {"robot_name": {"$in": ["spot_1", "spot_2", "spot_3"]}}
            spot_robots = list(robots_collection.find(spot_query))

            if spot_robots:
                closest_robot = find_closest_robot(bunker_location, spot_robots)

                if closest_robot:
                    dispatch_message = f"Dispatching {closest_robot.replace('_', ' ').upper()}"
                    generate_and_play_audio(dispatch_message)

                    # Update the bunker robot to remove the bunker field after dispatch
                    update_query = {"_id": bunker_robot['_id']}
                    update_operation = {"$unset": {"bunker": ""}}
                    robots_collection.update_one(update_query, update_operation)
                else:
                    print("No closest robot found.")
            else:
                print("No spot robots found.")
        else:
            print("No bunker detected in the database.")

    except Exception as e:
        print(f"Error accessing MongoDB: {e}")

    finally:
        client.close()  # Close MongoDB connection

def check_sdk_and_play_announcement():
    try:
        # Connect to MongoDB
        client = MongoClient(mongo_host, mongo_port)
        db = client[mongo_db]
        announcements_collection = db[announcements_collection_name]

        # Query MongoDB for SDK column where robot_name is "spot_1"
        query = {"robot_name": "spot_1"}
        projection = {"sdk": 1, "announcement": 1, "_id": 0}  # Only retrieve sdk and announcement fields

        robot = announcements_collection.find_one(query, projection)

        if robot:
            sdk_enabled = robot.get("sdk", "no").lower()  # Default to "no" if sdk field is not present
            announcement = robot.get("announcement", "")

            if sdk_enabled == "yes":
                if announcement:
                    # Play the announcement 3 times
                    for _ in range(3):
                        generate_and_play_audio(announcement)
                        time.sleep(1)

                    # Update sdk flag to "no"
                    update_query = {"robot_name": "spot_1"}
                    update_operation = {"$set": {"sdk": "no"}}
                    announcements_collection.update_one(update_query, update_operation)
                   
                else:
                    print("No announcement found for 'spot_1'")
            else:
                print("SDK not enabled for 'spot_1'")

        else:
            print("Robot 'spot_1' not found in the database")

    except Exception as e:
        print(f"Error accessing MongoDB: {e}")

    finally:
        client.close()  # Close MongoDB connection

if __name__ == "__main__":
    try:
        check_bunker_and_dispatch()
        check_sdk_and_play_announcement()

    except Exception as e:
        print(f"Error: {e}")
