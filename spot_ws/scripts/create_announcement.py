from pymongo import MongoClient, UpdateOne
import playsound
import os
import threading
from gtts import gTTS
import time

# MongoDB connection configuration
mongo_host = '192.168.1.64'  # Replace with your MongoDB host IP
mongo_port = 27017  # Default MongoDB port
mongo_db = 'robot_messaging'  # Replace with your MongoDB database name
collection_name = 'robots'  # MongoDB collection name

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

def check_sdk_and_play_announcement():
    try:
        # Connect to MongoDB
        client = MongoClient(mongo_host, mongo_port)
        db = client[mongo_db]
        collection = db[collection_name]

        # Query MongoDB for SDK column where robot_name is "spot_1"
        query = {"robot_name": "spot_1"}
        projection = {"sdk": 1, "announcement": 1, "_id": 0}  # Only retrieve sdk and announcement fields

        robot = collection.find_one(query, projection)

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
                    collection.update_one(update_query, update_operation)
                   
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
        check_sdk_and_play_announcement()

    except Exception as e:
        print(f"Error: {e}")
