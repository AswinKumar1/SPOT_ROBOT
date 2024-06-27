from pymongo import MongoClient
import playsound
import os
import threading
from gtts import gTTS
import time

# MongoDB connection configuration
mongo_host = '192.168.1.14'  
mongo_port = 27017  
mongo_db = 'robot_messaging'  
collection_name_1 = 'robots'  
collection_name_2 = 'robot_announcement'

def generate_and_play_audio(text):
    print(text)  # Print the text to be announced
    output_file_path = 'output.mp3'
   
    try:
        tts = gTTS(text=text, lang='en')
        tts.save(output_file_path)
       
        play_thread = threading.Thread(target=playsound.playsound, args=(output_file_path,))
        play_thread.start()
        play_thread.join()
    except KeyboardInterrupt:
        print("Process interrupted. Exiting.")
    except Exception as e:
        print(f"Error generating or playing audio: {e}")
    finally:
        if os.path.exists(output_file_path):
            os.remove(output_file_path)

def check_bunker_and_play_announcement():
    client = None
    try:
        client = MongoClient(mongo_host, mongo_port, serverSelectionTimeoutMS=5000)
        db = client[mongo_db]
        robots_collection = db[collection_name_1]
        announcements_collection = db[collection_name_2]

        query = {"robot_name": "husky", "bunker": "yes"}
        robot = robots_collection.find_one(query)

        if robot:
            announcement_query = {"robot_name": "husky", "flag": "yes"}
            announcement_doc = announcements_collection.find_one(announcement_query)

            if announcement_doc:
                announcement = announcement_doc.get("announcement", "")
                
                if announcement:
                    for _ in range(10):
                        generate_and_play_audio(announcement)
                        time.sleep(1)
                else:
                    print("No announcement found for 'husky'")
            else:
                print("No announcement document found for 'husky'")
        else:
            print("Bunker is not set to 'yes' for 'husky'")

    except Exception as e:
        print(f"Error accessing MongoDB: {e}")

    finally:
        if client:
            client.close()  # Close MongoDB connection

if __name__ == "__main__":
    try:
        check_bunker_and_play_announcement()
    except Exception as e:
        print(f"Error: {e}")
