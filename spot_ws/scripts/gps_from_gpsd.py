import pymongo
from gpsdclient import GPSDClient 
import socket

hostname = '192.168.1.16'
port = 27017
timeout_seconds = 60

# Function to save location data to MongoDB
def save_to_db(latitude, longitude, track):
    client = pymongo.MongoClient(hostname, port)
    db = client["robot_messaging"]
    collection = db["robots"]
    try:
        query = {'robot_name': "spot_2"}
        robot_exist = collection.find_one(query)
        if robot_exist:
            location_data = {"latitude": latitude, "longitude": longitude, "true_direction": track}
            collection.update_one(query, {"$set": location_data}, upsert=False)
        client.close()
    except Exception as e:
        print(e)

if __name__ == "__main__":
    while True:
        try:
            with GPSDClient(host="127.0.0.1", timeout=timeout_seconds) as client:
                for result in client.dict_stream(convert_datetime=True, filter=["TPV"]):
                    latitude = float(result.get("lat", "n/a"))
                    longitude = float(result.get("lon", "n/a"))
                    track = float(result.get("track", "n/a"))
                    print("latitude: %s" % result.get("lat", "n/a"))
                    print("longitude: %s" % result.get("lon", "n/a"))
                    print("Track: %s" % result.get("track", "n/a"))
                    save_to_db(latitude, longitude, track)
        except socket.timeout:
            print("Device connection timed out. Please check the device.")
        except Exception as e:
            print("An error occurred:", e)
