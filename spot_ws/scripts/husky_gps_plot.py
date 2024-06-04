import rospy
from nmea_msgs.msg import Sentence
import pymongo

robot_name = "husky"

# Callback function to handle incoming GPS data
def gps_callback(msg):
    nmea_sentence = msg.sentence
    if nmea_sentence.startswith('$GPRMC'):
        latitude, longitude, true_direction = parse_nmea_sentence(nmea_sentence)
        if latitude is not None and longitude is not None and true_direction is not None:
            save_to_db(latitude, longitude, true_direction)

# Function to parse NMEA RMC sentence and extract latitude, longitude, and true direction
def parse_nmea_sentence(nmea_sentence):
    parts = nmea_sentence.split(',')
    if parts[0] == '$GPRMC' and parts[2] == 'A':  # 'A' indicates valid data
        latitude = float(parts[3][:2]) + float(parts[3][2:]) / 60.0
        if parts[4] == 'S':
            latitude = -latitude
        longitude = float(parts[5][:3]) + float(parts[5][3:]) / 60.0
        if parts[6] == 'W':
            longitude = -longitude
        true_direction = float(parts[8]) if parts[8] else None
        print({"latitude": latitude}) # for debugging purposes
        print({"longitude": longitude}) # for debugging purposes
        print({"true_direction": true_direction}) # for debugging purposes
        return latitude, longitude, true_direction
    else:
        return None, None, None

hostname = '192.168.1.16'
port = 27017

# Function to save location data to MongoDB
def save_to_db(latitude, longitude, true_direction):
    client = pymongo.MongoClient(hostname, port)
    db = client["robot_messaging"]
    collection = db["robots"]
    try:
        query = {'robot_name': robot_name}
        robot_exist = collection.find_one(query)
        if robot_exist:
            location_data = {"$set": {"latitude": latitude, "longitude": longitude, "true_direction": true_direction}}
            collection.update_one(query, location_data)
        client.close()
    except Exception as e:
        print(e)

def main():
    rospy.init_node('gps_mapping_node', anonymous=True)
    rospy.Subscriber('/nmea_sentence', Sentence, gps_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
