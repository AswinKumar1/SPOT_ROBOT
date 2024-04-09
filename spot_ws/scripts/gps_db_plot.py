import rospy
from nmea_msgs.msg import Sentence
import pymongo

# Callback function to handle incoming GPS data
def gps_callback(msg):
    nmea_sentence = msg.sentence
    if nmea_sentence.startswith('$GPGGA'):
        latitude, longitude = parse_nmea_sentence(nmea_sentence)
        if latitude is not None and longitude is not None:
            save_to_mongodb(latitude, longitude)

# Function to parse NMEA sentence and extract latitude and longitude
def parse_nmea_sentence(nmea_sentence):
    parts = nmea_sentence.split(',')
    if parts[0] == '$GPGGA':
        latitude = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
        if parts[3] == 'S':
            latitude = -latitude
        longitude = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
        if parts[5] == 'W':
            longitude = -longitude
        return latitude, longitude
    else:
        return None, None

# Function to save location data to MongoDB
def save_to_db(latitude, longitude):
    client = pymongo.MongoClient("mongodb://localhost:27017/")
    db = client["robot_locations"]
    collection = db["locations"]
    location_data = {"latitude": latitude, "longitude": longitude}
    collection.insert_one(location_data)
    client.close()

def main():
    rospy.init_node('gps_mapping_node', anonymous=True)
    rospy.Subscriber('/nmea_sentence', Sentence, gps_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
