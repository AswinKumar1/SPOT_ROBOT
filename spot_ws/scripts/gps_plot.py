#!/usr/bin/python3 

import rospy
from nmea_msgs.msg import Sentence
import folium
import webbrowser
import threading

latitude = None
longitude = None

# Flag to check if web browser is already opened
browser_opened = False

# Callback function
def gps_callback(msg):
    global latitude, longitude
    nmea_sentence = msg.sentence
    if nmea_sentence.startswith('$GPGGA'):
        latitude, longitude = parse_nmea_sentence(nmea_sentence)
        if latitude is not None and longitude is not None:
            rospy.loginfo("Latitude: %f, Longitude: %f", latitude, longitude)

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

def plot_location():
    global latitude, longitude, browser_opened
    if latitude is not None and longitude is not None:
        m = folium.Map(location=[latitude, longitude], zoom_start=25)
        folium.Marker([latitude, longitude], popup='GPS Location', icon=folium.Icon(icon="cloud"),).add_to(m)
        file_path = "/home/cards/Desktop/spot-sdk/python/examples/gps_service/gps_location_map.html"
        m.save(file_path)
        if not browser_opened:
            webbrowser.open(file_path)
            browser_opened = True
    threading.Timer(1, plot_location).start()

def main():
    rospy.init_node('gps_mapping_node', anonymous=True)
    rospy.Subscriber('/nmea_sentence', Sentence, gps_callback)
    plot_location() 
    rospy.spin()

if __name__ == "__main__":
    main()
