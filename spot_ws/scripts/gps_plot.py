import rospy
from std_msgs.msg import String
import folium

# Callback function to handle incoming GPS data
def gps_callback(msg):
    nmea_sentence = msg.data
    latitude, longitude = parse_nmea_sentence(nmea_sentence)
    plot_location(latitude, longitude)

# Function to parse NMEA sentence and extract latitude and longitude
def parse_nmea_sentence(nmea_sentence):
    parts = nmea_sentence.split(',')
    if parts[0] == '$GPGGA':
        latitude = float(parts[2])
        longitude = float(parts[4])
        return latitude, longitude
    else:
        return None, None

# Function to plot location on the map
def plot_location(latitude, longitude):
    map_obj = folium.Map(location=[latitude, longitude], zoom_start=15)
    folium.Marker([latitude, longitude], popup='GPS Location').add_to(map_obj)
    map_obj.save('gps_location_map.html')

def main():
    rospy.init_node('gps_mapping_node', anonymous=True)
    rospy.Subscriber('/nmea_sentence', Sentence, gps_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
