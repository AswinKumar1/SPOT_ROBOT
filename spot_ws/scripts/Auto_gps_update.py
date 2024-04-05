import asyncio
import rospy
from std_msgs.msg import String
import folium
import websockets

latitude = None
longitude = None

# Callback function to handle incoming GPS data
def gps_callback(msg):
    global latitude, longitude
    nmea_sentence = msg.data
    latitude, longitude = parse_nmea_sentence(nmea_sentence)

# Function to parse NMEA sentence and extract latitude and longitude
def parse_nmea_sentence(nmea_sentence):
    parts = nmea_sentence.split(',')
    if parts[0] == '$GPGGA':
        latitude = float(parts[2])
        longitude = float(parts[4])
        return latitude, longitude
    else:
        return None, None

# Function to create and update the map
async def update_map(websocket, path):
    global latitude, longitude
    while True:
        if latitude is not None and longitude is not None:
            map_obj = folium.Map(location=[latitude, longitude], zoom_start=15)
            folium.Marker([latitude, longitude], popup='GPS Location').add_to(map_obj)
            map_obj.save('gps_location_map.html')
            with open('gps_location_map.html', 'r') as file:
                html_content = file.read()
                await websocket.send(html_content)
        await asyncio.sleep(1)  

def main():
    rospy.init_node('gps_mapping_node', anonymous=True)
    rospy.Subscriber('/nmea_sentence', String, gps_callback)
    start_server = websockets.serve(update_map, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(asyncio.gather(start_server))
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()
