import pymongo
import folium
import webbrowser

browser_opened = False

# Function to retrieve location data from MongoDB and plot it
def plot_locations_from_db():
    client = pymongo.MongoClient("mongodb://localhost:27017/")
    db = client["robot_locations"]
    collection = db["locations"]
    locations = collection.find()

    m = folium.Map(location=[0, 0], zoom_start=10)  # Initialize map
    for location in locations:
        latitude = location["latitude"]
        longitude = location["longitude"]
        folium.Marker([latitude, longitude], popup='Robot Location').add_to(m)

    file_path = "/map/map.html"
    m.save(file_path)
    if not browser_opened:
        webbrowser.open(file_path)
        browser_opened = True
    client.close()

if __name__ == "__main__":
    plot_locations_from_db()
