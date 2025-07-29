from http.server import HTTPServer, BaseHTTPRequestHandler
import rospy
from nav_msgs.msg import OccupancyGrid
import json

class MapServer(BaseHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.map_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        super().__init__(*args, **kwargs)

    def map_callback(self, msg):
        self.map_data = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "z": msg.info.origin.position.z
            },
            "data": list(msg.data)  # hrta ca listă (0=liber, 100=ocupat, -1=necunoscut)
        }

    def do_GET(self):
        if self.path == "/map":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(self.map_data or {}).encode())
        else:
            self.send_response(404)
            self.end_headers()

def mapServerProcess(port=8083):
    rospy.init_node("map_server", anonymous=True)
    server_address = ("0.0.0.0", port)
    httpd = HTTPServer(server_address, MapServer)
    print(f"Starting map server on http://0.0.0.0:{port}/map")
    httpd.serve_forever()