import SimpleHTTPServer
import SocketServer
import os

# Allowed file extensions
ALLOWED_EXTENSIONS = ('.png', '.ppm')

class CustomHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Get the requested file path
        file_path = self.translate_path(self.path)

        # If it's a directory (i.e., no file specified), deny access
        if os.path.isdir(file_path):
            self.send_response(403)
            self.end_headers()
            self.wfile.write("403 Forbidden: No index page available.\n")
            return
        
        # Check if the file has an allowed extension
        if not file_path.endswith(ALLOWED_EXTENSIONS):
            self.send_response(403)
            self.end_headers()
            self.wfile.write("403 Forbidden: Only .png and .ppm files are allowed.\n")
            return
        
        # If valid, serve the file
        return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

# Set server port
PORT = 8000

# Create and start the server
httpd = SocketServer.TCPServer(("", PORT), CustomHandler)
print("Serving .png and .ppm files on port", PORT)
httpd.serve_forever()