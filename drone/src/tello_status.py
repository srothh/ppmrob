# set up  socketserver.UDPServer to listen on port 8890
import socketserver

class MyUDPHandler(socketserver.BaseRequestHandler):
 
    def handle(self):
        data = self.request[0].strip()
        print(data)
 
if __name__ == "__main__":
    HOST, PORT = "", 8890
    with socketserver.UDPServer((HOST, PORT), MyUDPHandler) as server:
        server.serve_forever()