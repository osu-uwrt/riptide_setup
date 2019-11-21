try:
	from http.server import BaseHTTPRequestHandler,HTTPServer
except:
	from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import json
import socket
import threading
import time
import select
import sys

PORT_NUMBER = 2000
coproConection = None
coproAddress = '192.168.1.42'
print(str(sys.argv))
if len(sys.argv) > 1:
    coproAddress = sys.argv[1]
droppedCommand = False

class commandWaiter:
	def __init__(self, command):
		self.command = command
		self.event = threading.Event()
		self.response = None



#This class will handles any incoming request from
#the browser 
class myHandler(BaseHTTPRequestHandler):

    #Handler for the POST requests
    def do_POST(self):
        global droppedCommand
        global coproConection

        requestData = json.loads(self.rfile.read(int(self.headers['Content-Length'])))

        resultStr = ""

        if coproConection is None:
            self.send_response(503)
        else:
            result = processCommand(requestData)
            if result == None:
                self.send_response(503)
                droppedCommand = True
            else:
                self.send_response(200)
                resultStr = json.dumps(result)

        self.send_header('Content-type','text/html')
        self.send_header('Access-Control-Allow-Origin','*')
        self.send_header('Access-Control-Allow-Headers', 'content-type')
        self.end_headers()
        self.wfile.write(resultStr.encode())
        return

    #Handler for the OPTIONS requests
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Headers', 'content-type')
        self.send_header('Content-type','text/html')
        self.send_header('Access-Control-Allow-Origin','*')
        self.end_headers()
        return

toBeSentQueue = []
toBeReceivedQueue = []

def processCommand(byteArray):
	global coproConection
	global toBeSentQueue

	waiter = commandWaiter(byteArray)
	toBeSentQueue += [waiter]   # Enqueue data to be sent later
	waiter.event.wait(.5)    # Wait for a response
	return waiter.response


def background():
    global coproConection
    global toBeReceivedQueue
    global toBeSentQueue
    global droppedCommand

    inputBuffer = []

    while True:
        if coproConection != None:                  # If we are connected
            try:
                readable, writable, exceptional = select.select([coproConection], [coproConection], [coproConection], 0)
                if len(writable) > 0:                       # If we can send data...
                    if len(toBeSentQueue) > 0:              # And we have data to send...
                        toBeSent = toBeSentQueue.pop(0)    
                        command = toBeSent.command          # Send command with length prefix
                        command = [len(command) + 1] + command
                        coproConection.sendall(bytearray(command))
                        toBeReceivedQueue += [toBeSent]     # Wait for response
                
                if len(readable) > 0:
                    if droppedCommand == True:
                        raise Exception("Dropped a command")
                    data = coproConection.recv(1024)        # Get data
                    if data == None or len(data) == 0:      
                        raise TypeError
                    if sys.version_info < (3, 0):
                        data = list(map(ord, data))
                    inputBuffer += data                     # While we have a complete command in the buffer
                    while len(inputBuffer) > 0 and inputBuffer[0] <= len(inputBuffer):
                        w = toBeReceivedQueue.pop(0)                # Get the request this belongs to
                        response = inputBuffer[1 : inputBuffer[0]]
                        w.response = response      # Send a response to the http protocol
                        w.event.set()
                        inputBuffer = inputBuffer[inputBuffer[0]:]
                
                if len(exceptional) > 0:
                    raise TypeError
            except Exception as exc:
                print("Lost copro connection")
                print(exc)
                coproConection.sendall(bytearray([0]))
                coproConection.close()
                coproConection = None
                inputBuffer = []
                time.sleep(1)
                droppedCommand = False          
        else:
            inputBuffer = []                        # If we are not connected, clear all buffers and queues
            while len(toBeReceivedQueue) != 0:
                toBeReceivedQueue.pop().event.set()
            while len(toBeSentQueue) != 0:  
                toBeSentQueue.pop().event.set()

            try:
                coproConection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                coproConection.settimeout(1)
                coproConection.connect((coproAddress, 50000))
            except:
                coproConection = None           # If it failed, clear queue
        time.sleep(0.01)
                

                

try:
	#Create a web server and define the handler to manage the
	#incoming request
	server = HTTPServer(('', PORT_NUMBER), myHandler)
	print ('Started httpserver on port ' , PORT_NUMBER)

	b = threading.Thread(name='background', target=background)
	b.daemon = True
	b.start()

	#Wait forever for incoming htto requests
	server.serve_forever()

except KeyboardInterrupt:
    print ('^C received, shutting down the web server')
    server.socket.close()
    coproConection.sendall(bytearray([0]))
    coproConection.close()
