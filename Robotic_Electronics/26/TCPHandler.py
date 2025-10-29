import network
import usocket as socket

class TCPHandler:

    # Constructor defines the basic network parameters and initiates connecting to the network
    def __init__(self, ssid, password, port, led=None):
        self.ssid = ssid
        self.pw = password
        self.port = port
        self.led = led
        self.station = network.WLAN(network.STA_IF)  # Creates a WLAN network object for connection
        self.station.active(True)                    # Activate the network object
        self.connectToNetwork()                      # Connect to the network
        
    # Method connectToNetwork() tries to make a connection with the network
    def connectToNetwork(self):                      # Fist step is to connect to a WIFI network
        try:
            self.state = 0                           # state 0: Connecting to the network
            self.station.connect(self.ssid, self.pw) # Connect to the network
            self.checkNetworkConnection()
        except:
            pass

    # Method checkNetworkConnection() checks if the ESP32 is connected to the network. If true, the IP-address
    # is printed, attribute state is set to 1, a new socked object is created and the ESP32 will listen for
    # incoming clients.        
    def checkNetworkConnection(self):
        if self.station.isconnected():               # First check is device is connected to the network
            print('IP-address of ESP32 =', self.station.ifconfig()[0])  # Print IP-address of ESP32
            self.state = 1                           # state 1: Connection with network is made, listen to incoming Client connection
            self.sock = socket.socket()              # Create a socket object
            self.sock.bind(('', self.port))          # Connect socket, allow any IP-address to connect
            self.sock.setblocking(False)             # Prevent program to halt if no client is connected
            self.sock.listen(1)                      # Maximum 1 (pending) connection is allowed
            self.listenToClients()

    # Method listenToClients() first checks if the ESP32 is connected to the network. If false, it wil try to
    # reconnect to the network. If true, it will try to create a socket object. If it succeeds, it will print
    # the IP-address of the client and set attribute state 2 and turn on the led.        
    def listenToClients(self):
        if self.station.isconnected():               # First check on network connection and client listener
            try:
                self.conn, addr = self.sock.accept() # Create a new socket (conn) for the incoming socket
                self.conn.setblocking(False)         # Make the connection non blocking
                print('Incoming socket from address:', str(addr[0]))  # Print IP-address of Client
                self.state = 2                       # state 2: Connection with a Client
                if self.led is not None:
                   self.led.value(True)              # Turn on the led to show there is a connection with a Client
            except:
                pass
        else:                                        # Connection with network is lost
            self.connectToNetwork()                  # Try to restore the network connection
                
    # The method transmit(s) transmits string argument tx via the socket if a connection is present.
    def transmit(self, tx):
        if self.state == 2:                          # Check if state is 2 
            try:
                self.conn.sendall(tx)
                return
            except:
                pass
        self.resetSocket()                           # Reset the socket settings
                
    # The method receive() returns a bytes variable that contains all characters received from
    # the socket since the last read or None if nothing is received.
    def receive(self):
        if self.state == 2:                          # Check if state is 2
            try:
                rx = self.conn.recv(256)             # Read at most 256 characters from the socket
                if b'_q&' in rx:
                    print('Closing connection ...')
                    self.resetSocket()
                    return None
                return rx
            except:
                return None
            self.resetSocket()                           # Reset the socket settings
        return None

    # The method getState() checks if the network state is changed. It returns one of the following values:
    # 0:	The ESP32 is not connected to the network
    # 1:	The ESP32 is connected to the network but not connected to a client
    # 2:	The ESP32 is connected to a client. 
    def getState(self):
        if self.state == 0:
            self.checkNetworkConnection()            # Device is not yet connected to network
        elif self.state == 1:
            self.listenToClients()                   # Device is not yet connected to a client
        return self.state
    
    # The method resetSocket() resets the socket.It should be called when the client connection is lost.
    def resetSocket(self):
        if self.led is not None:
            self.led.value(False)                # Turn led off
        print('Client connection lost')          # Print error message
        self.state = 1                           # Go back to state = 1
        self.listenToClients()                   # Device is not yet connected to a client
