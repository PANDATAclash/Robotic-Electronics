import array

"""
The array reg is a 32-bit intebger array and is used for data exchange with a high level controller.
All 64 elements are initialized to 0.
"""
reg = array.array('i', [0] * 64)			# reg is an array of 64 integers (all initialized to 0)


class HighLevelController:
    
    """
    The class HighLevelController provides functionality to communicate with a high level controller via serial communidation and/or TCP.
    The methods readFromUart() and readFromTCP() should be called to read and process incoming data
    """


    """
    The constructor of the HighLevelController class has 2 arguments:
    uart: Object of the UART class used for the serial port communication.
    tcp:  Object of the TCPHandler class used for communication via a TCP connection.
    """
    def __init__(self, uart, tcp):				# Constructor
        self.uart = uart
        self.tcp = tcp
        self.uart_asd = AsciiStreamDecoder()	# object that processes the incoming stream from the uart
        self.uart_buf = bytearray(64)			# buffer to store incoming data from uart
        self.tcp_asd = AsciiStreamDecoder()		# object that processes the incoming stream from TCP
    
    """
    The method readFromUart() reads incoming characters from the uart object. If a valid read request has been received, it calls the method
    transmitReadResponse() that transmits the requested data via the uart object.
    If a valid write command has been received, the function stores the data received in array reg at the given address(es).
    """
    def readFromUart(self):
        n = self.uart.readinto(self.uart_buf)	# Read data from the uart
        if n is None:
            return								# No data received -> return
        commands = self.uart_asd.processInput(self.uart_buf[0:n])	# Process incoming data from the UART
        if commands is not None:				# Check if a complete read request or write command is received
            self.processHighLevelCommands(commands, self.uart)

    """
    The method readFromTCP() reads incoming characters from the tcp object. If a valid read request has been received, it calls the method
    transmitReadResponse() that transmits the requested data via the tcp object.
    If a valid write command has been received, the function stores the data received in array reg at the given address(es).
    """
    def readFromTCP(self):
        rx = self.tcp.receive()					# Read data from TCP
        if rx is None:
            return								# No data received -> return
        commands = self.tcp_asd.processInput(rx)	# Process incoming data from TCP
        if commands is not None:
            self.processHighLevelCommands(commands, self.tcp)

    """
    The method processHighLevelCommands() processes all read requests and write commands
    Argument commands is a list of commands. Each element of the list commands is again a list of 3 elements:
    The 1st element is a booland that is True in case of read request and False in case of a write command
    The 2nd element is the address of the read request or write command
    The 3rd element is the number of data in case of a read request and it is the value of the data in case of a write command
    Argument src is the source to which a read response is sent. It can be a UART or TCP object
    """
    def processHighLevelCommands(self, commands, src):	# src is source of read request (uart or tcp)
        for c in commands:
            if c[0]:							# c[0] is a boolean that is True for a read request
                self.transmitReadResponse(c[1],c[2],src)	# c[1] is the address, c[2] is the number of data to be transmitted
            else:								# c[0] = False  -->  Write command
                address = c[1]
                if address >= 9:				# First 9 addresses are read only (sensor data)
                    reg[address] = c[2]	# c[2] is the data

    """
    The method transmitReadResponse() transmits a read response
    Argument address is the address of the first variable of the read resonse
    Argument count is the number of variables of the read response
    Argument src is the source to which a read response is sent. It can be a UART or TCP object
    """
    def transmitReadResponse(self, address, count, src):
        endaddress = min(64, address+count)		# Get last address of read response (<64)
        tx = '(' + str(address)
        for a in range(address, endaddress):
            tx = tx + ',' + str(reg[a])
        tx = tx + ')'
        if src == self.uart:
            self.uart.write(tx.encode())  # Convert string to ASCII characters and transmit it via the uart
        else:
            self.tcp.transmit(tx)


class AsciiStreamDecoder:
    
    """
    The class AsciiSreamDecoder is used to decode a stream of ASCII characters that holds read requests and/or
    or write commands. A state machine is used for the decoding. The statemachine uses 5 attributes:
    state:   Integer that defines the state of the state machine
    address: Integer that holds the address of a read request or write command
    arg2:    Bytearray of at most 16 bytes that holds the 2nd argument of a read request or write command
    i2:      Number of bytes of argument 2 that have been received 
    read:    Boolean that defines whether a received string is a read request or write command
    The actions and exit conditions of each state are defined as follows:
    All states:
      Action: if '(' or '[' is received -> read = True if a '(' is received
                                        -> state = 1
    state 1:
      Action: if a valid digit is received  -> address = digit
      Exit condition to state 2: a valid digit is received
      Exit condition to state 0: no valid digit is received
    state 2:
      Action: if a valid digit is received -> Update address
              if a comma is received -> i2 = 1
      Exit condition to state 3: A comma is received
      Exit condition to state 0: No digit or comma is received or address >= 64 
    state 3:
      Action: Store received byte in arg2[0]
      Exit condition to state 4: True (always)
    state 4:  
      Action: if ')' or ']' is received -> Process read request or write command
              if no ')' and no ']' -> Append character to arg2[i2], i2 = i2 + 1
      Exit condition to state 0: An opening or closing bracket is received or i2 = 16
    """

    def __init__(self):							# Constructor of AsciiSreamDecoder
        self.state = 0							# Initial state of statemachine
        self.arg2 = bytearray(11)				# Define bytearray to store argument 2 of read- or writerequest

    """
    The method processInput(buf) processes a buffer of ASCII characters using a statemachine described above
    It returns a list of completed read requests or write commands
    A read request is returned as: [True, address, arg2]
    A write command is returned as: [False, address, arg2]
    An empty list is returned if buf doesn't hold at least one valid completed read request or write command
    """
    
    def processInput(self, buf):
        result = []								# List that holds the completed read- or write requests
        for c in buf:							# Walk through all characters in buf
            if c == 40 or c == 91:				# Ascii code of '(' is 40, Ascii code of '[' is 91 
                self.read = c == 40				# Character is '(' -> read request
                self.state = 1					# Goto state 1
            elif self.state == 1:				# The first digit of the address is expected
                x = c - 48						# x = digit value of digit (The ASCII offset of a digit is 48)
                if 0 <= x < 10:					# Valid digit
                    self.address = x
                    self.state = 2				# Goto state 2
                else:
                    self.state = 0				# No valid digit in address argument --> Go back to state 0
            elif self.state == 2:				# The second digit of the address is expected or a comma
                if c == 44:						# Ascii code of ',' is 44
                    self.state = 3				# Goto state 3
                else:
                    x = c - 48					# Get decimal value of ASCII character
                    if 0 <= x < 10:				# Valid digit
                        self.address = 10 * self.address + x
                        if self.address > 63:	# Check if address is not higher than 63
                            self.state = 0     
                    else:
                        self.state = 0			# No valid digit and no ',' --> Go back to state 0
            elif self.state == 3:				# The first digit of the second argument is expected
                self.arg2[0] = c				# Store character in arg2
                self.i2 = 1						# Number of characters stored in arg2
                self.state = 4					# Goto state 4
            elif self.state == 4:				# # The next digit of argument 2 is expected or a ')' or a ']'
                if c == 41 or c == 93:			# Ascii code of ')' is 41, Ascii code of ']' is 93 
                    if self.read == (c == 41):	# Check if opening bracket is same type as closing bracket
                        try:
                            result.append([self.read, self.address, int(self.arg2[0:self.i2])])	# Add request to the result list
                        except:
                            pass				# Invalid arg2 -> do nothing
                    self.state = 0				# End read request -> Ga back to state 0
                else:
                    if self.i2 >= 11:			# Maximum number of a 32-bits integer is 11
                        self.state = 0			# Too many characters in argument 2
                    else:    
                        self.arg2[self.i2] = c	# Store character in arg2
                        self.i2 += 1			# Increase i2
        return result