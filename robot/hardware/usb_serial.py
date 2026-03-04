import serial
import time

class USBSerial:
    def __init__(self, port_name: str, baudrate=115200, timeout=1): # baud rate TBD
        self.ser = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = port_name
        self.ser = serial.Serial
        
    def connect(self):
        """opens serial port + handshake"""
        self.ser = serial.Serial(
                timeout=self.timeout, 
                baudrate=self.baudrate,
                port=self.port
            )
        
    def write(self, data):
        if self.ser and self.ser.is_open:
            self.ser.write(data.encode())
        else:
            print("Serial connection not established.")

    def read(self):
        if self.ser and self.ser.is_open:
            return self.ser.readline().decode().strip()
        else:
            print("Serial connection not established.")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()