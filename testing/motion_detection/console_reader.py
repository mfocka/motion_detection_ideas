"""
Console Reader for Motion Detection Live Data

This module provides a simple interface to read live data from a serial console
for motion detection visualization and debugging.
"""

import serial
import time
import threading
from queue import Queue, Empty

class MotionConsole:
    """Serial console reader for motion detection data"""
    
    def __init__(self, port, baudrate=115200, timeout=1.0):
        """
        Initialize the console reader
        
        Args:
            port (str): Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate (int): Baud rate for serial communication
            timeout (float): Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self.line_queue = Queue()
        self.reader_thread = None
        self.running = False
        
        # Connect to serial port
        self._connect()
        
    def _connect(self):
        """Connect to the serial port"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.is_connected = True
            print(f"Connected to {self.port} at {self.baudrate} baud")
            
            # Start reader thread
            self.running = True
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.is_connected = False
            raise
    
    def _reader_loop(self):
        """Background thread that continuously reads from serial port"""
        buffer = ""
        
        while self.running and self.is_connected:
            try:
                # Read available data
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.line_queue.put(line)
                else:
                    time.sleep(0.001)  # Small delay to prevent busy waiting
                    
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                break
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
                buffer = ""  # Reset buffer on decode error
                continue
    
    def read_line(self, timeout=None):
        """
        Read a line from the console
        
        Args:
            timeout (float): Timeout in seconds (None for blocking read)
            
        Returns:
            str: Line read from console, or None if timeout
        """
        if not self.is_connected:
            return None
            
        try:
            return self.line_queue.get(timeout=timeout)
        except Empty:
            return None
    
    def write_line(self, data):
        """
        Write a line to the console
        
        Args:
            data (str): Data to write
        """
        if not self.is_connected:
            return False
            
        try:
            self.serial_conn.write((data + '\n').encode('utf-8'))
            return True
        except serial.SerialException as e:
            print(f"Serial write error: {e}")
            return False
    
    def flush_input(self):
        """Flush the input buffer"""
        if self.is_connected:
            self.serial_conn.reset_input_buffer()
            # Also clear our internal queue
            while not self.line_queue.empty():
                try:
                    self.line_queue.get_nowait()
                except Empty:
                    break
    
    def close(self):
        """Close the serial connection"""
        self.running = False
        
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"Disconnected from {self.port}")
        
        self.is_connected = False
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()

# Test functionality
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python console_reader.py <port>")
        print("Example: python console_reader.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        with MotionConsole(port) as console:
            print(f"Reading from {port}. Press Ctrl+C to stop.")
            while True:
                line = console.read_line(timeout=1.0)
                if line:
                    print(f"Received: {line}")
                    
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")