import serial
import threading
import time
import queue
from typing import Optional, Callable
import serial.tools.list_ports


class SerialConsole:
    def __init__(self, rx_callback: Optional[Callable[[str], None]] = None):
        self.serial_port: Optional[serial.Serial] = None
        self.rx_callback = rx_callback
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.rx_queue = queue.Queue(maxsize=10000)
        self.connected = False
        self.port = ""
        self.baudrate = 115200
        self.printing_raw = True
        
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        try:
            self.port = port
            self.baudrate = baudrate
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(0.5)
            
            self.running = True
            self.connected = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            print(f"Connected to {port} at {baudrate} baud")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
            
    def disconnect(self):
        self.running = False
        self.connected = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
            
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            
        print("Disconnected")
        
    def send_command(self, command: str) -> bool:
        if not self.connected or not self.serial_port:
            return False
            
        try:
            if not command.endswith('\r'):
                command += '\r'
            print("SENDING", command)
            self.serial_port.write(command.encode())
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
            
    def send_raw(self, data: bytes) -> bool:
        if not self.connected or not self.serial_port:
            return False
            
        try:
            self.serial_port.write(data)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
            
    def _rx_loop(self):
        buffer = b""
        
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    buffer += self.serial_port.read(self.serial_port.in_waiting)
                    
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line_str = line.decode('utf-8', errors='ignore').strip()
                        
                        if line_str:
                            # self.rx_queue.put(line_str) # TODO: better way to polling, but not implemented.
                            if self.rx_callback:
                                self.rx_callback(line_str)
                                
                else:
                    time.sleep(0.001)
                    
            except Exception as e:
                if self.running:
                    print(f"RX error: {e}")
                    time.sleep(0.1)
                    
    def get_line(self, timeout: float = 0.1) -> Optional[str]:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None
            
    def get_all_lines(self) -> list[str]:
        lines = []
        while True:
            line = self.get_line(timeout=0.001)
            if line is None:
                break
            lines.append(line)
        return lines
        
    def clear_rx_queue(self):
        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break
                
    def is_connected(self) -> bool:
        return self.connected and self.serial_port is not None
        
    def get_queue_size(self) -> int:
        return self.rx_queue.qsize()


class MotionConsole(SerialConsole):
    def __init__(self, rx_callback: Optional[Callable[[str], None]] = None):
        super().__init__(rx_callback)
        
    def enable_raw_output(self):
        return self.send_command("printraw 1")
        
    def toggle_raw_output(self):
        if self.printing_raw:
            self.printing_raw = False
            return self.send_command("printraw 0")
        self.printing_raw = True
        return self.send_command("printraw 1")
    def set_thresholds(self, altitude: int, azimuth: int):
        res_th = self.send_command(f"alth {altitude} {azimuth}")
        res_ti = self.send_command(f"ttc 240") # 4 hours
        res_ca = self.send_command(f"alcal")
        return (res_th and res_ti and res_ca)
        
    def calibrate(self, alt_threshold: float = 5.0, az_threshold: float = 10.0):
        altitude = (int)(alt_threshold * 10)
        azimuth = (int)(az_threshold * 10)
        return self.set_thresholds(altitude, azimuth)
    def reset_device(self):
        return self.send_command("RESET")
        
    def request_status(self):
        return self.send_command("STATUS")

    def get_available_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]


if __name__ == "__main__":
    import sys
    
    def print_line(line: str):
        print(f"RX: {line}")
        
    console = MotionConsole(rx_callback=print_line)
    
    port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
    
    if console.connect(port):
        print("Commands: 'raw', 'thresh <alt> <az>', 'cal', 'reset', 'status', 'quit'")
        
        console.enable_raw_output()
        
        try:
            while True:
                cmd = input("> ").strip()
                
                if cmd == "quit":
                    break
                elif cmd == "raw":
                    console.enable_raw_output()
                elif cmd.startswith("thresh"):
                    parts = cmd.split()
                    if len(parts) == 3:
                        console.set_thresholds(int(parts[1]), int(parts[2]))
                elif cmd == "cal":
                    console.calibrate()
                elif cmd == "reset":
                    console.reset_device()
                elif cmd == "status":
                    console.request_status()
                elif cmd:
                    console.send_command(cmd)
                    
        except KeyboardInterrupt:
            print("\nExiting...")
            
        console.disconnect()
    
    sys.exit()