import csv
import time
import sys
import os
from datetime import datetime
from typing import Optional, List, Dict
import argparse
from console_reader import MotionConsole

# Resulting csv's should be:
#         'angles': "motion_data_*.csv",
#         'quaternions': "*_quaternions.csv", 
#         'raw_data': "*_raw_data.csv",
#         'gyro_bias': "*_gyro_bias.csv",


headers = {
    "MDI": "TESTING,MDI,TIME,MDI_Q_W,MDI_Q_X,MDI_Q_Y,MDI_Q_Z",
    "MFX": "TESTING,MFX,TIME,MFX_Q_W,MFX_Q_X,MFX_Q_Y,MFX_Q_Z",
    "RAL": "TESTING,RAL,TIME,REF_ALT_W,REF_ALT_X,REF_ALT_Y,REF_ALT_Z",
    "RAZ": "TESTING,RAZ,TIME,REF_AZI_W,REF_AZI_X,REF_AZI_Y,REF_AZI_Z",
    "RES": "TESTING,RES,TIME,MFX_ALT,MDI_ALT,MFX_AZ,MDI_AZ"
}

class MotionDataCapture:
    def __init__(self, output_file: str = None):
        self.console = MotionConsole(rx_callback=self._process_line)

        self.log_file = None
        
        # Main CSV files and writers
        self.csv_writer = None
        self.csv_file = None
        self.quat_csv_writer = None
        self.quat_csv_file = None
        self.raw_csv_writer = None
        self.euler_csv_writer = None
        self.raw_csv_file = None
        self.euler_csv_file = None
        self.gyro_bias_csv_writer = None
        self.gyro_bias_csv_file = None
        self.temp_csv_writer = None
        self.temp_csv_file = None
        
        # Data buffers
        self.data_buffer = []
        self.quaternion_buffer = []
        self.raw_data_buffer = []
        self.euler_data_buffer = []
        self.gyro_bias_buffer = []
        self.temperature_buffer = []
        
        # Counters and state
        self.header_written = False
        self.prev_header = ""
        self.sample_count = 0
        self.quat_sample_count = 0
        self.raw_sample_count = 0
        self.euler_sample_count = 0
        self.gyro_bias_sample_count = 0
        self.temp_sample_count = 0
        self.start_time = None

        self.can_start = False
        
        # File naming
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"motion_data_{timestamp}"
        else:
            base_name = output_file.rsplit('.', 1)[0]

        self.log_output_file =  f"{base_name}_log.txt"

        self.output_file = f"{base_name}_angles.csv"
        self.quat_output_file = f"{base_name}_quaternions.csv"
        self.raw_output_file = f"{base_name}_raw_data.csv"
        self.euler_output_file = f"{base_name}_euler_data.csv"
        self.gyro_bias_output_file = f"{base_name}_gyro_bias.csv"
        self.temp_output_file = f"{base_name}_temperature.csv"

    def _process_line(self, line_full: str):
        try:
            if self.log_file is None:
                self.log_file = open(self.log_output_file,'a', encoding='utf-8')
            self.log_file.write(line_full)
            if not line_full.endswith('\n'):
                self.log_file.write('\n')
            self.log_file.flush()
            line_full = line_full.rstrip('\n\r')
            line_full = line_full.replace('"', '')
            lines = line_full.split("] ")
            if(len(lines) < 2):
                return
            else:
                line = lines[1]
            if self.can_start: 
                if line.startswith("TESTING,"):
                    self._parse_testing_data(line)
                elif line.startswith("ANGLES,"):
                    self._parse_angle_data(line)
                elif line.startswith("EULER_DIAG,"):
                    self._parse_euler_diag(line)
                elif line.startswith("QUAT"):
                    self._parse_quaternion_data(line)
                elif any(keyword in line for keyword in ["EULER_MFX_DIRECT", "EULER_FROM_QUAT"]):
                    self._parse_euler_angles(line)
                elif line.startswith("RAW_DATA,"):
                    self._parse_raw_data(line)
                elif line.startswith("GYRO_BIAS_MDI,") or line.startswith("GYRO_BIAS_MFX,"):
                    self._parse_gyro_bias(line)
                elif line.startswith("TEMPERATURE,"):
                    self._parse_temperature(line)
                # elif any(keyword in line for keyword in ["CALIBRATION_COMPLETE", "EVENT", "REFERENCE"]):
                elif any(keyword in line for keyword in ["CALIBRATION_COMPLETE", "REFERENCE"]):
                    print(f"[INFO] {line}")

                else:
                    self._parse_raw_data(line)
                # else:
                #     print(f"[DEBUG] Unhandled line: {line}")
            else:
                if any(keyword in line for keyword in ["REFERENCE", "ANGLES"]):
                    print(f"[INFO] {line}")
                    self.can_start = True
        except Exception as e:
            print(f"Error processing line: {e}")
    def _parse_euler_angles(self, line):
        """
        Parse EULER_MFX_DIRECT,57420780,331.967,-15.254,-17.505
        or    EULER_FROM_QUAT,57420780,-161.896,14.531,-156.656
        """
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                diag_type = parts[0].replace("EULER_", "")
                timestamp = int(parts[1])
                roll = float(parts[2])
                pitch = float(parts[3])
                yaw = float(parts[4])
                
                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                
                euler_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'diag_type': diag_type,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw
                }
                
                self.euler_data_buffer.append(euler_data)
                self.euler_sample_count += 1
                self._write_euler_data_to_csv(euler_data)
                # print(f"[EULER] {diag_type}: Roll={roll:.3f}°, Pitch={pitch:.3f}°, Yaw={yaw:.3f}°")
        except (ValueError, IndexError) as e:
            print(f"Error parsing euler angles: {e} - Line: {line}")
            return
    def _parse_raw_data(self, line: str):
        """Parse RAW_DATA,timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z"""
        try:
            parts = line.split(',')
            if 5 < len(parts) < 8:
                timestamp = time.monotonic()
                accel_x = float(parts[0])
                accel_y = float(parts[1])
                accel_z = float(parts[2])
                gyro_x = float(parts[3])
                gyro_y = float(parts[4])
                gyro_z = float(parts[5])

                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                
                raw_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'accel_x_mg': accel_x,
                    'accel_y_mg': accel_y,
                    'accel_z_mg': accel_z,
                    'gyro_x_dps': gyro_x,
                    'gyro_y_dps': gyro_y,
                    'gyro_z_dps': gyro_z,
                    'accel_magnitude': (accel_x**2 + accel_y**2 + accel_z**2)**0.5,
                    'gyro_magnitude': (gyro_x**2 + gyro_y**2 + gyro_z**2)**0.5
                }
                
                self.raw_data_buffer.append(raw_data)
                self.raw_sample_count += 1
                self._write_raw_data_to_csv(raw_data)
                
                if self.raw_sample_count % 104000 == 0:
                    print(f"Captured {self.raw_sample_count} raw samples... "
                          f"Latest: A({accel_x:.1f},{accel_y:.1f},{accel_z:.1f}) "
                          f"G({gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f})")
            elif len(parts) >= 8:
                timestamp = int(parts[1])
                accel_x = float(parts[2])
                accel_y = float(parts[3])
                accel_z = float(parts[4])
                gyro_x = float(parts[5])
                gyro_y = float(parts[6])
                gyro_z = float(parts[7])
                
                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                
                raw_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'accel_x_mg': accel_x,
                    'accel_y_mg': accel_y,
                    'accel_z_mg': accel_z,
                    'gyro_x_dps': gyro_x,
                    'gyro_y_dps': gyro_y,
                    'gyro_z_dps': gyro_z,
                    'accel_magnitude': (accel_x**2 + accel_y**2 + accel_z**2)**0.5,
                    'gyro_magnitude': (gyro_x**2 + gyro_y**2 + gyro_z**2)**0.5
                }
                
                self.raw_data_buffer.append(raw_data)
                self.raw_sample_count += 1
                self._write_raw_data_to_csv(raw_data)
                
                if self.raw_sample_count % 104000 == 0:
                    print(f"Captured {self.raw_sample_count} raw samples... "
                          f"Latest: A({accel_x:.1f},{accel_y:.1f},{accel_z:.1f}) "
                          f"G({gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f})")
        except (ValueError, IndexError) as e:
            print(f"Error parsing raw data: {e} - Line: {line}")

    def _parse_gyro_bias(self, line: str):
        """Parse GYRO_BIAS_MDI,timestamp,x,y,z or GYRO_BIAS_MFX,timestamp,x,y,z"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:  # At minimum: type, timestamp, x, y, z
                bias_type = parts[0].replace("GYRO_BIAS_", "")  # Extract MDI or MFX
                timestamp = int(parts[1])
                
                # Handle variable number of bias values (in case of inconsistent format)
                bias_values = [float(p) for p in parts[2:]]
                
                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                
                gyro_bias_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'bias_type': bias_type,
                    'bias_x': bias_values[0] if len(bias_values) > 0 else 0.0,
                    'bias_y': bias_values[1] if len(bias_values) > 1 else 0.0,
                    'bias_z': bias_values[2] if len(bias_values) > 2 else 0.0,
                }
                
                # Add additional values if present
                if len(bias_values) > 3:
                    gyro_bias_data['bias_extra'] = bias_values[3]
                else:
                    gyro_bias_data['bias_extra'] = 0.0
                    
                gyro_bias_data['bias_magnitude'] = (
                    gyro_bias_data['bias_x']**2 + 
                    gyro_bias_data['bias_y']**2 + 
                    gyro_bias_data['bias_z']**2
                )**0.5
                
                self.gyro_bias_buffer.append(gyro_bias_data)
                self.gyro_bias_sample_count += 1
                self._write_gyro_bias_to_csv(gyro_bias_data)
                      
        except (ValueError, IndexError) as e:
            print(f"Error parsing gyro bias: {e} - Line: {line}")

    def _parse_temperature(self, line: str):
        """Parse TEMPERATURE,timestamp,temperature_celsius"""
        try:
            parts = line.split(',')
            if len(parts) >= 3:
                timestamp = int(parts[1])
                temperature = float(parts[2])
                
                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                
                temp_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'temperature_celsius': temperature,
                    'temperature_kelvin': temperature + 273.15,
                    'temperature_fahrenheit': temperature * 9/5 + 32
                }
                
                self.temperature_buffer.append(temp_data)
                self.temp_sample_count += 1
                self._write_temperature_to_csv(temp_data)
                
                # if self.temp_sample_count % 20 == 0:
                #     print(f"Temperature samples: {self.temp_sample_count}, Latest: {temperature:.2f}°C")
                    
        except (ValueError, IndexError) as e:
            print(f"Error parsing temperature: {e} - Line: {line}")

    def _write_raw_data_to_csv(self, raw_data: Dict):
        if self.raw_csv_file is None:
            self.raw_csv_file = open(self.raw_output_file, 'w', newline='')
            fieldnames = ['timestamp_us', 'relative_time_ms', 'accel_x_mg', 'accel_y_mg', 'accel_z_mg',
                         'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps', 'accel_magnitude', 'gyro_magnitude']
            self.raw_csv_writer = csv.DictWriter(self.raw_csv_file, fieldnames=fieldnames)
            self.raw_csv_writer.writeheader()
            print(f"Created raw data CSV file: {self.raw_output_file}")
        self.raw_csv_writer.writerow(raw_data)
        self.raw_csv_file.flush()

    def _write_euler_data_to_csv(self, euler_data: Dict):
        if self.euler_csv_file is None:
            self.euler_csv_file = open(self.euler_output_file, 'w', newline='')
            fieldnames = euler_data.keys()  # Use keys from the first euler_data dict
            self.euler_csv_writer = csv.DictWriter(self.euler_csv_file, fieldnames=fieldnames)
            self.euler_csv_writer.writeheader()
            print(f"Created euler data CSV file: {self.euler_output_file}")
        self.euler_csv_writer.writerow(euler_data)
        self.euler_csv_file.flush()
    def _write_gyro_bias_to_csv(self, gyro_bias_data: Dict):
        if self.gyro_bias_csv_file is None:
            self.gyro_bias_csv_file = open(self.gyro_bias_output_file, 'w', newline='')
            fieldnames = ['timestamp_us', 'relative_time_ms', 'bias_type', 'bias_x', 'bias_y', 'bias_z', 
                         'bias_extra', 'bias_magnitude']
            self.gyro_bias_csv_writer = csv.DictWriter(self.gyro_bias_csv_file, fieldnames=fieldnames)
            self.gyro_bias_csv_writer.writeheader()
            print(f"Created gyro bias CSV file: {self.gyro_bias_output_file}")
        self.gyro_bias_csv_writer.writerow(gyro_bias_data)
        self.gyro_bias_csv_file.flush()

    def _write_temperature_to_csv(self, temp_data: Dict):
        if self.temp_csv_file is None:
            self.temp_csv_file = open(self.temp_output_file, 'w', newline='')
            fieldnames = ['timestamp_us', 'relative_time_ms', 'temperature_celsius', 
                         'temperature_kelvin', 'temperature_fahrenheit']
            self.temp_csv_writer = csv.DictWriter(self.temp_csv_file, fieldnames=fieldnames)
            self.temp_csv_writer.writeheader()
            print(f"Created temperature CSV file: {self.temp_output_file}")
        self.temp_csv_writer.writerow(temp_data)
        self.temp_csv_file.flush()

    def _parse_euler_diag(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) >= 6:
                diag_type = parts[1]
                timestamp = int(parts[2])
                roll = float(parts[3])
                pitch = float(parts[4])
                yaw = float(parts[5])
                # print(f"[EULER] {diag_type}: Roll={roll:.3f}°, Pitch={pitch:.3f}°, Yaw={yaw:.3f}°")
        except (ValueError, IndexError) as e:
            print(f"Error parsing Euler diag: {e} - Line: {line}")

    def _parse_testing_data(self, line: str):
        try:
            parts = line.split(',')
            type_part = parts[1]
            self.prev_header = headers[type_part]
            if len(parts) < 3:
                return
            if len(parts) >= 6 and type_part in ["MDI", "MFX", "RAL", "RAZ"]:
                self._parse_quaternion_data(line)
            elif len(parts) >= 6 and type_part == "RES":
                self._parse_angle_data(line)
        except (ValueError, IndexError) as e:
            print(f"Error parsing testing data: {e} - Line: {line}")

    def _parse_quaternion_data(self, line: str):
        try:
            parts = line.split(',')
            h_parts = self.prev_header.split(',')
            
            if len(parts) >= 6:
                if parts[0] == "QUAT":
                    data_type = "MFX"
                    timestamp = int(parts[1])
                    w = float(parts[-4])
                    x = float(parts[-3])
                    y = float(parts[-2])
                    z = float(parts[-1])
                else:
                    data_type = h_parts[1]  # This gives us MDI, MFX, RAL, or RAZ
                    timestamp = int(parts[2])  # parts[1] is the type, parts[2] is timestamp
                    w = float(parts[3])
                    x = float(parts[4])
                    y = float(parts[5])
                    z = float(parts[6])
                if self.start_time is None:
                    self.start_time = timestamp
                relative_time_ms = (timestamp - self.start_time) / 1000.0
                quat_data = {
                    'timestamp_us': timestamp,
                    'relative_time_ms': relative_time_ms,
                    'type': data_type,  # MDI, MFX, RAL, RAZ
                    'w': w, 'x': x, 'y': y, 'z': z,
                    'magnitude': (w*w + x*x + y*y + z*z)**0.5
                }
                if not hasattr(self, 'quaternion_buffer'):
                    self.quaternion_buffer = []
                self.quaternion_buffer.append(quat_data)
                self._write_quaternion_to_csv(quat_data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing quaternion data: {e} - Line: {line}")

    def _parse_angle_data(self, line: str):
        try:
            parts = line.split(',')
            if len(parts) >= 6:
                if parts[0] == "ANGLES":
                    timestamp = int(parts[1])  # parts[2] is timestamp for RES type
                    alt = float(parts[2])
                    az = float(parts[3])
                    zen = float(parts[4])
                    state = parts[5]
                    if self.start_time is None:
                        self.start_time = timestamp
                    relative_time_ms = (timestamp - self.start_time) / 1000.0
                    data_point = {
                        'timestamp_us': timestamp,
                        'relative_time_ms': relative_time_ms,
                        'altitude': alt,
                        'azimuth': az,
                        'zenith': zen,
                    }
                    self.data_buffer.append(data_point)
                    self.sample_count += 1
                    self._write_to_csv(data_point)
                    # if self.sample_count % 1040 == 0:
                    #     print(f"Captured {self.sample_count} samples... "
                    #         f"Latest: ALT:{alt:.2f}, AZ:{az:.2f})")
                elif parts[0] == "TESTING":
                    # For RES type: TESTING,RES,TIME,MFX_ALT,MDI_ALT,MFX_AZ,MDI_AZ
                    timestamp = int(parts[2])  # parts[2] is timestamp for RES type
                    mfx_alt = float(parts[3])
                    mdi_alt = float(parts[4])
                    mfx_az = float(parts[5])
                    mdi_az = float(parts[6])
                    if self.start_time is None:
                        self.start_time = timestamp
                    relative_time_ms = (timestamp - self.start_time) / 1000.0
                    data_point = {
                        'timestamp_us': timestamp,
                        'relative_time_ms': relative_time_ms,
                        'mfx_altitude': mfx_alt,
                        'mdi_altitude': mdi_alt,
                        'mfx_azimuth': mfx_az,
                        'mdi_azimuth': mdi_az,
                        'altitude_diff': abs(mfx_alt - mdi_alt),
                        'azimuth_diff': abs(mfx_az - mdi_az)
                    }
                    self.data_buffer.append(data_point)
                    self.sample_count += 1
                    self._write_to_csv(data_point)
                    if self.sample_count % 1040 == 0:
                        print(f"Captured {self.sample_count} samples... "
                            f"Latest: ALT(MFX:{mfx_alt:.2f}, MDI:{mdi_alt:.2f}) "
                            f"AZ(MFX:{mfx_az:.2f}, MDI:{mdi_az:.2f})")
        except (ValueError, IndexError) as e:
            print(f"Error parsing angle data: {e} - Line: {line}")

    def _write_to_csv(self, data_point: Dict):
        if self.csv_file is None:
            self.csv_file = open(self.output_file, 'w', newline='')

            # fieldnames = ['timestamp_us', 'relative_time_ms', 'mfx_altitude', 'mdi_altitude',
            #              'mfx_azimuth', 'mdi_azimuth', 'altitude_diff', 'azimuth_diff']
            fieldnames = data_point.keys()
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            print(f"Created CSV file: {self.output_file}")
        self.csv_writer.writerow(data_point)
        self.csv_file.flush() # Should write faster

    def _write_quaternion_to_csv(self, quat_data: Dict):
        if self.quat_csv_file is None:
            self.quat_csv_file = open(self.quat_output_file, 'w', newline='')
            fieldnames = ['timestamp_us', 'relative_time_ms', 'type', 'w', 'x', 'y', 'z', 'magnitude']
            self.quat_csv_writer = csv.DictWriter(self.quat_csv_file, fieldnames=fieldnames)
            self.quat_csv_writer.writeheader()
            print(f"Created quaternion CSV file: {self.quat_output_file}")
        self.quat_csv_writer.writerow(quat_data)
        self.quat_csv_file.flush()
        self.quat_sample_count += 1

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        if self.console.connect(port, baudrate):
            print(f"Connected to {port}")
            self.console.calibrate(10, 20)
            time.sleep(2)
            return True
        return False

    def process_file(self, file_path: str):
        """Reads data from a text file and processes each line."""
        print(f"Processing data from file: {file_path}")
        try:
            with open(file_path, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    self._process_line(line)
            print(f"Finished processing file: {file_path}")
        except FileNotFoundError:
            print(f"Error: File '{file_path}' not found.")
        except Exception as e:
            print(f"An error occurred while processing the file: {e}")
        finally:
            self._finalize_capture()

    def start_capture(self, duration_seconds: Optional[int] = None):
        if duration_seconds:
            print(f"Will capture for {duration_seconds} seconds")
        start_capture_time = time.time()
        try:
            while True:
                if duration_seconds and (time.time() - start_capture_time) > duration_seconds:
                    print(f"Capture duration of {duration_seconds} seconds reached")
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nCapture interrupted by user")
        self._finalize_capture()

    def _finalize_capture(self):
        # Close all files and print summaries
        files_created = []

        if self.log_file:
            self.log_file.close()
            files_created.append(f"Raw log: {self.log_output_file}")
        
        if self.csv_file:
            self.csv_file.close()
            files_created.append(f"Angle data: {self.output_file} ({self.sample_count} samples)")
            
        if self.quat_csv_file:
            self.quat_csv_file.close()
            files_created.append(f"Quaternion data: {self.quat_output_file} ({self.quat_sample_count} samples)")
            
        if self.raw_csv_file:
            self.raw_csv_file.close()
            files_created.append(f"Raw sensor data: {self.raw_output_file} ({self.raw_sample_count} samples)")
            
        if self.euler_csv_file:
            self.euler_csv_file.close()
            files_created.append(f"Euler sensor data: {self.euler_output_file} ({self.euler_sample_count} samples)")
        if self.gyro_bias_csv_file:
            self.gyro_bias_csv_file.close()
            files_created.append(f"Gyro bias data: {self.gyro_bias_output_file} ({self.gyro_bias_sample_count} samples)")
            
        if self.temp_csv_file:
            self.temp_csv_file.close()
            files_created.append(f"Temperature data: {self.temp_output_file} ({self.temp_sample_count} samples)")
        
        print("\n" + "="*60)
        print("DATA CAPTURE COMPLETE")
        print("="*60)
        
        if files_created:
            print("Files created:")
            for file_info in files_created:
                print(f"  • {file_info}")
        else:
            print("No data files created (no data captured)")
            
        # Print detailed summaries
        if self.data_buffer:
            self._print_summary()
        if hasattr(self, 'quaternion_buffer') and self.quaternion_buffer:
            self._print_quaternion_summary()
        if self.raw_data_buffer:
            self._print_raw_data_summary()
        if self.gyro_bias_buffer:
            self._print_gyro_bias_summary()
        if self.temperature_buffer:
            self._print_temperature_summary()
            
        # Only disconnect console if it was actually connected (relevant for serial mode)
        if hasattr(self.console, 'serial') and self.console.serial and self.console.serial.is_open:
             self.console.disconnect()

    def _print_raw_data_summary(self):
        print("\nRAW SENSOR DATA SUMMARY")
        print("-" * 30)
        if not self.raw_data_buffer:
            print("No raw data captured.")
            return
            
        accel_x_vals = [d['accel_x_mg'] for d in self.raw_data_buffer]
        accel_y_vals = [d['accel_y_mg'] for d in self.raw_data_buffer]
        accel_z_vals = [d['accel_z_mg'] for d in self.raw_data_buffer]
        gyro_x_vals = [d['gyro_x_dps'] for d in self.raw_data_buffer]
        gyro_y_vals = [d['gyro_y_dps'] for d in self.raw_data_buffer]
        gyro_z_vals = [d['gyro_z_dps'] for d in self.raw_data_buffer]
        accel_mags = [d['accel_magnitude'] for d in self.raw_data_buffer]
        gyro_mags = [d['gyro_magnitude'] for d in self.raw_data_buffer]
        
        print(f"Raw samples: {len(self.raw_data_buffer)}")
        if self.raw_data_buffer:
            print(f"Duration: {self.raw_data_buffer[-1]['relative_time_ms']:.1f} ms")
        
        print(f"Accelerometer (mg):")
        print(f"  X: {min(accel_x_vals):.1f} to {max(accel_x_vals):.1f}, avg: {sum(accel_x_vals)/len(accel_x_vals):.1f}")
        print(f"  Y: {min(accel_y_vals):.1f} to {max(accel_y_vals):.1f}, avg: {sum(accel_y_vals)/len(accel_y_vals):.1f}")
        print(f"  Z: {min(accel_z_vals):.1f} to {max(accel_z_vals):.1f}, avg: {sum(accel_z_vals)/len(accel_z_vals):.1f}")
        print(f"  Magnitude: {min(accel_mags):.1f} to {max(accel_mags):.1f}, avg: {sum(accel_mags)/len(accel_mags):.1f}")
        
        print(f"Gyroscope (dps):")
        print(f"  X: {min(gyro_x_vals):.3f} to {max(gyro_x_vals):.3f}, avg: {sum(gyro_x_vals)/len(gyro_x_vals):.3f}")
        print(f"  Y: {min(gyro_y_vals):.3f} to {max(gyro_y_vals):.3f}, avg: {sum(gyro_y_vals)/len(gyro_y_vals):.3f}")
        print(f"  Z: {min(gyro_z_vals):.3f} to {max(gyro_z_vals):.3f}, avg: {sum(gyro_z_vals)/len(gyro_z_vals):.3f}")
        print(f"  Magnitude: {min(gyro_mags):.3f} to {max(gyro_mags):.3f}, avg: {sum(gyro_mags)/len(gyro_mags):.3f}")

    def _print_gyro_bias_summary(self):
        print("\nGYRO BIAS SUMMARY")
        print("-" * 20)
        if not self.gyro_bias_buffer:
            print("No gyro bias data captured.")
            return
            
        # Separate MDI and MFX bias data
        mdi_biases = [d for d in self.gyro_bias_buffer if d['bias_type'] == 'MDI']
        mfx_biases = [d for d in self.gyro_bias_buffer if d['bias_type'] == 'MFX']
        
        for bias_type, biases in [('MDI', mdi_biases), ('MFX', mfx_biases)]:
            if biases:
                print(f"{bias_type} Bias ({len(biases)} samples):")
                bias_x_vals = [d['bias_x'] for d in biases]
                bias_y_vals = [d['bias_y'] for d in biases]
                bias_z_vals = [d['bias_z'] for d in biases]
                bias_mags = [d['bias_magnitude'] for d in biases]
                
                print(f"  X: {min(bias_x_vals):.3f} to {max(bias_x_vals):.3f}, avg: {sum(bias_x_vals)/len(bias_x_vals):.3f}")
                print(f"  Y: {min(bias_y_vals):.3f} to {max(bias_y_vals):.3f}, avg: {sum(bias_y_vals)/len(bias_y_vals):.3f}")
                print(f"  Z: {min(bias_z_vals):.3f} to {max(bias_z_vals):.3f}, avg: {sum(bias_z_vals)/len(bias_z_vals):.3f}")
                print(f"  Magnitude: {min(bias_mags):.3f} to {max(bias_mags):.3f}, avg: {sum(bias_mags)/len(bias_mags):.3f}")

    def _print_temperature_summary(self):
        print("\nTEMPERATURE SUMMARY")
        print("-" * 22)
        if not self.temperature_buffer:
            print("No temperature data captured.")
            return
            
        temps = [d['temperature_celsius'] for d in self.temperature_buffer]
        print(f"Temperature samples: {len(self.temperature_buffer)}")
        if self.temperature_buffer:
            print(f"Duration: {self.temperature_buffer[-1]['relative_time_ms']:.1f} ms")
        print(f"Temperature range: {min(temps):.2f}°C to {max(temps):.2f}°C")
        print(f"Average temperature: {sum(temps)/len(temps):.2f}°C")
        
        # Check for temperature stability
        temp_std = (sum([(t - sum(temps)/len(temps))**2 for t in temps]) / len(temps))**0.5
        print(f"Temperature std dev: {temp_std:.3f}°C ({'STABLE' if temp_std < 1.0 else 'VARIABLE'})")

    def _print_quaternion_summary(self):
        print("\nQUATERNION CAPTURE SUMMARY")
        print("-" * 32)
        types = {}
        for q in self.quaternion_buffer:
            qtype = q['type']
            if qtype not in types:
                types[qtype] = []
            types[qtype].append(q)
        for qtype, quats in types.items():
            print(f"\n{qtype} Quaternions: {len(quats)} samples")
            if quats:
                mags = [q['magnitude'] for q in quats]
                print(f"  Magnitude range: {min(mags):.4f} to {max(mags):.4f}")
                # Check if quaternions are normalized (should be close to 1.0)
                avg_mag = sum(mags) / len(mags)
                if abs(avg_mag - 1.0) > 0.1:
                    print(f"\tWARNING: Average magnitude {avg_mag:.4f} - quaternions may not be normalized!")
                else:
                    print(f"\tAverage magnitude {avg_mag:.4f} - quaternions are properly normalized")

    def _print_summary(self):
        print("\nANGLE DATA SUMMARY")
        print("-" * 20)
        if not self.data_buffer:
            print("No angle data captured.")
            return

        alt_diffs = [d['altitude_diff'] for d in self.data_buffer]
        az_diffs = [d['azimuth_diff'] for d in self.data_buffer]
        mfx_alts = [d['mfx_altitude'] for d in self.data_buffer]
        mdi_alts = [d['mdi_altitude'] for d in self.data_buffer]
        mfx_azs = [d['mfx_azimuth'] for d in self.data_buffer]
        mdi_azs = [d['mdi_azimuth'] for d in self.data_buffer]
        print(f"Angle samples: {len(self.data_buffer)}")
        if self.data_buffer:
            print(f"Duration: {self.data_buffer[-1]['relative_time_ms']:.1f} ms")
        if alt_diffs:
            print(f"Altitude difference - Min: {min(alt_diffs):.3f}°, Max: {max(alt_diffs):.3f}°, Avg: {sum(alt_diffs)/len(alt_diffs):.3f}°")
        if az_diffs:
            print(f"Azimuth difference - Min: {min(az_diffs):.3f}°, Max: {max(az_diffs):.3f}°, Avg: {sum(az_diffs)/len(az_diffs):.3f}°")
        if mfx_alts:
            print(f"MFX Altitude range: {min(mfx_alts):.3f}° to {max(mfx_alts):.3f}°")
        if mdi_alts:
            print(f"MDI Altitude range: {min(mdi_alts):.3f}° to {max(mdi_alts):.3f}°")
        if mfx_azs:
            print(f"MFX Azimuth range: {min(mfx_azs):.3f}° to {max(mfx_azs):.3f}°")
        if mdi_azs:
            print(f"MDI Azimuth range: {min(mdi_azs):.3f}° to {max(mdi_azs):.3f}°")


def main():
    parser = argparse.ArgumentParser(description="Capture motion data from serial or file.")
    parser.add_argument('--port', type=str, default="COM4", help="Serial port (default: COM4)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    # parser.add_argument('--baud', type=int, default=921600, help="Baud rate (default: 115200)")
    parser.add_argument('--duration', type=int, help="Capture duration in seconds (default: None - run indefinitely until Ctrl+C)")
    parser.add_argument('--file', type=str, help="Path to a text file containing motion data to process")
    parser.add_argument('--output', type=str, help="Base name for output CSV files (e.g., 'mydata' creates multiple CSV files)")

    args = parser.parse_args()

    output_file = args.output if args.output else None # MotionDataCapture should handle default naming if None

    capture = MotionDataCapture(output_file=output_file)

    if args.file:
        # Process data from file
        capture.can_start = True
        capture.process_file(args.file)
    else:
        if not capture.connect(args.port, args.baud):
            print(f"Failed to connect to {args.port}")
            return 1
        capture.start_capture(args.duration)

    return 0

if __name__ == "__main__":
    sys.exit(main())