#!/usr/bin/env python3
"""
Live Motion Detection Plotter

Real-time visualization of motion detection data using console_reader.
Displays all angle sources in separate subplots for comprehensive analysis.

Usage:
    python3 live_motion_plotter.py --port <serial_port>
    
Examples:
    python3 live_motion_plotter.py --port COM15
    python3 live_motion_plotter.py --port /dev/ttyUSB0 --baudrate 115200
"""

import sys
import os
import argparse
import time
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Import console_reader from the motion_detection_ideas directory
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../motion_detection_ideas'))
try:
    from console_reader import MotionConsole
    CONSOLE_AVAILABLE = True
except ImportError:
    print("Error: console_reader module not found!")
    print("Make sure console_reader.py is available in motion_detection_ideas/")
    CONSOLE_AVAILABLE = False
    sys.exit(1)

class LiveMotionData:
    def __init__(self, max_samples=500):
        self.max_samples = max_samples
        
        # Time series data with circular buffers
        self.timestamps = deque(maxlen=max_samples)
        
        # Raw sensor data
        self.accel_x = deque(maxlen=max_samples)
        self.accel_y = deque(maxlen=max_samples)
        self.accel_z = deque(maxlen=max_samples)
        self.gyro_x = deque(maxlen=max_samples)
        self.gyro_y = deque(maxlen=max_samples)
        self.gyro_z = deque(maxlen=max_samples)
        
        # Final angles (altitude, azimuth, zenith)
        self.angles_altitude = deque(maxlen=max_samples)
        self.angles_azimuth = deque(maxlen=max_samples)
        self.angles_zenith = deque(maxlen=max_samples)
        
        # MotionDI angles (pitch, yaw, roll)
        self.di_pitch = deque(maxlen=max_samples)
        self.di_yaw = deque(maxlen=max_samples)
        self.di_roll = deque(maxlen=max_samples)
        
        # Simple Integration filter (pitch, yaw, roll)
        self.si_pitch = deque(maxlen=max_samples)
        self.si_yaw = deque(maxlen=max_samples)
        self.si_roll = deque(maxlen=max_samples)
        
        # Complementary filter (pitch, yaw, roll)
        self.co_pitch = deque(maxlen=max_samples)
        self.co_yaw = deque(maxlen=max_samples)
        self.co_roll = deque(maxlen=max_samples)
        
        # Fused angles (pitch, yaw, roll)
        self.fu_pitch = deque(maxlen=max_samples)
        self.fu_yaw = deque(maxlen=max_samples)
        self.fu_roll = deque(maxlen=max_samples)
        
        # Gyro bias
        self.bias_x = deque(maxlen=max_samples)
        self.bias_y = deque(maxlen=max_samples)
        self.bias_z = deque(maxlen=max_samples)
        
        # State tracking
        self.last_update_time = time.time()
        self.data_lock = threading.Lock()
        
    def add_raw_data(self, timestamp, accel_mg, gyro_dps):
        """Add raw sensor data point"""
        with self.data_lock:
            self.timestamps.append(timestamp)
            self.accel_x.append(accel_mg[0])
            self.accel_y.append(accel_mg[1])
            self.accel_z.append(accel_mg[2])
            self.gyro_x.append(gyro_dps[0])
            self.gyro_y.append(gyro_dps[1])
            self.gyro_z.append(gyro_dps[2])
            self.last_update_time = time.time()
    
    def add_angles_data(self, timestamp, altitude, azimuth, zenith):
        """Add final angles data point"""
        with self.data_lock:
            self.angles_altitude.append(altitude)
            self.angles_azimuth.append(azimuth)
            self.angles_zenith.append(zenith)
    
    def add_di_angles(self, timestamp, pitch, yaw, roll):
        """Add MotionDI angles data point"""
        with self.data_lock:
            self.di_pitch.append(pitch)
            self.di_yaw.append(yaw)
            self.di_roll.append(roll)
    
    def add_si_angles(self, timestamp, pitch, yaw, roll):
        """Add Simple Integration filter data point"""
        with self.data_lock:
            self.si_pitch.append(pitch)
            self.si_yaw.append(yaw)
            self.si_roll.append(roll)
    
    def add_co_angles(self, timestamp, pitch, yaw, roll):
        """Add Complementary filter data point"""
        with self.data_lock:
            self.co_pitch.append(pitch)
            self.co_yaw.append(yaw)
            self.co_roll.append(roll)
    
    def add_fu_angles(self, timestamp, pitch, yaw, roll):
        """Add Fused angles data point"""
        with self.data_lock:
            self.fu_pitch.append(pitch)
            self.fu_yaw.append(yaw)
            self.fu_roll.append(roll)
    
    def add_gyro_bias(self, timestamp, bias_x, bias_y, bias_z):
        """Add gyro bias data point"""
        with self.data_lock:
            self.bias_x.append(bias_x)
            self.bias_y.append(bias_y)
            self.bias_z.append(bias_z)
    
    def get_data_snapshot(self):
        """Get thread-safe snapshot of current data"""
        with self.data_lock:
            return {
                'timestamps': list(self.timestamps),
                'accel': [list(self.accel_x), list(self.accel_y), list(self.accel_z)],
                'gyro': [list(self.gyro_x), list(self.gyro_y), list(self.gyro_z)],
                'angles': [list(self.angles_altitude), list(self.angles_azimuth), list(self.angles_zenith)],
                'di': [list(self.di_pitch), list(self.di_yaw), list(self.di_roll)],
                'si': [list(self.si_pitch), list(self.si_yaw), list(self.si_roll)],
                'co': [list(self.co_pitch), list(self.co_yaw), list(self.co_roll)],
                'fu': [list(self.fu_pitch), list(self.fu_yaw), list(self.fu_roll)],
                'bias': [list(self.bias_x), list(self.bias_y), list(self.bias_z)]
            }

class LiveMotionPlotter:
    def __init__(self, port, baudrate=115200, max_samples=500):
        self.port = port
        self.baudrate = baudrate
        self.data = LiveMotionData(max_samples)
        self.console = None
        self.running = False
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(3, 3, figsize=(18, 12))
        self.fig.suptitle(f'Live Motion Detection Data - {port}', fontsize=16)
        
        # Configure subplots
        self.axes[0, 0].set_title('Raw Accelerometer (mg)')
        self.axes[0, 1].set_title('Raw Gyroscope (dps)')
        self.axes[0, 2].set_title('ANGLES - Final Output')
        self.axes[1, 0].set_title('ANGLES_DI - MotionDI Euler')
        self.axes[1, 1].set_title('ANGLES_SI - Simple Integration')
        self.axes[1, 2].set_title('ANGLES_CO - Complementary')
        self.axes[2, 0].set_title('ANGLES_FU - Fused Output')
        self.axes[2, 1].set_title('Gyro Bias (dps)')
        self.axes[2, 2].set_title('Pitch Comparison')
        
        for ax_row in self.axes:
            for ax in ax_row:
                ax.grid(True, alpha=0.3)
                ax.set_xlabel('Time (s)')
        
        plt.tight_layout()
        
    def connect_and_start(self):
        """Connect to device and start live plotting"""
        print(f"Connecting to {self.port} at {self.baudrate} baud...")
        
        self.console = MotionConsole(rx_callback=self._handle_serial_data)
        
        if not self.console.connect(self.port, self.baudrate):
            print(f"Failed to connect to {self.port}")
            return False
        
        print("Connected successfully!")
        print("Enabling debug output...")
        
        # Enable all debug outputs
        time.sleep(0.5)
        self.console.send_command("printraw 1")
        time.sleep(0.1)
        self.console.send_command("setdebug 127")  # Enable all debug flags including PRINT_ESTIMATOR
        time.sleep(0.1)
        
        print("Starting live visualization...")
        print("Press Ctrl+C to stop")
        
        self.running = True
        
        # Start animation
        try:
            ani = FuncAnimation(self.fig, self._update_plots, interval=100, blit=False)
            plt.show()
        except KeyboardInterrupt:
            print("\\nStopping...")
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """Stop the live plotter and disconnect"""
        self.running = False
        if self.console:
            self.console.disconnect()
        plt.close('all')
    
    def _handle_serial_data(self, line):
        """Parse incoming serial data"""
        if not self.running:
            return
            
        try:
            current_time = time.time()
            
            if line.startswith('RAW_DATA'):
                self._parse_raw_data(line, current_time)
            elif line.startswith('ANGLES,'):
                self._parse_angles(line, current_time)
            elif line.startswith('ANGLES_DI'):
                self._parse_angles_di(line, current_time)
            elif line.startswith('ANGLES_SI'):
                self._parse_angles_si(line, current_time)
            elif line.startswith('ANGLES_CO'):
                self._parse_angles_co(line, current_time)
            elif line.startswith('ANGLES_FU'):
                self._parse_angles_fu(line, current_time)
            elif line.startswith('GYRO_BIAS_MDI'):
                self._parse_gyro_bias(line, current_time)
                
        except Exception as e:
            print(f"Error parsing line: {e}")
    
    def _parse_raw_data(self, line, timestamp):
        """Parse RAW_DATA line"""
        try:
            parts = line.split(',')
            if len(parts) >= 7:
                accel_mg = [float(parts[2]), float(parts[3]), float(parts[4])]
                gyro_dps = [float(parts[5]), float(parts[6]), float(parts[7])]
                self.data.add_raw_data(timestamp, accel_mg, gyro_dps)
        except (ValueError, IndexError):
            pass
    
    def _parse_angles(self, line, timestamp):
        """Parse ANGLES line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                altitude = float(parts[2])
                azimuth = float(parts[3])
                zenith = float(parts[4])
                self.data.add_angles_data(timestamp, altitude, azimuth, zenith)
        except (ValueError, IndexError):
            pass
    
    def _parse_angles_di(self, line, timestamp):
        """Parse ANGLES_DI line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                pitch = float(parts[2])
                yaw = float(parts[3])
                roll = float(parts[4])
                self.data.add_di_angles(timestamp, pitch, yaw, roll)
        except (ValueError, IndexError):
            pass
    
    def _parse_angles_si(self, line, timestamp):
        """Parse ANGLES_SI line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                pitch = float(parts[2])
                yaw = float(parts[3])
                roll = float(parts[4])
                self.data.add_si_angles(timestamp, pitch, yaw, roll)
        except (ValueError, IndexError):
            pass
    
    def _parse_angles_co(self, line, timestamp):
        """Parse ANGLES_CO line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                pitch = float(parts[2])
                yaw = float(parts[3])
                roll = float(parts[4])
                self.data.add_co_angles(timestamp, pitch, yaw, roll)
        except (ValueError, IndexError):
            pass
    
    def _parse_angles_fu(self, line, timestamp):
        """Parse ANGLES_FU line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                pitch = float(parts[2])
                yaw = float(parts[3])
                roll = float(parts[4])
                self.data.add_fu_angles(timestamp, pitch, yaw, roll)
        except (ValueError, IndexError):
            pass
    
    def _parse_gyro_bias(self, line, timestamp):
        """Parse GYRO_BIAS_MDI line"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                bias_x = float(parts[2])
                bias_y = float(parts[3])
                bias_z = float(parts[4])
                self.data.add_gyro_bias(timestamp, bias_x, bias_y, bias_z)
        except (ValueError, IndexError):
            pass
    
    def _update_plots(self, frame):
        """Update all plots with latest data"""
        snapshot = self.data.get_data_snapshot()
        
        if not snapshot['timestamps']:
            return
        
        # Clear all axes
        for ax_row in self.axes:
            for ax in ax_row:
                ax.clear()
                ax.grid(True, alpha=0.3)
        
        timestamps = np.array(snapshot['timestamps'])
        
        # Plot 1: Raw Accelerometer
        if len(timestamps) > 0:
            self.axes[0, 0].plot(timestamps, snapshot['accel'][0], 'r-', label='X', alpha=0.7)
            self.axes[0, 0].plot(timestamps, snapshot['accel'][1], 'g-', label='Y', alpha=0.7)
            self.axes[0, 0].plot(timestamps, snapshot['accel'][2], 'b-', label='Z', alpha=0.7)
            self.axes[0, 0].set_title('Raw Accelerometer (mg)')
            self.axes[0, 0].set_ylabel('Acceleration (mg)')
            self.axes[0, 0].legend()
        
        # Plot 2: Raw Gyroscope
        if len(timestamps) > 0:
            self.axes[0, 1].plot(timestamps, snapshot['gyro'][0], 'r-', label='X', alpha=0.7)
            self.axes[0, 1].plot(timestamps, snapshot['gyro'][1], 'g-', label='Y', alpha=0.7)
            self.axes[0, 1].plot(timestamps, snapshot['gyro'][2], 'b-', label='Z', alpha=0.7)
            self.axes[0, 1].set_title('Raw Gyroscope (dps)')
            self.axes[0, 1].set_ylabel('Angular Rate (dps)')
            self.axes[0, 1].legend()
        
        # Plot 3: Final ANGLES
        if len(snapshot['angles'][0]) > 0:
            angle_timestamps = timestamps[-len(snapshot['angles'][0]):]
            self.axes[0, 2].plot(angle_timestamps, snapshot['angles'][0], 'r-', label='Altitude', linewidth=2)
            self.axes[0, 2].plot(angle_timestamps, snapshot['angles'][1], 'g-', label='Azimuth', linewidth=2)
            self.axes[0, 2].plot(angle_timestamps, snapshot['angles'][2], 'b-', label='Zenith', linewidth=2)
            self.axes[0, 2].set_title('ANGLES - Final Output')
            self.axes[0, 2].set_ylabel('Angle (degrees)')
            self.axes[0, 2].legend()
        
        # Plot 4: MotionDI Angles
        if len(snapshot['di'][0]) > 0:
            di_timestamps = timestamps[-len(snapshot['di'][0]):]
            self.axes[1, 0].plot(di_timestamps, snapshot['di'][0], 'r-', label='Pitch', alpha=0.8)
            self.axes[1, 0].plot(di_timestamps, snapshot['di'][1], 'g-', label='Yaw', alpha=0.8)
            self.axes[1, 0].plot(di_timestamps, snapshot['di'][2], 'b-', label='Roll', alpha=0.8)
            self.axes[1, 0].set_title('ANGLES_DI - MotionDI Euler')
            self.axes[1, 0].set_ylabel('Angle (degrees)')
            self.axes[1, 0].legend()
        
        # Plot 5: Simple Integration Filter
        if len(snapshot['si'][0]) > 0:
            si_timestamps = timestamps[-len(snapshot['si'][0]):]
            self.axes[1, 1].plot(si_timestamps, snapshot['si'][0], 'r--', label='Pitch', alpha=0.7)
            self.axes[1, 1].plot(si_timestamps, snapshot['si'][1], 'g--', label='Yaw', alpha=0.7)
            self.axes[1, 1].plot(si_timestamps, snapshot['si'][2], 'b--', label='Roll', alpha=0.7)
            self.axes[1, 1].set_title('ANGLES_SI - Simple Integration')
            self.axes[1, 1].set_ylabel('Angle (degrees)')
            self.axes[1, 1].legend()
        
        # Plot 6: Complementary Filter
        if len(snapshot['co'][0]) > 0:
            co_timestamps = timestamps[-len(snapshot['co'][0]):]
            self.axes[1, 2].plot(co_timestamps, snapshot['co'][0], 'r:', label='Pitch', alpha=0.8, linewidth=2)
            self.axes[1, 2].plot(co_timestamps, snapshot['co'][1], 'g:', label='Yaw', alpha=0.8, linewidth=2)
            self.axes[1, 2].plot(co_timestamps, snapshot['co'][2], 'b:', label='Roll', alpha=0.8, linewidth=2)
            self.axes[1, 2].set_title('ANGLES_CO - Complementary')
            self.axes[1, 2].set_ylabel('Angle (degrees)')
            self.axes[1, 2].legend()
        
        # Plot 7: Fused Angles
        if len(snapshot['fu'][0]) > 0:
            fu_timestamps = timestamps[-len(snapshot['fu'][0]):]
            self.axes[2, 0].plot(fu_timestamps, snapshot['fu'][0], 'r-', label='Pitch', alpha=0.9, linewidth=2)
            self.axes[2, 0].plot(fu_timestamps, snapshot['fu'][1], 'g-', label='Yaw', alpha=0.9, linewidth=2)
            self.axes[2, 0].plot(fu_timestamps, snapshot['fu'][2], 'b-', label='Roll', alpha=0.9, linewidth=2)
            self.axes[2, 0].set_title('ANGLES_FU - Fused Output')
            self.axes[2, 0].set_ylabel('Angle (degrees)')
            self.axes[2, 0].legend()
        
        # Plot 8: Gyro Bias
        if len(snapshot['bias'][0]) > 0:
            bias_timestamps = timestamps[-len(snapshot['bias'][0]):]
            self.axes[2, 1].plot(bias_timestamps, snapshot['bias'][0], 'r-', label='X Bias', alpha=0.7)
            self.axes[2, 1].plot(bias_timestamps, snapshot['bias'][1], 'g-', label='Y Bias', alpha=0.7)
            self.axes[2, 1].plot(bias_timestamps, snapshot['bias'][2], 'b-', label='Z Bias', alpha=0.7)
            self.axes[2, 1].set_title('Gyro Bias (dps)')
            self.axes[2, 1].set_ylabel('Bias (dps)')
            self.axes[2, 1].legend()
        
        # Plot 9: Pitch Comparison
        self._plot_pitch_comparison(snapshot)
        
        # Set x-axis labels
        for ax in self.axes[2, :]:
            ax.set_xlabel('Time (s)')
        
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)
    
    def _plot_pitch_comparison(self, snapshot):
        """Plot pitch from all sources for comparison"""
        ax = self.axes[2, 2]
        
        # Final angles (altitude)
        if len(snapshot['angles'][0]) > 0:
            timestamps = np.array(snapshot['timestamps'][-len(snapshot['angles'][0]):])
            ax.plot(timestamps, snapshot['angles'][0], 'k-', label='ANGLES (Final)', linewidth=3, alpha=0.9)
        
        # MotionDI pitch
        if len(snapshot['di'][0]) > 0:
            timestamps = np.array(snapshot['timestamps'][-len(snapshot['di'][0]):])
            ax.plot(timestamps, snapshot['di'][0], 'r-', label='DI Pitch', alpha=0.7)
        
        # Simple Integration pitch
        if len(snapshot['si'][0]) > 0:
            timestamps = np.array(snapshot['timestamps'][-len(snapshot['si'][0]):])
            ax.plot(timestamps, snapshot['si'][0], 'g--', label='SI Pitch', alpha=0.6)
        
        # Complementary pitch
        if len(snapshot['co'][0]) > 0:
            timestamps = np.array(snapshot['timestamps'][-len(snapshot['co'][0]):])
            ax.plot(timestamps, snapshot['co'][0], 'b:', label='CO Pitch', alpha=0.7, linewidth=2)
        
        # Fused pitch
        if len(snapshot['fu'][0]) > 0:
            timestamps = np.array(snapshot['timestamps'][-len(snapshot['fu'][0]):])
            ax.plot(timestamps, snapshot['fu'][0], 'm-', label='FU Pitch', alpha=0.8)
        
        ax.set_title('Pitch/Altitude Comparison')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
    
    def start(self):
        """Start the live plotter"""
        return self.connect_and_start()

def main():
    parser = argparse.ArgumentParser(description='Live Motion Detection Plotter')
    parser.add_argument('--port', required=True, help='Serial port (e.g., COM15, /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate')
    parser.add_argument('--samples', type=int, default=500, help='Maximum samples to display')
    
    args = parser.parse_args()
    
    if not CONSOLE_AVAILABLE:
        print("Error: console_reader module not available!")
        return
    
    plotter = LiveMotionPlotter(args.port, args.baudrate, args.samples)
    plotter.start()

if __name__ == "__main__":
    main()