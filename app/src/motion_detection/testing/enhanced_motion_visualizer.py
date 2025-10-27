#!/usr/bin/env python3
"""
Enhanced Motion Detection Debug Visualizer

This tool helps debug motion detection issues by visualizing:
1. Raw sensor data (accelerometer and gyroscope)
2. ANGLES - Final output angles (altitude, azimuth, zenith)
3. ANGLES_DI - MotionDI euler angles (pitch, yaw, roll)
4. ANGLES_SI - Simple Integration filter angles (pitch, yaw, roll)
5. ANGLES_CO - Complementary filter angles (pitch, yaw, roll)
6. ANGLES_FU - Fused angles from MDI + MotionEstimator (pitch, yaw, roll)
7. Gyro bias over time

Supports both file-based and live data visualization.

Usage:
    # File-based analysis
    python3 enhanced_motion_visualizer.py <log_file>
    
    # Live data visualization
    python3 enhanced_motion_visualizer.py --live --port /dev/ttyUSB0
    
Examples:
    python3 enhanced_motion_visualizer.py motion_detection.log
    python3 enhanced_motion_visualizer.py --live --port COM15 --baudrate 115200
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
import argparse
from datetime import datetime
import threading
import queue
import time
from collections import deque
import os

# Import console_reader from the motion_detection_ideas directory
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../motion_detection_ideas'))
try:
    from console_reader import MotionConsole
    LIVE_MODE_AVAILABLE = True
except ImportError:
    print("Warning: console_reader not found. Live mode will not be available.")
    LIVE_MODE_AVAILABLE = False

class EnhancedMotionDataParser:
    def __init__(self, max_live_samples=2000):
        self.raw_data = []
        self.angles = []           # Final output angles (altitude, azimuth, zenith)
        self.angles_di = []        # MotionDI euler angles (pitch, yaw, roll)
        self.angles_si = []        # Simple Integration filter (pitch, yaw, roll)
        self.angles_co = []        # Complementary filter (pitch, yaw, roll)
        self.angles_fu = []        # Fused angles (pitch, yaw, roll)
        self.gyro_bias = []
        self.timestamps = []
        
        # Live mode data structures
        self.max_live_samples = max_live_samples
        self.live_mode = False
        self.data_queue = queue.Queue()
        
    def enable_live_mode(self):
        """Enable live mode with circular buffers"""
        self.live_mode = True
        self.raw_data = deque(maxlen=self.max_live_samples)
        self.angles = deque(maxlen=self.max_live_samples)
        self.angles_di = deque(maxlen=self.max_live_samples)
        self.angles_si = deque(maxlen=self.max_live_samples)
        self.angles_co = deque(maxlen=self.max_live_samples)
        self.angles_fu = deque(maxlen=self.max_live_samples)
        self.gyro_bias = deque(maxlen=self.max_live_samples)
        self.timestamps = deque(maxlen=self.max_live_samples)
        
    def parse_log_file(self, filename):
        """Parse the motion detection log file"""
        print(f"Parsing log file: {filename}")
        
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    self._parse_line(line.strip(), line_num)
                except Exception as e:
                    print(f"Warning: Error parsing line {line_num}: {e}")
                    continue
        
        print(f"Parsed {len(self.raw_data)} raw data samples")
        print(f"Parsed {len(self.angles)} ANGLES samples")
        print(f"Parsed {len(self.angles_di)} ANGLES_DI samples")
        print(f"Parsed {len(self.angles_si)} ANGLES_SI samples")
        print(f"Parsed {len(self.angles_co)} ANGLES_CO samples")
        print(f"Parsed {len(self.angles_fu)} ANGLES_FU samples")
        print(f"Parsed {len(self.gyro_bias)} gyro bias samples")
    
    def parse_live_data(self, line):
        """Parse a single line from live data"""
        try:
            self._parse_line(line.strip(), time.time())
            return True
        except Exception as e:
            print(f"Warning: Error parsing live line: {e}")
            return False
    
    def _parse_line(self, line, timestamp_or_line_num):
        """Parse a single log line"""
        if not line:
            return
            
        # Extract timestamp if present, otherwise use provided value
        timestamp_match = re.search(r'\[([\d.]+)s\]', line)
        timestamp = float(timestamp_match.group(1)) if timestamp_match else timestamp_or_line_num
        
        # Parse different data types
        if line.startswith('RAW_DATA'):
            self._parse_raw_data(line, timestamp)
        elif line.startswith('ANGLES,'):
            self._parse_angles(line, timestamp)
        elif line.startswith('ANGLES_DI'):
            self._parse_angles_di(line, timestamp)
        elif line.startswith('ANGLES_SI'):
            self._parse_angles_si(line, timestamp)
        elif line.startswith('ANGLES_CO'):
            self._parse_angles_co(line, timestamp)
        elif line.startswith('ANGLES_FU'):
            self._parse_angles_fu(line, timestamp)
        elif line.startswith('GYRO_BIAS_MDI'):
            self._parse_gyro_bias(line, timestamp)
    
    def _parse_raw_data(self, line, timestamp):
        """Parse RAW_DATA line: RAW_DATA,timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z"""
        try:
            parts = line.split(',')
            if len(parts) >= 7:
                data = {
                    'timestamp': timestamp,
                    'accel_mg': [float(parts[2]), float(parts[3]), float(parts[4])],
                    'gyro_dps': [float(parts[5]), float(parts[6]), float(parts[7])]
                }
                self.raw_data.append(data)
                if not self.live_mode:
                    self.timestamps.append(timestamp)
        except (ValueError, IndexError) as e:
            print(f"Error parsing RAW_DATA line: {e}")
    
    def _parse_angles(self, line, timestamp):
        """Parse ANGLES line: ANGLES,timestamp,altitude,azimuth,zenith,state"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'altitude': float(parts[2]),
                    'azimuth': float(parts[3]), 
                    'zenith': float(parts[4]),
                    'state': parts[5] if len(parts) > 5 else 'UNKNOWN'
                }
                self.angles.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing ANGLES line: {e}")
    
    def _parse_angles_di(self, line, timestamp):
        """Parse ANGLES_DI line: ANGLES_DI,timestamp,pitch,yaw,roll"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'pitch': float(parts[2]),
                    'yaw': float(parts[3]),
                    'roll': float(parts[4])
                }
                self.angles_di.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing ANGLES_DI line: {e}")
    
    def _parse_angles_si(self, line, timestamp):
        """Parse ANGLES_SI line: ANGLES_SI,timestamp,pitch,yaw,roll"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'pitch': float(parts[2]),
                    'yaw': float(parts[3]),
                    'roll': float(parts[4])
                }
                self.angles_si.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing ANGLES_SI line: {e}")
    
    def _parse_angles_co(self, line, timestamp):
        """Parse ANGLES_CO line: ANGLES_CO,timestamp,pitch,yaw,roll"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'pitch': float(parts[2]),
                    'yaw': float(parts[3]),
                    'roll': float(parts[4])
                }
                self.angles_co.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing ANGLES_CO line: {e}")
    
    def _parse_angles_fu(self, line, timestamp):
        """Parse ANGLES_FU line: ANGLES_FU,timestamp,pitch,yaw,roll"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'pitch': float(parts[2]),
                    'yaw': float(parts[3]),
                    'roll': float(parts[4])
                }
                self.angles_fu.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing ANGLES_FU line: {e}")
    
    def _parse_gyro_bias(self, line, timestamp):
        """Parse GYRO_BIAS_MDI line: GYRO_BIAS_MDI,timestamp,bias_x,bias_y,bias_z"""
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                data = {
                    'timestamp': timestamp,
                    'bias_x': float(parts[2]),
                    'bias_y': float(parts[3]),
                    'bias_z': float(parts[4])
                }
                self.gyro_bias.append(data)
        except (ValueError, IndexError) as e:
            print(f"Error parsing GYRO_BIAS_MDI line: {e}")

class EnhancedMotionVisualizer:
    def __init__(self, parser):
        self.parser = parser
        self.fig = None
        self.axes = {}
        self.live_mode = False
        
    def create_static_visualization(self):
        """Create static visualization of motion data from log file"""
        if not self.parser.raw_data:
            print("No data to visualize!")
            return
            
        # Create figure with subplots
        self.fig = plt.figure(figsize=(20, 14))
        gs = GridSpec(4, 3, figure=self.fig)
        
        # Plot 1: Raw accelerometer data
        ax1 = self.fig.add_subplot(gs[0, 0])
        self._plot_raw_accelerometer(ax1)
        
        # Plot 2: Raw gyroscope data  
        ax2 = self.fig.add_subplot(gs[0, 1])
        self._plot_raw_gyroscope(ax2)
        
        # Plot 3: Final ANGLES output
        ax3 = self.fig.add_subplot(gs[0, 2])
        self._plot_angles_output(ax3)
        
        # Plot 4: MotionDI angles (ANGLES_DI)
        ax4 = self.fig.add_subplot(gs[1, 0])
        self._plot_angles_di(ax4)
        
        # Plot 5: Simple Integration filter (ANGLES_SI)
        ax5 = self.fig.add_subplot(gs[1, 1])
        self._plot_angles_si(ax5)
        
        # Plot 6: Complementary filter (ANGLES_CO)
        ax6 = self.fig.add_subplot(gs[1, 2])
        self._plot_angles_co(ax6)
        
        # Plot 7: Fused angles (ANGLES_FU)
        ax7 = self.fig.add_subplot(gs[2, 0])
        self._plot_angles_fu(ax7)
        
        # Plot 8: Gyro bias over time
        ax8 = self.fig.add_subplot(gs[2, 1])
        self._plot_gyro_bias(ax8)
        
        # Plot 9: Comparison of all angle sources
        ax9 = self.fig.add_subplot(gs[2, 2])
        self._plot_angle_comparison(ax9)
        
        # Plot 10: Analysis summary
        ax10 = self.fig.add_subplot(gs[3, :])
        self._plot_analysis_summary(ax10)
        
        plt.tight_layout()
        plt.show()
        
    def create_live_visualization(self):
        """Create live visualization that updates with incoming data"""
        if not LIVE_MODE_AVAILABLE:
            print("Error: Live mode not available. console_reader module not found.")
            return False
            
        self.live_mode = True
        
        # Create figure with subplots for live mode
        self.fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(3, 3, figure=self.fig)
        
        # Store axes for live updates
        self.axes = {
            'raw_accel': self.fig.add_subplot(gs[0, 0]),
            'raw_gyro': self.fig.add_subplot(gs[0, 1]),
            'angles_final': self.fig.add_subplot(gs[0, 2]),
            'angles_di': self.fig.add_subplot(gs[1, 0]),
            'angles_filters': self.fig.add_subplot(gs[1, 1]),
            'angles_fused': self.fig.add_subplot(gs[1, 2]),
            'gyro_bias': self.fig.add_subplot(gs[2, 0]),
            'comparison': self.fig.add_subplot(gs[2, 1:])
        }
        
        # Initialize empty plots
        self._init_live_plots()
        
        return True
        
    def _init_live_plots(self):
        """Initialize empty plots for live mode"""
        for ax_name, ax in self.axes.items():
            ax.clear()
            ax.grid(True, alpha=0.3)
            
        # Set titles
        self.axes['raw_accel'].set_title('Raw Accelerometer (mg)')
        self.axes['raw_gyro'].set_title('Raw Gyroscope (dps)')
        self.axes['angles_final'].set_title('ANGLES - Final Output')
        self.axes['angles_di'].set_title('ANGLES_DI - MotionDI Euler')
        self.axes['angles_filters'].set_title('ANGLES_SI/CO - Filters')
        self.axes['angles_fused'].set_title('ANGLES_FU - Fused')
        self.axes['gyro_bias'].set_title('Gyro Bias (dps)')
        self.axes['comparison'].set_title('Angle Comparison - All Sources')
        
    def update_live_plots(self, frame):
        """Update plots with latest data for live visualization"""
        if not self.parser.raw_data:
            return
            
        # Clear and redraw all plots
        for ax in self.axes.values():
            ax.clear()
            ax.grid(True, alpha=0.3)
        
        # Update each plot
        self._plot_raw_accelerometer(self.axes['raw_accel'])
        self._plot_raw_gyroscope(self.axes['raw_gyro'])
        self._plot_angles_output(self.axes['angles_final'])
        self._plot_angles_di(self.axes['angles_di'])
        self._plot_angles_filters_combined(self.axes['angles_filters'])
        self._plot_angles_fu(self.axes['angles_fused'])
        self._plot_gyro_bias(self.axes['gyro_bias'])
        self._plot_live_comparison(self.axes['comparison'])
        
        # Re-set titles
        self.axes['raw_accel'].set_title('Raw Accelerometer (mg)')
        self.axes['raw_gyro'].set_title('Raw Gyroscope (dps)')
        self.axes['angles_final'].set_title('ANGLES - Final Output')
        self.axes['angles_di'].set_title('ANGLES_DI - MotionDI Euler')
        self.axes['angles_filters'].set_title('ANGLES_SI/CO - Filters')
        self.axes['angles_fused'].set_title('ANGLES_FU - Fused')
        self.axes['gyro_bias'].set_title('Gyro Bias (dps)')
        self.axes['comparison'].set_title('Angle Comparison - All Sources')
        
        plt.tight_layout()
        
    def _plot_raw_accelerometer(self, ax):
        """Plot raw accelerometer data"""
        if not self.parser.raw_data:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.raw_data]
        accel_x = [d['accel_mg'][0] for d in self.parser.raw_data]
        accel_y = [d['accel_mg'][1] for d in self.parser.raw_data]
        accel_z = [d['accel_mg'][2] for d in self.parser.raw_data]
        
        ax.plot(timestamps, accel_x, 'r-', label='Accel X', alpha=0.7)
        ax.plot(timestamps, accel_y, 'g-', label='Accel Y', alpha=0.7)
        ax.plot(timestamps, accel_z, 'b-', label='Accel Z', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (mg)')
        ax.legend()
        
    def _plot_raw_gyroscope(self, ax):
        """Plot raw gyroscope data"""
        if not self.parser.raw_data:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.raw_data]
        gyro_x = [d['gyro_dps'][0] for d in self.parser.raw_data]
        gyro_y = [d['gyro_dps'][1] for d in self.parser.raw_data]
        gyro_z = [d['gyro_dps'][2] for d in self.parser.raw_data]
        
        ax.plot(timestamps, gyro_x, 'r-', label='Gyro X', alpha=0.7)
        ax.plot(timestamps, gyro_y, 'g-', label='Gyro Y', alpha=0.7)
        ax.plot(timestamps, gyro_z, 'b-', label='Gyro Z', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Rate (dps)')
        ax.legend()
        
    def _plot_angles_output(self, ax):
        """Plot final ANGLES output (altitude, azimuth, zenith)"""
        if not self.parser.angles:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles]
        altitude = [d['altitude'] for d in self.parser.angles]
        azimuth = [d['azimuth'] for d in self.parser.angles]
        zenith = [d['zenith'] for d in self.parser.angles]
        
        ax.plot(timestamps, altitude, 'r-', label='Altitude', alpha=0.8, linewidth=2)
        ax.plot(timestamps, azimuth, 'g-', label='Azimuth', alpha=0.8, linewidth=2)
        ax.plot(timestamps, zenith, 'b-', label='Zenith', alpha=0.8, linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_angles_di(self, ax):
        """Plot MotionDI euler angles (pitch, yaw, roll)"""
        if not self.parser.angles_di:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_di]
        pitch = [d['pitch'] for d in self.parser.angles_di]
        yaw = [d['yaw'] for d in self.parser.angles_di]
        roll = [d['roll'] for d in self.parser.angles_di]
        
        ax.plot(timestamps, pitch, 'r-', label='Pitch', alpha=0.8)
        ax.plot(timestamps, yaw, 'g-', label='Yaw', alpha=0.8)
        ax.plot(timestamps, roll, 'b-', label='Roll', alpha=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_angles_si(self, ax):
        """Plot Simple Integration filter angles"""
        if not self.parser.angles_si:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_si]
        pitch = [d['pitch'] for d in self.parser.angles_si]
        yaw = [d['yaw'] for d in self.parser.angles_si]
        roll = [d['roll'] for d in self.parser.angles_si]
        
        ax.plot(timestamps, pitch, 'r--', label='SI Pitch', alpha=0.8)
        ax.plot(timestamps, yaw, 'g--', label='SI Yaw', alpha=0.8)
        ax.plot(timestamps, roll, 'b--', label='SI Roll', alpha=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_angles_co(self, ax):
        """Plot Complementary filter angles"""
        if not self.parser.angles_co:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_co]
        pitch = [d['pitch'] for d in self.parser.angles_co]
        yaw = [d['yaw'] for d in self.parser.angles_co]
        roll = [d['roll'] for d in self.parser.angles_co]
        
        ax.plot(timestamps, pitch, 'r:', label='CO Pitch', alpha=0.8)
        ax.plot(timestamps, yaw, 'g:', label='CO Yaw', alpha=0.8)
        ax.plot(timestamps, roll, 'b:', label='CO Roll', alpha=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_angles_fu(self, ax):
        """Plot Fused angles"""
        if not self.parser.angles_fu:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_fu]
        pitch = [d['pitch'] for d in self.parser.angles_fu]
        yaw = [d['yaw'] for d in self.parser.angles_fu]
        roll = [d['roll'] for d in self.parser.angles_fu]
        
        ax.plot(timestamps, pitch, 'r-', label='FU Pitch', alpha=0.9, linewidth=2)
        ax.plot(timestamps, yaw, 'g-', label='FU Yaw', alpha=0.9, linewidth=2)
        ax.plot(timestamps, roll, 'b-', label='FU Roll', alpha=0.9, linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_angles_filters_combined(self, ax):
        """Plot SI and CO filters on same plot for comparison"""
        has_si = bool(self.parser.angles_si)
        has_co = bool(self.parser.angles_co)
        
        if has_si:
            timestamps = [d['timestamp'] for d in self.parser.angles_si]
            pitch = [d['pitch'] for d in self.parser.angles_si]
            yaw = [d['yaw'] for d in self.parser.angles_si]
            roll = [d['roll'] for d in self.parser.angles_si]
            
            ax.plot(timestamps, pitch, 'r--', label='SI Pitch', alpha=0.6)
            ax.plot(timestamps, yaw, 'g--', label='SI Yaw', alpha=0.6)
            ax.plot(timestamps, roll, 'b--', label='SI Roll', alpha=0.6)
        
        if has_co:
            timestamps = [d['timestamp'] for d in self.parser.angles_co]
            pitch = [d['pitch'] for d in self.parser.angles_co]
            yaw = [d['yaw'] for d in self.parser.angles_co]
            roll = [d['roll'] for d in self.parser.angles_co]
            
            ax.plot(timestamps, pitch, 'r:', label='CO Pitch', alpha=0.8)
            ax.plot(timestamps, yaw, 'g:', label='CO Yaw', alpha=0.8)
            ax.plot(timestamps, roll, 'b:', label='CO Roll', alpha=0.8)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        
    def _plot_gyro_bias(self, ax):
        """Plot gyro bias over time"""
        if not self.parser.gyro_bias:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.gyro_bias]
        bias_x = [d['bias_x'] for d in self.parser.gyro_bias]
        bias_y = [d['bias_y'] for d in self.parser.gyro_bias]
        bias_z = [d['bias_z'] for d in self.parser.gyro_bias]
        
        ax.plot(timestamps, bias_x, 'r-', label='Bias X', alpha=0.7)
        ax.plot(timestamps, bias_y, 'g-', label='Bias Y', alpha=0.7)
        ax.plot(timestamps, bias_z, 'b-', label='Bias Z', alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Bias (dps)')
        ax.legend()
        
    def _plot_angle_comparison(self, ax):
        """Plot comparison of pitch angles from all sources"""
        if not any([self.parser.angles, self.parser.angles_di, self.parser.angles_si, 
                   self.parser.angles_co, self.parser.angles_fu]):
            return
            
        # Plot pitch from all sources for comparison
        if self.parser.angles:
            timestamps = [d['timestamp'] for d in self.parser.angles]
            altitude = [d['altitude'] for d in self.parser.angles]
            ax.plot(timestamps, altitude, 'k-', label='ANGLES (Final)', linewidth=2, alpha=0.9)
        
        if self.parser.angles_di:
            timestamps = [d['timestamp'] for d in self.parser.angles_di]
            pitch = [d['pitch'] for d in self.parser.angles_di]
            ax.plot(timestamps, pitch, 'r-', label='ANGLES_DI (MotionDI)', alpha=0.7)
        
        if self.parser.angles_si:
            timestamps = [d['timestamp'] for d in self.parser.angles_si]
            pitch = [d['pitch'] for d in self.parser.angles_si]
            ax.plot(timestamps, pitch, 'g--', label='ANGLES_SI (Simple)', alpha=0.6)
        
        if self.parser.angles_co:
            timestamps = [d['timestamp'] for d in self.parser.angles_co]
            pitch = [d['pitch'] for d in self.parser.angles_co]
            ax.plot(timestamps, pitch, 'b:', label='ANGLES_CO (Compl)', alpha=0.7)
        
        if self.parser.angles_fu:
            timestamps = [d['timestamp'] for d in self.parser.angles_fu]
            pitch = [d['pitch'] for d in self.parser.angles_fu]
            ax.plot(timestamps, pitch, 'm-', label='ANGLES_FU (Fused)', alpha=0.8)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pitch/Altitude (degrees)')
        ax.legend()
        
    def _plot_live_comparison(self, ax):
        """Simplified comparison plot for live mode"""
        self._plot_angle_comparison(ax)
        
    def _plot_analysis_summary(self, ax):
        """Plot analysis summary and insights"""
        ax.axis('off')
        
        # Analyze the data
        summary = self._generate_analysis_summary()
        
        ax.text(0.05, 0.95, 'ENHANCED MOTION DETECTION ANALYSIS', 
                transform=ax.transAxes, fontsize=14, fontweight='bold')
        
        y_pos = 0.85
        for line in summary:
            ax.text(0.05, y_pos, line, transform=ax.transAxes, fontsize=10, 
                   family='monospace')
            y_pos -= 0.04
            if y_pos < 0.05:  # Prevent text from going off the plot
                break
            
    def _generate_analysis_summary(self):
        """Generate enhanced analysis summary"""
        summary = []
        
        # Data availability summary
        summary.append(f"Data Summary:")
        summary.append(f"  Raw Data: {len(self.parser.raw_data)} samples")
        summary.append(f"  ANGLES: {len(self.parser.angles)} samples")
        summary.append(f"  ANGLES_DI: {len(self.parser.angles_di)} samples")
        summary.append(f"  ANGLES_SI: {len(self.parser.angles_si)} samples")
        summary.append(f"  ANGLES_CO: {len(self.parser.angles_co)} samples")
        summary.append(f"  ANGLES_FU: {len(self.parser.angles_fu)} samples")
        summary.append("")
        
        # Analyze final ANGLES output
        if self.parser.angles:
            altitude = [d['altitude'] for d in self.parser.angles]
            azimuth = [d['azimuth'] for d in self.parser.angles]
            zenith = [d['zenith'] for d in self.parser.angles]
            
            if all(a == 0 for a in altitude + azimuth + zenith):
                summary.append("🔴 CRITICAL ISSUE: All ANGLES are 0!")
                summary.append("   This suggests fabsf() is being applied incorrectly")
                summary.append("   Check _calculateAnglesToReference() function")
            else:
                summary.append("✅ ANGLES are varying normally")
                summary.append(f"   Altitude range: [{min(altitude):.2f}, {max(altitude):.2f}]")
                summary.append(f"   Azimuth range: [{min(azimuth):.2f}, {max(azimuth):.2f}]")
                summary.append(f"   Zenith range: [{min(zenith):.2f}, {max(zenith):.2f}]")
        
        summary.append("")
        
        # Analyze MotionDI angles
        if self.parser.angles_di:
            pitch = [d['pitch'] for d in self.parser.angles_di]
            yaw = [d['yaw'] for d in self.parser.angles_di]
            roll = [d['roll'] for d in self.parser.angles_di]
            
            summary.append("MotionDI Analysis (ANGLES_DI):")
            summary.append(f"   Pitch range: [{min(pitch):.2f}, {max(pitch):.2f}]")
            summary.append(f"   Yaw range: [{min(yaw):.2f}, {max(yaw):.2f}]")
            summary.append(f"   Roll range: [{min(roll):.2f}, {max(roll):.2f}]")
        
        # Filter comparison
        if self.parser.angles_si and self.parser.angles_co:
            summary.append("")
            summary.append("Filter Comparison Available:")
            summary.append("   ✅ Both SI and CO filters have data")
            summary.append("   Check plots for filter behavior differences")
        
        # Recommendations
        summary.append("")
        summary.append("RECOMMENDATIONS:")
        if self.parser.angles and all(d['altitude'] == 0 and d['azimuth'] == 0 and d['zenith'] == 0 
                                     for d in self.parser.angles):
            summary.append("1. 🔧 Fix fabsf() issue in _calculateAnglesToReference()")
            summary.append("2. 🔧 Remove absolute value conversions that lose direction")
        else:
            summary.append("1. ✅ Angle calculations appear to be working")
            
        summary.append("3. 📊 Compare filter outputs for consistency")
        summary.append("4. 🔍 Monitor gyro bias stability")
        summary.append("5. 🎯 Verify coordinate system conversions")
        
        return summary

class LiveMotionVisualizer:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.parser = EnhancedMotionDataParser()
        self.parser.enable_live_mode()
        self.console = None
        self.running = False
        
    def start_live_visualization(self):
        """Start live visualization with serial connection"""
        if not LIVE_MODE_AVAILABLE:
            print("Error: Live mode not available. console_reader module not found.")
            return False
            
        # Connect to device
        self.console = MotionConsole(rx_callback=self._handle_serial_data)
        
        if not self.console.connect(self.port, self.baudrate):
            print(f"Failed to connect to {self.port}")
            return False
            
        print(f"Connected to {self.port} at {self.baudrate} baud")
        print("Starting live visualization...")
        
        # Enable raw output and estimator prints
        self.console.send_command("printraw 1")
        time.sleep(0.1)
        # Enable estimator prints (assuming bit 6 = 0x40)
        self.console.send_command("setdebug 127")  # Enable all debug prints
        
        # Create visualizer and start animation
        visualizer = EnhancedMotionVisualizer(self.parser)
        if not visualizer.create_live_visualization():
            return False
        
        self.running = True
        
        # Start animation
        ani = FuncAnimation(visualizer.fig, visualizer.update_live_plots, 
                          interval=100, blit=False, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping live visualization...")
        finally:
            self.running = False
            if self.console:
                self.console.disconnect()
        
        return True
        
    def _handle_serial_data(self, line):
        """Handle incoming serial data"""
        if self.running:
            self.parser.parse_live_data(line)

def main():
    parser = argparse.ArgumentParser(description='Enhanced Motion Detection Debug Visualizer')
    parser.add_argument('log_file', nargs='?', help='Path to the motion detection log file')
    parser.add_argument('--live', action='store_true', help='Enable live visualization mode')
    parser.add_argument('--port', help='Serial port for live mode (e.g., COM15, /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--output', '-o', help='Output PNG file (optional, static mode only)')
    
    args = parser.parse_args()
    
    if args.live:
        # Live mode
        if not args.port:
            print("Error: --port is required for live mode")
            if LIVE_MODE_AVAILABLE:
                console = MotionConsole()
                ports = console.get_available_ports()
                print("Available ports:", ports)
            return
            
        live_viz = LiveMotionVisualizer(args.port, args.baudrate)
        live_viz.start_live_visualization()
        
    else:
        # Static file mode
        if not args.log_file:
            print("Error: log_file is required for static mode")
            return
            
        # Parse the log file
        data_parser = EnhancedMotionDataParser()
        data_parser.parse_log_file(args.log_file)
        
        if not data_parser.raw_data:
            print("No data found in log file!")
            return
        
        # Create visualization
        visualizer = EnhancedMotionVisualizer(data_parser)
        visualizer.create_static_visualization()
        
        # Save to file if requested
        if args.output:
            plt.savefig(args.output, dpi=300, bbox_inches='tight')
            print(f"Visualization saved to: {args.output}")

if __name__ == "__main__":
    main()