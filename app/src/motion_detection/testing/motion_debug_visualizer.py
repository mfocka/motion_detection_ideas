#!/usr/bin/env python3
"""
Motion Detection Debug Visualizer

This tool helps debug motion detection issues by visualizing:
1. Raw sensor data (converted to ENU)
2. MotionDI output angles
3. Simple filter output
4. Complementary filter output  
5. Fused output (combination of MDI and MotionEstimator)
6. Reference angles for both systems

Usage:
    python3 motion_debug_visualizer.py <log_file>
    
Example:
    python3 motion_debug_visualizer.py motion_detection.log
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
from datetime import datetime

class MotionDataParser:
    def __init__(self):
        self.raw_data = []
        self.angles = []           # Final output angles (altitude, azimuth, zenith)
        self.angles_di = []        # MotionDI euler angles (pitch, yaw, roll)
        self.angles_si = []        # Simple Integration filter (pitch, yaw, roll)
        self.angles_co = []        # Complementary filter (pitch, yaw, roll)
        self.angles_fu = []        # Fused angles (pitch, yaw, roll)
        self.gyro_bias = []
        self.motion_estimator_debug = []
        self.motion_estimator_fusion = []
        self.timestamps = []
        
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
    
    def _parse_line(self, line, line_num):
        """Parse a single log line"""
        if not line:
            return
            
        # Extract timestamp if present
        timestamp_match = re.search(r'\[([\d.]+)s\]', line)
        timestamp = float(timestamp_match.group(1)) if timestamp_match else line_num
        
        # Parse RAW_DATA lines
        if line.startswith('RAW_DATA'):
            self._parse_raw_data(line, timestamp)
            
        # Parse ANGLES lines (final output)
        elif line.startswith('ANGLES,'):
            self._parse_angles(line, timestamp)
        
        # Parse new angle format lines
        elif line.startswith('ANGLES_DI'):
            self._parse_angles_di(line, timestamp)
        elif line.startswith('ANGLES_SI'):
            self._parse_angles_si(line, timestamp)
        elif line.startswith('ANGLES_CO'):
            self._parse_angles_co(line, timestamp)
        elif line.startswith('ANGLES_FU'):
            self._parse_angles_fu(line, timestamp)
            
        # Parse GYRO_BIAS_MDI lines
        elif line.startswith('GYRO_BIAS_MDI'):
            self._parse_gyro_bias(line, timestamp)
            
        # Parse legacy MotionEstimator Debug lines (for backward compatibility)
        elif 'MotionEstimator Debug:' in line:
            self._parse_motion_estimator_debug(line, timestamp)
            
        # Parse legacy MotionEstimator Fusion lines (for backward compatibility)
        elif 'MotionEstimator Fusion:' in line:
            self._parse_motion_estimator_fusion(line, timestamp)
    
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
    
    def _parse_motion_estimator_debug(self, line, timestamp):
        """Parse MotionEstimator Debug section (legacy support)"""
        # This is a multi-line section, we'll need to handle it specially
        pass
    
    def _parse_motion_estimator_fusion(self, line, timestamp):
        """Parse MotionEstimator Fusion section (legacy support)"""
        # This is a multi-line section, we'll need to handle it specially
        pass

class MotionVisualizer:
    def __init__(self, parser):
        self.parser = parser
        
    def create_visualization(self):
        """Create comprehensive visualization of motion data"""
        if not self.parser.raw_data:
            print("No data to visualize!")
            return
            
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 14))
        gs = GridSpec(4, 3, figure=fig)
        
        # Plot 1: Raw accelerometer data
        ax1 = fig.add_subplot(gs[0, 0])
        self._plot_raw_accelerometer(ax1)
        
        # Plot 2: Raw gyroscope data  
        ax2 = fig.add_subplot(gs[0, 1])
        self._plot_raw_gyroscope(ax2)
        
        # Plot 3: Final ANGLES output
        ax3 = fig.add_subplot(gs[0, 2])
        self._plot_angles_output(ax3)
        
        # Plot 4: MotionDI angles (ANGLES_DI)
        ax4 = fig.add_subplot(gs[1, 0])
        self._plot_angles_di(ax4)
        
        # Plot 5: Filter angles (ANGLES_SI + ANGLES_CO)
        ax5 = fig.add_subplot(gs[1, 1])
        self._plot_filter_angles(ax5)
        
        # Plot 6: Fused angles (ANGLES_FU)
        ax6 = fig.add_subplot(gs[1, 2])
        self._plot_angles_fu(ax6)
        
        # Plot 7: Angle comparison
        ax7 = fig.add_subplot(gs[2, 0:2])
        self._plot_angle_comparison(ax7)
        
        # Plot 8: Gyro bias over time
        ax8 = fig.add_subplot(gs[2, 2])
        self._plot_gyro_bias(ax8)
        
        # Plot 9: Analysis summary
        ax9 = fig.add_subplot(gs[3, :])
        self._plot_analysis_summary(ax9)
        
        plt.tight_layout()
        plt.show()
        
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
        ax.set_title('Raw Accelerometer Data (mg)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (mg)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
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
        ax.set_title('Raw Gyroscope Data (dps)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Rate (dps)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_angles_output(self, ax):
        """Plot ANGLES output"""
        if not self.parser.angles:
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles]
        altitude = [d['altitude'] for d in self.parser.angles]
        azimuth = [d['azimuth'] for d in self.parser.angles]
        zenith = [d['zenith'] for d in self.parser.angles]
        
        ax.plot(timestamps, altitude, 'r-', label='Altitude', alpha=0.7)
        ax.plot(timestamps, azimuth, 'g-', label='Azimuth', alpha=0.7)
        ax.plot(timestamps, zenith, 'b-', label='Zenith', alpha=0.7)
        ax.set_title('ANGLES Output (Degrees)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
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
        ax.set_title('Gyro Bias Over Time (dps)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Bias (dps)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_angles_di(self, ax):
        """Plot MotionDI euler angles (ANGLES_DI)"""
        if not self.parser.angles_di:
            ax.text(0.5, 0.5, 'No ANGLES_DI data available', 
                   transform=ax.transAxes, ha='center', va='center')
            ax.set_title('ANGLES_DI - MotionDI Euler')
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_di]
        pitch = [d['pitch'] for d in self.parser.angles_di]
        yaw = [d['yaw'] for d in self.parser.angles_di]
        roll = [d['roll'] for d in self.parser.angles_di]
        
        ax.plot(timestamps, pitch, 'r-', label='Pitch', alpha=0.8)
        ax.plot(timestamps, yaw, 'g-', label='Yaw', alpha=0.8)
        ax.plot(timestamps, roll, 'b-', label='Roll', alpha=0.8)
        ax.set_title('ANGLES_DI - MotionDI Euler')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_filter_angles(self, ax):
        """Plot filter angles (SI and CO) on same plot"""
        has_data = False
        
        if self.parser.angles_si:
            timestamps = [d['timestamp'] for d in self.parser.angles_si]
            pitch = [d['pitch'] for d in self.parser.angles_si]
            yaw = [d['yaw'] for d in self.parser.angles_si]
            roll = [d['roll'] for d in self.parser.angles_si]
            
            ax.plot(timestamps, pitch, 'r--', label='SI Pitch', alpha=0.7)
            ax.plot(timestamps, yaw, 'g--', label='SI Yaw', alpha=0.7)
            ax.plot(timestamps, roll, 'b--', label='SI Roll', alpha=0.7)
            has_data = True
        
        if self.parser.angles_co:
            timestamps = [d['timestamp'] for d in self.parser.angles_co]
            pitch = [d['pitch'] for d in self.parser.angles_co]
            yaw = [d['yaw'] for d in self.parser.angles_co]
            roll = [d['roll'] for d in self.parser.angles_co]
            
            ax.plot(timestamps, pitch, 'r:', label='CO Pitch', alpha=0.8, linewidth=2)
            ax.plot(timestamps, yaw, 'g:', label='CO Yaw', alpha=0.8, linewidth=2)
            ax.plot(timestamps, roll, 'b:', label='CO Roll', alpha=0.8, linewidth=2)
            has_data = True
        
        if not has_data:
            ax.text(0.5, 0.5, 'No filter angle data available', 
                   transform=ax.transAxes, ha='center', va='center')
        
        ax.set_title('Filter Angles (SI=dashed, CO=dotted)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_angles_fu(self, ax):
        """Plot fused angles (ANGLES_FU)"""
        if not self.parser.angles_fu:
            ax.text(0.5, 0.5, 'No ANGLES_FU data available', 
                   transform=ax.transAxes, ha='center', va='center')
            ax.set_title('ANGLES_FU - Fused Output')
            return
            
        timestamps = [d['timestamp'] for d in self.parser.angles_fu]
        pitch = [d['pitch'] for d in self.parser.angles_fu]
        yaw = [d['yaw'] for d in self.parser.angles_fu]
        roll = [d['roll'] for d in self.parser.angles_fu]
        
        ax.plot(timestamps, pitch, 'r-', label='Fused Pitch', alpha=0.9, linewidth=2)
        ax.plot(timestamps, yaw, 'g-', label='Fused Yaw', alpha=0.9, linewidth=2)
        ax.plot(timestamps, roll, 'b-', label='Fused Roll', alpha=0.9, linewidth=2)
        ax.set_title('ANGLES_FU - Fused Output')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_angle_comparison(self, ax):
        """Plot comparison of pitch from all sources"""
        has_data = False
        
        # Plot final output
        if self.parser.angles:
            timestamps = [d['timestamp'] for d in self.parser.angles]
            altitude = [d['altitude'] for d in self.parser.angles]
            ax.plot(timestamps, altitude, 'k-', label='ANGLES (Final)', linewidth=3, alpha=0.9)
            has_data = True
        
        # Plot MotionDI
        if self.parser.angles_di:
            timestamps = [d['timestamp'] for d in self.parser.angles_di]
            pitch = [d['pitch'] for d in self.parser.angles_di]
            ax.plot(timestamps, pitch, 'r-', label='DI Pitch', alpha=0.7)
            has_data = True
        
        # Plot filters
        if self.parser.angles_si:
            timestamps = [d['timestamp'] for d in self.parser.angles_si]
            pitch = [d['pitch'] for d in self.parser.angles_si]
            ax.plot(timestamps, pitch, 'g--', label='SI Pitch', alpha=0.6)
            has_data = True
        
        if self.parser.angles_co:
            timestamps = [d['timestamp'] for d in self.parser.angles_co]
            pitch = [d['pitch'] for d in self.parser.angles_co]
            ax.plot(timestamps, pitch, 'b:', label='CO Pitch', alpha=0.7)
            has_data = True
        
        if self.parser.angles_fu:
            timestamps = [d['timestamp'] for d in self.parser.angles_fu]
            pitch = [d['pitch'] for d in self.parser.angles_fu]
            ax.plot(timestamps, pitch, 'm-', label='FU Pitch', alpha=0.8)
            has_data = True
        
        if not has_data:
            ax.text(0.5, 0.5, 'No angle data available for comparison', 
                   transform=ax.transAxes, ha='center', va='center')
        
        ax.set_title('Pitch/Altitude Angle Comparison - All Sources')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pitch/Altitude (degrees)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _parse_motion_estimator_debug(self, line, timestamp):
        """Parse MotionEstimator Debug section (legacy support)"""
        # This is a multi-line section, we'll need to handle it specially
        pass
    
    def _parse_motion_estimator_fusion(self, line, timestamp):
        """Parse MotionEstimator Fusion section (legacy support)"""
        # This is a multi-line section, we'll need to handle it specially
        pass
        
    def _plot_analysis_summary(self, ax):
        """Plot analysis summary and insights"""
        ax.axis('off')
        
        # Analyze the data
        summary = self._generate_analysis_summary()
        
        ax.text(0.05, 0.95, 'MOTION DETECTION ANALYSIS SUMMARY', 
                transform=ax.transAxes, fontsize=14, fontweight='bold')
        
        y_pos = 0.85
        for line in summary:
            ax.text(0.05, y_pos, line, transform=ax.transAxes, fontsize=10, 
                   family='monospace')
            y_pos -= 0.05
            
    def _generate_analysis_summary(self):
        """Generate analysis summary"""
        summary = []
        
        if self.parser.raw_data:
            summary.append(f"Raw Data Samples: {len(self.parser.raw_data)}")
            
            # Analyze accelerometer data
            accel_x = [d['accel_mg'][0] for d in self.parser.raw_data]
            accel_y = [d['accel_mg'][1] for d in self.parser.raw_data]
            accel_z = [d['accel_mg'][2] for d in self.parser.raw_data]
            
            summary.append(f"Accel X: min={min(accel_x):.2f}, max={max(accel_x):.2f}, mean={np.mean(accel_x):.2f}")
            summary.append(f"Accel Y: min={min(accel_y):.2f}, max={max(accel_y):.2f}, mean={np.mean(accel_y):.2f}")
            summary.append(f"Accel Z: min={min(accel_z):.2f}, max={max(accel_z):.2f}, mean={np.mean(accel_z):.2f}")
            
            # Analyze gyroscope data
            gyro_x = [d['gyro_dps'][0] for d in self.parser.raw_data]
            gyro_y = [d['gyro_dps'][1] for d in self.parser.raw_data]
            gyro_z = [d['gyro_dps'][2] for d in self.parser.raw_data]
            
            summary.append(f"Gyro X: min={min(gyro_x):.2f}, max={max(gyro_x):.2f}, mean={np.mean(gyro_x):.2f}")
            summary.append(f"Gyro Y: min={min(gyro_y):.2f}, max={max(gyro_y):.2f}, mean={np.mean(gyro_y):.2f}")
            summary.append(f"Gyro Z: min={min(gyro_z):.2f}, max={max(gyro_z):.2f}, mean={np.mean(gyro_z):.2f}")
        
        # Analyze final ANGLES output
        if self.parser.angles:
            summary.append(f"ANGLES (Final): {len(self.parser.angles)} samples")
            
            altitude = [d['altitude'] for d in self.parser.angles]
            azimuth = [d['azimuth'] for d in self.parser.angles]
            zenith = [d['zenith'] for d in self.parser.angles]
            
            if all(a == 0 for a in altitude + azimuth + zenith):
                summary.append("🔴 CRITICAL: All ANGLES are 0!")
                summary.append("   Check _calculateAnglesToReference() fabsf() issue")
            else:
                summary.append("✅ ANGLES varying normally")
                summary.append(f"   Altitude: [{min(altitude):.2f}, {max(altitude):.2f}]")
                summary.append(f"   Azimuth: [{min(azimuth):.2f}, {max(azimuth):.2f}]")
                summary.append(f"   Zenith: [{min(zenith):.2f}, {max(zenith):.2f}]")
        
        # Analyze MotionDI angles
        if self.parser.angles_di:
            summary.append(f"ANGLES_DI (MotionDI): {len(self.parser.angles_di)} samples")
            pitch = [d['pitch'] for d in self.parser.angles_di]
            yaw = [d['yaw'] for d in self.parser.angles_di]
            roll = [d['roll'] for d in self.parser.angles_di]
            summary.append(f"   Pitch: [{min(pitch):.2f}, {max(pitch):.2f}]")
            summary.append(f"   Yaw: [{min(yaw):.2f}, {max(yaw):.2f}]")
            summary.append(f"   Roll: [{min(roll):.2f}, {max(roll):.2f}]")
        
        # Analyze filter outputs
        if self.parser.angles_si:
            summary.append(f"ANGLES_SI (Simple): {len(self.parser.angles_si)} samples")
        if self.parser.angles_co:
            summary.append(f"ANGLES_CO (Complementary): {len(self.parser.angles_co)} samples")
        if self.parser.angles_fu:
            summary.append(f"ANGLES_FU (Fused): {len(self.parser.angles_fu)} samples")
        
        if self.parser.gyro_bias:
            summary.append(f"Gyro Bias Samples: {len(self.parser.gyro_bias)}")
            
            # Check if bias is stable
            bias_x = [d['bias_x'] for d in self.parser.gyro_bias]
            bias_y = [d['bias_y'] for d in self.parser.gyro_bias]
            bias_z = [d['bias_z'] for d in self.parser.gyro_bias]
            
            bias_std_x = np.std(bias_x)
            bias_std_y = np.std(bias_y)
            bias_std_z = np.std(bias_z)
            
            summary.append(f"Bias Stability (std): X={bias_std_x:.3f}, Y={bias_std_y:.3f}, Z={bias_std_z:.3f}")
            
            if bias_std_x < 0.01 and bias_std_y < 0.01 and bias_std_z < 0.01:
                summary.append("✅ Gyro bias is very stable")
            elif bias_std_x < 0.1 and bias_std_y < 0.1 and bias_std_z < 0.1:
                summary.append("✅ Gyro bias is stable")
            else:
                summary.append("⚠️  Gyro bias shows some variation")
        
        # Add recommendations
        summary.append("")
        summary.append("RECOMMENDATIONS:")
        if self.parser.angles and all(d['altitude'] == 0 and d['azimuth'] == 0 and d['zenith'] == 0 
                                     for d in self.parser.angles):
            summary.append("1. 🔧 URGENT: Fix fabsf() in _calculateAnglesToReference()")
            summary.append("2. 🔧 Remove absolute value that loses directional info")
        else:
            summary.append("1. ✅ Final angle calculations working")
        
        summary.append("3. 📊 Compare all angle sources for consistency")
        summary.append("4. 🔍 Monitor filter performance (SI vs CO vs FU)")
        summary.append("5. 🎯 Verify coordinate transformations")
        summary.append("6. 📈 Check gyro bias stability over time")
        
        return summary

def main():
    parser = argparse.ArgumentParser(description='Motion Detection Debug Visualizer')
    parser.add_argument('log_file', help='Path to the motion detection log file')
    parser.add_argument('--output', '-o', help='Output PNG file (optional)')
    
    args = parser.parse_args()
    
    # Parse the log file
    data_parser = MotionDataParser()
    data_parser.parse_log_file(args.log_file)
    
    if not data_parser.raw_data:
        print("No data found in log file!")
        return
    
    # Create visualization
    visualizer = MotionVisualizer(data_parser)
    visualizer.create_visualization()
    
    # Save to file if requested
    if args.output:
        plt.savefig(args.output, dpi=300, bbox_inches='tight')
        print(f"Visualization saved to: {args.output}")

if __name__ == "__main__":
    main()