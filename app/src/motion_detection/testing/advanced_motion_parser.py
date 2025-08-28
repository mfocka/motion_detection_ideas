#!/usr/bin/env python3
"""
Advanced Motion Detection Parser

This tool provides detailed parsing of motion detection logs including:
- Multi-line MotionEstimator debug sections
- MotionEstimator fusion data
- Comprehensive data analysis
- CSV export for further analysis

Usage:
    python3 advanced_motion_parser.py <log_file> [--csv] [--plot]
    
Example:
    python3 advanced_motion_parser.py motion_detection.log --csv --plot
"""

import sys
import re
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
from collections import defaultdict

class AdvancedMotionParser:
    def __init__(self):
        self.raw_data = []
        self.angles = []
        self.gyro_bias = []
        self.motion_estimator_debug = []
        self.motion_estimator_fusion = []
        self.timestamps = []
        self.current_debug_section = None
        self.current_fusion_section = None
        
    def parse_log_file(self, filename):
        """Parse the motion detection log file with multi-line support"""
        print(f"Parsing log file: {filename}")
        
        with open(filename, 'r') as f:
            lines = f.readlines()
            
        # First pass: identify multi-line sections
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if 'MotionEstimator Debug:' in line:
                # Start of debug section
                self.current_debug_section = self._parse_motion_estimator_debug_section(lines, i)
                if self.current_debug_section:
                    self.motion_estimator_debug.append(self.current_debug_section)
                    i += len(self.current_debug_section['lines'])
                else:
                    i += 1
                    
            elif 'MotionEstimator Fusion:' in line:
                # Start of fusion section
                self.current_fusion_section = self._parse_motion_estimator_fusion_section(lines, i)
                if self.current_fusion_section:
                    self.motion_estimator_fusion.append(self.current_fusion_section)
                    i += len(self.current_fusion_section['lines'])
                else:
                    i += 1
                    
            else:
                # Parse single line
                self._parse_single_line(line, i)
                i += 1
        
        print(f"Parsed {len(self.raw_data)} raw data samples")
        print(f"Parsed {len(self.angles)} angle samples")
        print(f"Parsed {len(self.motion_estimator_debug)} MotionEstimator debug entries")
        print(f"Parsed {len(self.motion_estimator_fusion)} MotionEstimator fusion entries")
    
    def _parse_single_line(self, line, line_num):
        """Parse a single log line"""
        if not line:
            return
            
        # Extract timestamp if present
        timestamp_match = re.search(r'\[([\d.]+)s\]', line)
        timestamp = float(timestamp_match.group(1)) if timestamp_match else line_num
        
        # Parse different line types
        if line.startswith('RAW_DATA'):
            self._parse_raw_data(line, timestamp)
        elif line.startswith('ANGLES'):
            self._parse_angles(line, timestamp)
        elif line.startswith('GYRO_BIAS_MDI'):
            self._parse_gyro_bias(line, timestamp)
    
    def _parse_motion_estimator_debug_section(self, lines, start_idx):
        """Parse multi-line MotionEstimator debug section"""
        section = {
            'timestamp': None,
            'lines': [],
            'simple_filter': {'yaw': None, 'pitch': None, 'roll': None},
            'complementary_filter': {'yaw': None, 'pitch': None, 'roll': None},
            'fused_output': {'yaw': None, 'pitch': None, 'roll': None},
            'reference': {'yaw': None, 'pitch': None, 'roll': None},
            'calibration': {'ready': False, 'samples': 0}
        }
        
        # Extract timestamp from first line
        timestamp_match = re.search(r'\[([\d.]+)s\]', lines[start_idx])
        if timestamp_match:
            section['timestamp'] = float(timestamp_match.group(1))
        
        # Parse the section lines
        i = start_idx
        while i < len(lines) and not lines[i].strip().startswith('MotionEstimator Fusion:'):
            line = lines[i].strip()
            section['lines'].append(line)
            
            # Parse specific debug information
            if 'Simple Filter:' in line:
                self._parse_filter_line(line, section['simple_filter'])
            elif 'Complementary Filter:' in line:
                self._parse_filter_line(line, section['complementary_filter'])
            elif 'Fused Output:' in line:
                self._parse_filter_line(line, section['fused_output'])
            elif 'Reference:' in line:
                self._parse_filter_line(line, section['reference'])
            elif 'Calibration:' in line:
                self._parse_calibration_line(line, section['calibration'])
            
            i += 1
        
        return section
    
    def _parse_motion_estimator_fusion_section(self, lines, start_idx):
        """Parse multi-line MotionEstimator fusion section"""
        section = {
            'timestamp': None,
            'lines': [],
            'input_mdi': {'yaw': None, 'pitch': None, 'roll': None},
            'output_fused': {'yaw': None, 'pitch': None, 'roll': None},
            'horizontal_coords': {'azimuth': None, 'altitude': None, 'zenith': None},
            'trust_vector': [None, None, None]
        }
        
        # Extract timestamp from first line
        timestamp_match = re.search(r'\[([\d.]+)s\]', lines[start_idx])
        if timestamp_match:
            section['timestamp'] = float(timestamp_match.group(1))
        
        # Parse the section lines
        i = start_idx
        while i < len(lines) and not lines[i].strip().startswith('Motion:'):
            line = lines[i].strip()
            section['lines'].append(line)
            
            # Parse specific fusion information
            if 'Input (MotionDI):' in line:
                self._parse_filter_line(line, section['input_mdi'])
            elif 'Output (Fused):' in line:
                self._parse_filter_line(line, section['output_fused'])
            elif 'Horizontal Coords:' in line:
                self._parse_horizontal_coords_line(line, section['horizontal_coords'])
            elif 'Trust Vector:' in line:
                self._parse_trust_vector_line(line, section['trust_vector'])
            
            i += 1
        
        return section
    
    def _parse_filter_line(self, line, filter_dict):
        """Parse filter output line like 'Simple Filter: Yaw=0.069 deg, Pitch=0.731 deg, Roll=10.938 deg'"""
        yaw_match = re.search(r'Yaw=([-\d.]+)', line)
        pitch_match = re.search(r'Pitch=([-\d.]+)', line)
        roll_match = re.search(r'Roll=([-\d.]+)', line)
        
        if yaw_match:
            filter_dict['yaw'] = float(yaw_match.group(1))
        if pitch_match:
            filter_dict['pitch'] = float(pitch_match.group(1))
        if roll_match:
            filter_dict['roll'] = float(roll_match.group(1))
    
    def _parse_calibration_line(self, line, cal_dict):
        """Parse calibration line like 'Calibration: Ready (1040 samples)'"""
        if 'Ready' in line:
            cal_dict['ready'] = True
        samples_match = re.search(r'\((\d+) samples\)', line)
        if samples_match:
            cal_dict['samples'] = int(samples_match.group(1))
    
    def _parse_horizontal_coords_line(self, line, coords_dict):
        """Parse horizontal coordinates line like 'Horizontal Coords: Azimuth=0 deg, Altitude=0 deg, Zenith=0 deg'"""
        azimuth_match = re.search(r'Azimuth=([-\d.]+)', line)
        altitude_match = re.search(r'Altitude=([-\d.]+)', line)
        zenith_match = re.search(r'Zenith=([-\d.]+)', line)
        
        if azimuth_match:
            coords_dict['azimuth'] = float(azimuth_match.group(1))
        if altitude_match:
            coords_dict['altitude'] = float(altitude_match.group(1))
        if zenith_match:
            coords_dict['zenith'] = float(zenith_match.group(1))
    
    def _parse_trust_vector_line(self, line, trust_vector):
        """Parse trust vector line like 'Trust Vector: [0.300, 0.899, 0.500]'"""
        vector_match = re.search(r'\[([-\d., ]+)\]', line)
        if vector_match:
            values = vector_match.group(1).split(',')
            for i, val in enumerate(values):
                if i < 3:
                    try:
                        trust_vector[i] = float(val.strip())
                    except ValueError:
                        pass
    
    def _parse_raw_data(self, line, timestamp):
        """Parse RAW_DATA line"""
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
        """Parse ANGLES line"""
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
        """Parse GYRO_BIAS_MDI line"""
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
    
    def export_to_csv(self, filename):
        """Export parsed data to CSV files"""
        # Export raw data
        if self.raw_data:
            with open(f"{filename}_raw_data.csv", 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'accel_x_mg', 'accel_y_mg', 'accel_z_mg', 'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps'])
                for data in self.raw_data:
                    writer.writerow([
                        data['timestamp'],
                        data['accel_mg'][0], data['accel_mg'][1], data['accel_mg'][2],
                        data['gyro_dps'][0], data['gyro_dps'][1], data['gyro_dps'][2]
                    ])
            print(f"Raw data exported to {filename}_raw_data.csv")
        
        # Export angles
        if self.angles:
            with open(f"{filename}_angles.csv", 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'altitude', 'azimuth', 'zenith', 'state'])
                for data in self.angles:
                    writer.writerow([
                        data['timestamp'],
                        data['altitude'], data['azimuth'], data['zenith'], data['state']
                    ])
            print(f"Angles exported to {filename}_angles.csv")
        
        # Export MotionEstimator debug
        if self.motion_estimator_debug:
            with open(f"{filename}_motion_estimator_debug.csv", 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'simple_yaw', 'simple_pitch', 'simple_roll',
                    'complementary_yaw', 'complementary_pitch', 'complementary_roll',
                    'fused_yaw', 'fused_pitch', 'fused_roll',
                    'ref_yaw', 'ref_pitch', 'ref_roll',
                    'calibration_ready', 'calibration_samples'
                ])
                for data in self.motion_estimator_debug:
                    writer.writerow([
                        data['timestamp'],
                        data['simple_filter']['yaw'], data['simple_filter']['pitch'], data['simple_filter']['roll'],
                        data['complementary_filter']['yaw'], data['complementary_filter']['pitch'], data['complementary_filter']['roll'],
                        data['fused_output']['yaw'], data['fused_output']['pitch'], data['fused_output']['roll'],
                        data['reference']['yaw'], data['reference']['pitch'], data['reference']['roll'],
                        data['calibration']['ready'], data['calibration']['samples']
                    ])
            print(f"MotionEstimator debug exported to {filename}_motion_estimator_debug.csv")
        
        # Export MotionEstimator fusion
        if self.motion_estimator_fusion:
            with open(f"{filename}_motion_estimator_fusion.csv", 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'input_yaw', 'input_pitch', 'input_roll',
                    'output_yaw', 'output_pitch', 'output_roll',
                    'azimuth', 'altitude', 'zenith',
                    'trust_x', 'trust_y', 'trust_z'
                ])
                for data in self.motion_estimator_fusion:
                    writer.writerow([
                        data['timestamp'],
                        data['input_mdi']['yaw'], data['input_mdi']['pitch'], data['input_mdi']['roll'],
                        data['output_fused']['yaw'], data['output_fused']['pitch'], data['output_fused']['roll'],
                        data['horizontal_coords']['azimuth'], data['horizontal_coords']['altitude'], data['horizontal_coords']['zenith'],
                        data['trust_vector'][0], data['trust_vector'][1], data['trust_vector'][2]
                    ])
            print(f"MotionEstimator fusion exported to {filename}_motion_estimator_fusion.csv")
    
    def generate_analysis_report(self):
        """Generate comprehensive analysis report"""
        report = []
        report.append("=" * 80)
        report.append("MOTION DETECTION COMPREHENSIVE ANALYSIS REPORT")
        report.append("=" * 80)
        report.append("")
        
        # Data summary
        report.append("DATA SUMMARY:")
        report.append(f"  Raw Data Samples: {len(self.raw_data)}")
        report.append(f"  Angle Samples: {len(self.angles)}")
        report.append(f"  MotionEstimator Debug Entries: {len(self.motion_estimator_debug)}")
        report.append(f"  MotionEstimator Fusion Entries: {len(self.motion_estimator_fusion)}")
        report.append("")
        
        # Raw data analysis
        if self.raw_data:
            report.append("RAW SENSOR DATA ANALYSIS:")
            accel_x = [d['accel_mg'][0] for d in self.raw_data]
            accel_y = [d['accel_mg'][1] for d in self.raw_data]
            accel_z = [d['accel_mg'][2] for d in self.raw_data]
            gyro_x = [d['gyro_dps'][0] for d in self.raw_data]
            gyro_y = [d['gyro_dps'][1] for d in self.raw_data]
            gyro_z = [d['gyro_dps'][2] for d in self.raw_data]
            
            report.append(f"  Accelerometer X: min={min(accel_x):.2f}, max={max(accel_x):.2f}, mean={np.mean(accel_x):.2f}, std={np.std(accel_x):.2f}")
            report.append(f"  Accelerometer Y: min={min(accel_y):.2f}, max={max(accel_y):.2f}, mean={np.mean(accel_y):.2f}, std={np.std(accel_y):.2f}")
            report.append(f"  Accelerometer Z: min={min(accel_z):.2f}, max={max(accel_z):.2f}, mean={np.mean(accel_z):.2f}, std={np.std(accel_z):.2f}")
            report.append(f"  Gyroscope X: min={min(gyro_x):.2f}, max={max(gyro_x):.2f}, mean={np.mean(gyro_x):.2f}, std={np.std(gyro_x):.2f}")
            report.append(f"  Gyroscope Y: min={min(gyro_y):.2f}, max={max(gyro_y):.2f}, mean={np.mean(gyro_y):.2f}, std={np.std(gyro_y):.2f}")
            report.append(f"  Gyroscope Z: min={min(gyro_z):.2f}, max={max(gyro_z):.2f}, mean={np.mean(gyro_z):.2f}, std={np.std(gyro_z):.2f}")
            report.append("")
        
        # Angles analysis
        if self.angles:
            report.append("ANGLES OUTPUT ANALYSIS:")
            altitude = [d['altitude'] for d in self.angles]
            azimuth = [d['azimuth'] for d in self.angles]
            zenith = [d['zenith'] for d in self.angles]
            
            # Check for the zero angles issue
            if all(a == 0 for a in altitude + azimuth + zenith):
                report.append("  ⚠️  CRITICAL ISSUE DETECTED: All angles are 0!")
                report.append("     This indicates a problem in the angle calculation pipeline")
                report.append("     Likely causes:")
                report.append("     1. _calculateAnglesToReference() applying fabsf() incorrectly")
                report.append("     2. Reference quaternion not set properly")
                report.append("     3. Coordinate system conversion issues")
            else:
                report.append("  ✅ Angles are varying normally")
                
            report.append(f"  Altitude: min={min(altitude):.2f}, max={max(altitude):.2f}, mean={np.mean(altitude):.2f}, std={np.std(altitude):.2f}")
            report.append(f"  Azimuth: min={min(azimuth):.2f}, max={max(azimuth):.2f}, mean={np.mean(azimuth):.2f}, std={np.std(azimuth):.2f}")
            report.append(f"  Zenith: min={min(zenith):.2f}, max={max(zenith):.2f}, mean={np.mean(zenith):.2f}, std={np.std(zenith):.2f}")
            report.append("")
        
        # MotionEstimator analysis
        if self.motion_estimator_debug:
            report.append("MOTIONESTIMATOR DEBUG ANALYSIS:")
            simple_yaw = [d['simple_filter']['yaw'] for d in self.motion_estimator_debug if d['simple_filter']['yaw'] is not None]
            simple_pitch = [d['simple_filter']['pitch'] for d in self.motion_estimator_debug if d['simple_filter']['pitch'] is not None]
            simple_roll = [d['simple_filter']['roll'] for d in self.motion_estimator_debug if d['simple_filter']['roll'] is not None]
            
            if simple_yaw:
                report.append(f"  Simple Filter Yaw: min={min(simple_yaw):.2f}, max={max(simple_yaw):.2f}, mean={np.mean(simple_yaw):.2f}")
            if simple_pitch:
                report.append(f"  Simple Filter Pitch: min={min(simple_pitch):.2f}, max={max(simple_pitch):.2f}, mean={np.mean(simple_pitch):.2f}")
            if simple_roll:
                report.append(f"  Simple Filter Roll: min={min(simple_roll):.2f}, max={max(simple_roll):.2f}, mean={np.mean(simple_roll):.2f}")
            report.append("")
        
        # Fusion analysis
        if self.motion_estimator_fusion:
            report.append("MOTIONESTIMATOR FUSION ANALYSIS:")
            input_yaw = [d['input_mdi']['yaw'] for d in self.motion_estimator_fusion if d['input_mdi']['yaw'] is not None]
            output_yaw = [d['output_fused']['yaw'] for d in self.motion_estimator_fusion if d['output_fused']['yaw'] is not None]
            
            if input_yaw and output_yaw:
                report.append(f"  Input (MotionDI) Yaw: min={min(input_yaw):.2f}, max={max(input_yaw):.2f}, mean={np.mean(input_yaw):.2f}")
                report.append(f"  Output (Fused) Yaw: min={min(output_yaw):.2f}, max={max(output_yaw):.2f}, mean={np.mean(output_yaw):.2f}")
                
                # Check for discrepancies
                if len(input_yaw) == len(output_yaw):
                    differences = [abs(i - o) for i, o in zip(input_yaw, output_yaw)]
                    max_diff = max(differences)
                    mean_diff = np.mean(differences)
                    report.append(f"  Max difference between input and output: {max_diff:.2f}°")
                    report.append(f"  Mean difference between input and output: {mean_diff:.2f}°")
            report.append("")
        
        # Recommendations
        report.append("RECOMMENDATIONS:")
        report.append("1. Check _calculateAnglesToReference() function for incorrect fabsf() usage")
        report.append("2. Verify reference quaternion initialization and updates")
        report.append("3. Check coordinate system conversions (WDS->ENU)")
        report.append("4. Review MotionEstimator fusion logic and trust vector values")
        report.append("5. Ensure proper calibration sequence completion")
        report.append("")
        
        report.append("=" * 80)
        
        return report

def main():
    parser = argparse.ArgumentParser(description='Advanced Motion Detection Parser')
    parser.add_argument('log_file', help='Path to the motion detection log file')
    parser.add_argument('--csv', action='store_true', help='Export data to CSV files')
    parser.add_argument('--plot', action='store_true', help='Generate plots')
    parser.add_argument('--output', '-o', help='Output base filename for CSV export')
    
    args = parser.parse_args()
    
    # Parse the log file
    data_parser = AdvancedMotionParser()
    data_parser.parse_log_file(args.log_file)
    
    if not data_parser.raw_data:
        print("No data found in log file!")
        return
    
    # Generate analysis report
    report = data_parser.generate_analysis_report()
    for line in report:
        print(line)
    
    # Export to CSV if requested
    if args.csv:
        output_base = args.output or 'motion_analysis'
        data_parser.export_to_csv(output_base)
    
    # Generate plots if requested
    if args.plot:
        print("Plot generation not yet implemented in this version")

if __name__ == "__main__":
    main()