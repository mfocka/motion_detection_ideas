#!/usr/bin/env python3
"""
Motion Detection Analysis Tools Runner

This script provides easy access to all motion detection analysis tools.
"""

import sys
import os
import argparse

def show_help():
    """Show available tools and usage"""
    print("""
Motion Detection Analysis Tools
===============================

Available Tools:
    
1. Enhanced Motion Debug Visualizer (motion_debug_visualizer.py)
   - Static analysis of log files with new angle format support
   - Comprehensive visualization of all angle sources
   
   Usage:
     python3 motion_debug_visualizer.py <log_file>
     python3 motion_debug_visualizer.py motion_detection.log --output plot.png

2. Enhanced Motion Visualizer (enhanced_motion_visualizer.py)
   - Advanced static and live analysis
   - Support for live serial data visualization
   
   Usage:
     # Static mode
     python3 enhanced_motion_visualizer.py motion_detection.log
     
     # Live mode
     python3 enhanced_motion_visualizer.py --live --port COM15

3. Live Motion Plotter (live_motion_plotter.py)
   - Dedicated real-time visualization
   - Optimized for continuous monitoring
   
   Usage:
     python3 live_motion_plotter.py --port COM15
     python3 live_motion_plotter.py --port /dev/ttyUSB0 --baudrate 115200

4. Advanced Motion Parser (advanced_motion_parser.py)
   - Legacy tool for detailed CSV export
   
   Usage:
     python3 advanced_motion_parser.py motion_detection.log --csv

5. Test Tools
   - test_enhanced_visualizer.py: Test the new angle format parsing
   
New Angle Format Support:
========================

The tools now support the enhanced angle output format:

- ANGLES: Final output angles (altitude, azimuth, zenith)
- ANGLES_DI: MotionDI euler angles (pitch, yaw, roll)  
- ANGLES_SI: Simple Integration filter (pitch, yaw, roll)
- ANGLES_CO: Complementary filter (pitch, yaw, roll)
- ANGLES_FU: Fused angles (pitch, yaw, roll)

All estimator-related prints are controlled by PRINT_ESTIMATOR flag.

Quick Start:
============

For live analysis:
  python3 live_motion_plotter.py --port <your_port>

For log file analysis:
  python3 motion_debug_visualizer.py <your_log_file>

Dependencies:
=============
  pip3 install --break-system-packages numpy matplotlib pyserial

""")

def main():
    parser = argparse.ArgumentParser(description='Motion Detection Tools Runner')
    parser.add_argument('--list', action='store_true', help='List available tools')
    parser.add_argument('--test', action='store_true', help='Run test suite')
    parser.add_argument('--install-deps', action='store_true', help='Install dependencies')
    
    args = parser.parse_args()
    
    if args.list or len(sys.argv) == 1:
        show_help()
    elif args.test:
        print("Running test suite...")
        os.system("python3 test_enhanced_visualizer.py")
    elif args.install_deps:
        print("Installing dependencies...")
        os.system("pip3 install --break-system-packages numpy matplotlib pyserial")
    else:
        show_help()

if __name__ == "__main__":
    main()