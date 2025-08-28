#!/usr/bin/env python3
"""
Test script for the enhanced motion visualizer

Creates sample data in the new format and tests the parsing/visualization.
"""

import os
import sys
import tempfile

def create_sample_log():
    """Create a sample log file with the new angle format"""
    sample_data = """[0.000s] MotionDetection: Starting...
RAW_DATA,0,1000.2,50.3,-950.8,0.1,-0.2,0.3
ANGLES,0,1.5,-2.3,0.8,MONITORING
ANGLES_DI,0,1.2,-2.1,0.9
ANGLES_SI,0,1.4,-2.2,0.7
ANGLES_CO,0,1.3,-2.0,0.8
ANGLES_FU,0,1.35,-2.15,0.75
GYRO_BIAS_MDI,0,-0.411,0.587,0.774

[0.010s] Next sample...
RAW_DATA,10000,1001.1,51.2,-949.5,0.15,-0.18,0.32
ANGLES,10000,1.6,-2.4,0.9,MONITORING
ANGLES_DI,10000,1.3,-2.2,1.0
ANGLES_SI,10000,1.5,-2.3,0.8
ANGLES_CO,10000,1.4,-2.1,0.9
ANGLES_FU,10000,1.45,-2.25,0.85
GYRO_BIAS_MDI,10000,-0.412,0.586,0.773

[0.020s] Third sample...
RAW_DATA,20000,999.8,49.8,-951.2,0.12,-0.22,0.28
ANGLES,20000,1.4,-2.2,0.7,MONITORING
ANGLES_DI,20000,1.1,-2.0,0.8
ANGLES_SI,20000,1.3,-2.1,0.6
ANGLES_CO,20000,1.2,-1.9,0.7
ANGLES_FU,20000,1.25,-2.05,0.65
GYRO_BIAS_MDI,20000,-0.410,0.588,0.775
"""
    
    # Create temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False) as f:
        f.write(sample_data)
        return f.name

def test_enhanced_visualizer():
    """Test the enhanced motion visualizer"""
    print("Testing Enhanced Motion Visualizer...")
    
    # Create sample data
    log_file = create_sample_log()
    print(f"Created sample log: {log_file}")
    
    try:
        # Import and test the enhanced visualizer
        from motion_debug_visualizer import MotionDataParser, MotionVisualizer
        
        # Parse the sample data
        parser = MotionDataParser()
        parser.parse_log_file(log_file)
        
        # Verify parsing worked
        print("\\nParsing Results:")
        print(f"  Raw data: {len(parser.raw_data)} samples")
        print(f"  ANGLES: {len(parser.angles)} samples")
        print(f"  ANGLES_DI: {len(parser.angles_di)} samples") 
        print(f"  ANGLES_SI: {len(parser.angles_si)} samples")
        print(f"  ANGLES_CO: {len(parser.angles_co)} samples")
        print(f"  ANGLES_FU: {len(parser.angles_fu)} samples")
        print(f"  Gyro bias: {len(parser.gyro_bias)} samples")
        
        # Test visualization creation
        visualizer = MotionVisualizer(parser)
        print("\\nCreating visualization...")
        visualizer.create_visualization()
        
        print("✅ Test completed successfully!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        try:
            os.unlink(log_file)
        except:
            pass

if __name__ == "__main__":
    test_enhanced_visualizer()