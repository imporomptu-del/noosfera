#!/home/a/seaqr-horizon/bin/python3
"""
Simple Bayer to Color Converter using OpenCV
============================================

Convert RAW8 MP4 files to color MP4 files using OpenCV.

Usage:
    python3 bayer_converter.py input.mp4
"""

import sys
import os
import glob
import cv2
import numpy as np

def convert_file(input_file):
    """Convert a single RAW8 MP4 file to color using OpenCV"""
    
    if not os.path.exists(input_file):
        print(f"❌ File not found: {input_file}")
        return False
    
    # Generate output filename
    base_name = os.path.splitext(input_file)[0]
    output_file = f"{base_name}_color.mp4"
    
    print(f"🎬 Converting: {input_file}")
    print(f"📁 Output: {output_file}")
    
    # Open input video
    cap = cv2.VideoCapture(input_file)
    if not cap.isOpened():
        print(f"❌ Cannot open video file: {input_file}")
        return False
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"📊 Video info: {width}x{height}, {fps:.1f} FPS, {total_frames} frames")
    
    # Define codec and create VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
    
    if not out.isOpened():
        print(f"❌ Cannot create output video file: {output_file}")
        cap.release()
        return False
    
    frame_count = 0
    print("🚀 Starting conversion...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert from Bayer to RGB
        # Assuming the input is grayscale (RAW8/Bayer pattern)
        if len(frame.shape) == 3:
            # If it's already color, convert to grayscale first
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        
        # Apply Bayer demosaicing (RGGB pattern is most common for ZWO cameras)
        color_frame = cv2.cvtColor(gray, cv2.COLOR_BAYER_RG2BGR)
        
        # Write the frame
        out.write(color_frame)
        
        frame_count += 1
        if frame_count % 100 == 0:
            progress = (frame_count / total_frames) * 100
            print(f"⏳ Progress: {frame_count}/{total_frames} frames ({progress:.1f}%)")
    
    # Release everything
    cap.release()
    out.release()
    
    if os.path.exists(output_file):
        file_size = os.path.getsize(output_file) / (1024 * 1024)
        print(f"✅ Success! Output: {output_file} ({file_size:.1f} MB)")
        print(f"📊 Processed {frame_count} frames")
        return True
    else:
        print(f"❌ Conversion failed - output file not created")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bayer_converter.py <input.mp4>")
        print("   or: python3 bayer_converter.py *.mp4")
        return 1
    
    input_pattern = sys.argv[1]
    
    # Handle wildcards
    files = glob.glob(input_pattern)
    if not files:
        files = [input_pattern]
    
    print("🌈 RAW8 to Color Converter")
    print("=" * 30)
    
    success_count = 0
    for file in files:
        if convert_file(file):
            success_count += 1
        print()
    
    print(f"📊 Converted {success_count}/{len(files)} files")
    return 0

if __name__ == "__main__":
    sys.exit(main())
