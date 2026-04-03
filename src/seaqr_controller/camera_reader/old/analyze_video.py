#!/usr/bin/env python3
"""
Analyze MP4 files to show FPS, duration, and frame count
"""
import os
import sys
import cv2
import glob
from pathlib import Path

def analyze_video(video_path):
    """Analyze a single video file"""
    if not os.path.exists(video_path):
        print(f"❌ File not found: {video_path}")
        return None
    
    # Open video with OpenCV
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"❌ Cannot open video: {video_path}")
        return None
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Calculate duration
    duration_seconds = frame_count / fps if fps > 0 else 0
    duration_minutes = duration_seconds / 60
    
    # Get file size
    file_size_bytes = os.path.getsize(video_path)
    file_size_mb = file_size_bytes / (1024 * 1024)
    
    cap.release()
    
    return {
        'filename': os.path.basename(video_path),
        'fps': fps,
        'frame_count': frame_count,
        'duration_seconds': duration_seconds,
        'duration_minutes': duration_minutes,
        'width': width,
        'height': height,
        'file_size_mb': file_size_mb
    }

def analyze_directory(directory_path):
    """Analyze all MP4 files in a directory"""
    mp4_files = glob.glob(os.path.join(directory_path, "*.mp4"))
    
    if not mp4_files:
        print(f"❌ No MP4 files found in: {directory_path}")
        return
    
    mp4_files.sort()  # Sort alphabetically
    
    print(f"📁 Analyzing MP4 files in: {directory_path}")
    print("=" * 80)
    
    total_duration = 0
    total_frames = 0
    
    for video_path in mp4_files:
        result = analyze_video(video_path)
        if result:
            print(f"📹 {result['filename']}")
            print(f"   🎬 FPS: {result['fps']:.2f}")
            print(f"   ⏱️  Duration: {result['duration_seconds']:.2f}s ({result['duration_minutes']:.2f} min)")
            print(f"   🖼️  Frames: {result['frame_count']}")
            print(f"   📐 Resolution: {result['width']}x{result['height']}")
            print(f"   💾 Size: {result['file_size_mb']:.2f} MB")
            print()
            
            total_duration += result['duration_seconds']
            total_frames += result['frame_count']
    
    print("=" * 80)
    print(f"📊 SUMMARY:")
    print(f"   📁 Total files: {len(mp4_files)}")
    print(f"   ⏱️  Total duration: {total_duration:.2f}s ({total_duration/60:.2f} min)")
    print(f"   🖼️  Total frames: {total_frames}")
    if total_duration > 0:
        avg_fps = total_frames / total_duration
        print(f"   🎬 Average FPS: {avg_fps:.2f}")

def main():
    # Default directory
    default_dir = "/home/a/Projects/ros2_ws/data/camera"
    
    if len(sys.argv) > 1:
        # Use provided path
        path = sys.argv[1]
    else:
        # Use default directory
        path = default_dir
    
    if os.path.isfile(path):
        # Single file analysis
        print(f"📹 Analyzing single file: {path}")
        print("=" * 50)
        result = analyze_video(path)
        if result:
            print(f"📹 {result['filename']}")
            print(f"   🎬 FPS: {result['fps']:.2f}")
            print(f"   ⏱️  Duration: {result['duration_seconds']:.2f}s ({result['duration_minutes']:.2f} min)")
            print(f"   🖼️  Frames: {result['frame_count']}")
            print(f"   📐 Resolution: {result['width']}x{result['height']}")
            print(f"   💾 Size: {result['file_size_mb']:.2f} MB")
    
    elif os.path.isdir(path):
        # Directory analysis
        analyze_directory(path)
    
    else:
        print(f"❌ Path not found: {path}")
        print(f"Usage: python3 {sys.argv[0]} [path_to_video_or_directory]")
        print(f"Default directory: {default_dir}")

if __name__ == "__main__":
    main()
