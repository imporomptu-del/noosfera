#!/usr/bin/env python3
"""
GStreamer Video Player with Infinite Looping
Supports GPU acceleration, rotation, and scaling

Usage:
    python gstreamer_play.py <video_file.mp4>
    python gstreamer_play.py <video_file.mp4> --no-loop
"""

import sys
import time
import argparse
import gi

from pipeline_manager import PipelineManager

from on_bus_message import start_time, loop_count

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from gstreamer_pipeline import create_pipeline, print_pipeline_info, print_pipeline

def main():
    global loop_count, start_time

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='GStreamer video player with GPU acceleration and looping'
    )
    parser.add_argument('video_file', help='Path to video file (MP4)')
    parser.add_argument('--no-loop', action='store_true',
                       help='Disable infinite looping (play once)')
    parser.add_argument('--no-gpu', action='store_true',
                       help='Disable GPU acceleration (use CPU decoder)')
    parser.add_argument('--show-caps', action='store_true',
                       help='Show negotiated capabilities')

    args = parser.parse_args()

    Gst.init(None)
    manager = PipelineManager(create_pipeline, args)

    loop = GLib.MainLoop()
    manager.start_new_chunk(loop)

    pipeline = manager.pipeline
    print_pipeline(pipeline)

    print("\n" + "="*60)
    print("GStreamer Video Player")
    print("="*60)
    print(f"File: {args.video_file}")
    print(f"Looping: {'enabled' if not args.no_loop else 'disabled'}")
    print(f"GPU decode: {'enabled' if not args.no_gpu else 'disabled'}")
    print("="*60)
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")

    # Start pipeline
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("ERROR: Unable to set pipeline to PLAYING state")
        sys.exit(1)

    # Wait for negotiation and print caps if requested
    if args.show_caps:
        time.sleep(1)  # Wait for negotiation
        print_pipeline_info(pipeline)

    start_time = time.time()

    # Run main loop
    try:
        loop.run()
    except KeyboardInterrupt:
        elapsed = time.time() - start_time if start_time else 0
        print(f"\n\n{'='*60}")
        print(f"Playback stopped by user")
        print(f"Total loops: {loop_count}")
        print(f"Total time: {elapsed:.1f}s")
        print(f"{'='*60}\n")
        manager.pipeline.get_by_name("smux").send_event(Gst.Event.new_eos())
        manager.finalize_all()


if __name__ == "__main__":
    main()