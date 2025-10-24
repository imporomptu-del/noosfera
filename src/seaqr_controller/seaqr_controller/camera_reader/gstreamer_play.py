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

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from gstreamer_pipeline import create_pipeline, print_pipeline_info, print_pipeline

# Global state
loop_count = 0
start_time = None

def on_message(bus, message, loop, pipeline, enable_looping):
    global loop_count, start_time

    if message.type == Gst.MessageType.EOS:
        if enable_looping:
            # Calculate stats
            elapsed = time.time() - start_time if start_time else 0
            loop_count += 1

            print(f"\n{'='*50}")
            print(f"Loop #{loop_count} complete (elapsed: {elapsed:.1f}s)")
            print(f"{'='*50}")

            src = pipeline.get_by_name("filesrc")
            src.set_state(Gst.State.NULL)
            src.set_state(Gst.State.READY)
            src.set_state(Gst.State.PLAYING)

            start_time = time.time()  # Reset timer
        else:
            print("\nEnd of stream")
            loop.quit()

    elif message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"\n{'='*50}")
        print(f"ERROR: {err}")
        print(f"Debug info: {debug}")
        print(f"{'='*50}\n")
        loop.quit()
    elif message.type == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(f"WARNING: {err}")

    elif message.type == Gst.MessageType.STATE_CHANGED:
        if message.src == pipeline:
            old, new, pending = message.parse_state_changed()
            if new == Gst.State.PLAYING and old == Gst.State.PAUSED:
                print(f"Pipeline state: {old.value_nick} → {new.value_nick}")

    return True



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
    pipeline, elements = create_pipeline(video_file=args.video_file, use_gpu=not args.no_gpu)
    print_pipeline(pipeline)
    loop = GLib.MainLoop()

    # Setup bus monitoring
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message, loop, pipeline, not args.no_loop)

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
        print_pipeline_info(elements)

    # Start timing
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
        elements['splitmux'].send_event(Gst.Event.new_eos())
        # gives the muxer time to finish writing the header and close the file.
        bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE,
            Gst.MessageType.EOS | Gst.MessageType.ERROR
        )


# Cleanup
    pipeline.set_state(Gst.State.NULL)
    print("Done!")



    pipeline.set_state(Gst.State.NULL)




    # bus.timed_pop_filtered(
    #     Gst.CLOCK_TIME_NONE,
    #     Gst.MessageType.EOS | Gst.MessageType.ERROR
    # )


if __name__ == "__main__":
    main()