"""
GStreamer Pipeline Setup Module
Handles pipeline construction with GPU acceleration and video transformations
"""

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

def on_pad_added(element, pad, next_element):
    """Handle dynamic pad from qtdemux"""
    # Get the pad capabilities to check if it's video
    caps = pad.get_current_caps()
    if caps:
        structure = caps.get_structure(0)
        name = structure.get_name()

        # Link only video pads
        if name.startswith("video/"):
            sink_pad = next_element.get_static_pad("sink")
            if not sink_pad.is_linked():
                result = pad.link(sink_pad)
                if result == Gst.PadLinkReturn.OK:
                    print(f"✓ Linked video stream: {name}")
                else:
                    print(f"✗ Failed to link video stream: {result}")

def create_pipeline(video_file, use_gpu=True):
    """
    Create a video playback pipeline with optional GPU acceleration
    decoder
       ↓
       tee
       ├── queue_display → convert → flip → scale → capsfilter → videosink
       └── queue_record  → nvh264enc → h264parse → splitmuxsink


    Args:
        video_file: Path to video file
        use_gpu: Use NVIDIA GPU decoder if available (default: True)

    Returns:
        tuple: (pipeline, elements_dict)
    """

    pipeline = Gst.Pipeline.new("video-player")
    filesrc = Gst.ElementFactory.make("filesrc", "source")
    qtdemux = Gst.ElementFactory.make("qtdemux", "demuxer")
    h264parse = Gst.ElementFactory.make("h264parse", "parser")

    # Decoder (try GPU first if requested)
    if use_gpu:
        decoder = Gst.ElementFactory.make("nvh264dec", "decoder")
        if decoder:
            print("  ✓ Using NVIDIA GPU decoder (nvh264dec)")
        else:
            print("  ✗ NVIDIA decoder unavailable, falling back to software")
            decoder = Gst.ElementFactory.make("avdec_h264", "decoder")
            print("  ✓ Using software decoder (avdec_h264)")
    else:
        decoder = Gst.ElementFactory.make("avdec_h264", "decoder")
        print("  ✓ Using software decoder (avdec_h264)")

    tee = Gst.ElementFactory.make("tee", "tee")

    # recording branch
    queue_record = Gst.ElementFactory.make("queue", "queue_record")
    encoder = Gst.ElementFactory.make("nvh264enc", "encoder")
    parse = Gst.ElementFactory.make("h264parse", "parse")
    splitmux = Gst.ElementFactory.make("splitmuxsink", "smux")

    # display branch
    queue_display = Gst.ElementFactory.make("queue", "queue_display")
    convert = Gst.ElementFactory.make("videoconvert", "converter")
    videoflip = Gst.ElementFactory.make("videoflip", "flipper")
    videoscale = Gst.ElementFactory.make("videoscale", "scaler")
    capsfilter = Gst.ElementFactory.make("capsfilter", "filter")
    videosink = Gst.ElementFactory.make("ximagesink", "sink")

    # Check all elements created successfully
    elements = {
        'filesrc': filesrc,
        'qtdemux': qtdemux,
        'h264parse': h264parse,
        'decoder': decoder,
        'queue_record': queue_record,
        'encoder': encoder,
        'parse': parse,
        'splitmux': splitmux,
        'queue_display': queue_display,
        'convert': convert,
        'videoflip': videoflip,
        'videoscale': videoscale,
        'capsfilter': capsfilter,
        'videosink': videosink,
        'tee': tee
    }

    if not all(elements.values()):
        print("ERROR: Failed to create one or more elements")
        print("Make sure you have gstreamer1.0-plugins-good and gstreamer1.0-plugins-bad installed")
        sys.exit(1)

    # Configure properties
    filesrc.set_property("location", video_file)
    splitmux.set_property("muxer-factory", "mp4mux")
    splitmux.set_property("max-size-time", 5 * 1_000_000_000)  # 5 min
    splitmux.set_property("location", "chunk_%05d.mp4")

    # Set rotation 0=none, 1=90°CW, 2=180°, 3=270°CW, 4=h-flip, 5=v-flip
    videoflip.set_property("method", 3)

    # Scaling method (0=nearest, 1=bilinear, 2=bicubic, 3=lanczos)
    videoscale.set_property("method", 0)  # Fastest
    videoscale.set_property("add-borders", True)  # Maintain aspect ratio with letterboxing

    # Set output dimensions
    caps = Gst.Caps.from_string(
        f"video/x-raw,width=(int){800},height=(int){800},pixel-aspect-ratio=(fraction)1/1"
    )
    capsfilter.set_property("caps", caps)

    # Sink properties
    videosink.set_property("force-aspect-ratio", True)

    # Add all elements to pipeline
    print("Building pipeline...")
    for element in elements.values():
        pipeline.add(element)

    # Link elements
    # Static links
    filesrc.link(qtdemux)
    h264parse.link(decoder)
    decoder.link(tee)
    # record branch
    tee.link(queue_record)
    queue_record.link(encoder)
    encoder.link(parse)
    parse.link(splitmux)
    # display branch
    tee.link(queue_display)
    queue_display.link(convert)
    convert.link(videoflip)
    videoflip.link(videoscale)
    videoscale.link(capsfilter)
    capsfilter.link(videosink)

    # Dynamic link (qtdemux → h264parse)
    qtdemux.connect("pad-added", on_pad_added, h264parse)

    print("Pipeline construction complete")

    return pipeline, elements

def print_pipeline_info(elements):
    """
    Print negotiated formats for all elements

    Args:
        elements: Dictionary of pipeline elements
    """

    print("\n" + "="*60)
    print("Pipeline Negotiated Formats:")
    print("="*60)

    for name, elem in elements.items():
        # Check source pads
        for pad in elem.srcpads:
            caps = pad.get_current_caps()
            if caps:
                structure = caps.get_structure(0)
                caps_name = structure.get_name()

                # Extract key info
                info = f"{name}:src → {caps_name}"

                if structure.has_field("width") and structure.has_field("height"):
                    width = structure.get_int("width")[1]
                    height = structure.get_int("height")[1]
                    info += f" ({width}x{height})"

                if structure.has_field("format"):
                    fmt = structure.get_string("format")
                    info += f" [{fmt}]"

                print(f"  {info}")

    print("="*60 + "\n")

def print_pipeline(pipeline):
    it = pipeline.iterate_elements()
    done = False
    while not done:
        res, element = it.next()
        if res == Gst.IteratorResult.OK:
            print(f"• {element.name} ({element.__class__.__name__})")
        elif res == Gst.IteratorResult.DONE:
            done = True
        elif res == Gst.IteratorResult.RESYNC:
            it.resync()
        elif res == Gst.IteratorResult.ERROR:
            print("Iterator error")
            done = True