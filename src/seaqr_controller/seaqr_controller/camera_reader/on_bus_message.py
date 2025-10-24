import time
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Global state
loop_count = 0
start_time = None


def on_message(bus, message, loop, mgr, enable_looping):
    global loop_count, start_time

    if message.type == Gst.MessageType.EOS:
        if enable_looping:
            elapsed = time.time() - start_time if start_time else 0
            loop_count += 1

            print(f"\n{'='*50}")
            print(f"Loop #{loop_count} complete (elapsed: {elapsed:.1f}s)")
            print(f"{'='*50}")

            print("EOS: rolling to next chunk")
            mgr.start_new_chunk(loop)

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
        if message.src == mgr.pipeline:
            old, new, pending = message.parse_state_changed()
            if new == Gst.State.PLAYING and old == Gst.State.PAUSED:
                print(f"Pipeline state: {old.value_nick} → {new.value_nick}")

    return True
