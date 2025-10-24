"""
Keep current pipeline, finalizes prev pipelines in separate threads
"""
import threading, gi

from on_bus_message import on_message

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

class PipelineManager:
    def __init__(self, build_func, args):
        self.build_func = build_func
        self.pipeline = None
        self.finalizers = []
        self.chunk_id = 0
        self.args = args

    def _finalize_async(self, pipeline, tag):
        """Background finalization thread."""
        def worker():
            bus = pipeline.get_bus()
            pipeline.send_event(Gst.Event.new_eos())
            msg = bus.timed_pop_filtered(
                Gst.CLOCK_TIME_NONE,
                Gst.MessageType.EOS | Gst.MessageType.ERROR
            )
            if msg and msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                print(f"[{tag}] finalize error: {err} ({dbg})")
            pipeline.set_state(Gst.State.NULL)
            print(f"[{tag}] finalize complete.")

        t = threading.Thread(target=worker, daemon=True)
        t.start()
        self.finalizers.append(t)

    def start_new_chunk(self, loop):
        """Finalize old pipeline and start a new one."""
        old = self.pipeline
        if old:
            old_bus = old.get_bus()
            old_bus.remove_signal_watch()
            self._finalize_async(old, f"chunk_{self.chunk_id:05d}")
        self.chunk_id += 1
        print(f"▶ Starting chunk {self.chunk_id}")
        video_file = self.args.video_file
        use_gpu = not self.args.no_gpu
        self.pipeline = self.build_func(video_file, use_gpu, self.chunk_id)
        self.pipeline.set_state(Gst.State.PLAYING)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_message, loop, self, not self.args.no_loop)

        return self.pipeline

    def finalize_all(self):
        """Wait for background finalizations on shutdown."""
        for t in self.finalizers:
            t.join(timeout=5)
        if self.pipeline:
            print("Sending EOS for last active pipeline...")
            bus = self.pipeline.get_bus()
            self.pipeline.send_event(Gst.Event.new_eos())
            bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE,
                                   Gst.MessageType.EOS | Gst.MessageType.ERROR)
            self.pipeline.set_state(Gst.State.NULL)
        print("All pipelines finalized.")

