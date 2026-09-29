"""Interactive viewing after evidence has been saved; no benchmark execution."""

import signal
import time

PREVIEW_READY = "R5_SCENE_PREVIEW_READY"


def hold_scene_preview(backend):
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    # The Windows launcher forwards Ctrl+C as CTRL_BREAK_EVENT.
    previous = signal.signal(signal.SIGBREAK, interrupt) if hasattr(signal, "SIGBREAK") else None
    try:
        backend.world.pause()
        print("Scene check saved. View freely; press Ctrl+C in the terminal or close the window to exit.", flush=True)
        # The supervisor cancels only the viewing phase's watchdog on this marker.
        print(PREVIEW_READY, flush=True)
        while backend.app.is_running():
            backend.app.update()
            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass  # Leaving the viewer must not turn a completed check into a failure.
    finally:
        if hasattr(signal, "SIGBREAK"):
            signal.signal(signal.SIGBREAK, previous)
