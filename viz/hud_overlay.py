"""OS-level always-on-top overlay window for live run metrics, separate
from the PyBullet window -- a borderless Tk window, simpler and immune to
camera movement than anchoring text inside the 3D world.

Runs Tk's mainloop in a separate OS *process*: Tcl/Tk's interpreter is not
thread-safe to touch off-thread, so a background thread crashes. Uses
multiprocessing's "spawn" (not the Linux default "fork"), since PyBullet
already has a live GL context and native threads open by the time this is
created -- forking with those open is unsafe.
"""
from __future__ import annotations
import multiprocessing
import queue as queue_mod


def _overlay_process_main(q: "multiprocessing.Queue") -> None:
    import tkinter as tk

    root = tk.Tk()
    root.overrideredirect(True)
    root.attributes("-topmost", True)
    try:
        root.attributes("-alpha", 0.85)
    except tk.TclError:
        pass
    root.configure(bg="#12161c")
    sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
    margin = 24

    label = tk.Label(root, text="", justify="left", anchor="nw",
                      font=("TkFixedFont", 12), fg="#e8e8e8", bg="#12161c",
                      padx=14, pady=12)
    label.pack(fill="both", expand=True)

    def resize_to_fit():
        # Auto-size to content (a fixed box clips growing text). Re-anchor
        # to the bottom-right corner each time, since resizing moves the
        # window's top-left but the corner should stay fixed.
        root.update_idletasks()
        w = max(1, label.winfo_reqwidth())
        h = max(1, label.winfo_reqheight())
        root.geometry(f"{w}x{h}+{sw - w - margin}+{sh - h - margin}")

    def poll():
        changed = False
        try:
            while True:
                text = q.get_nowait()
                if text is None:
                    root.destroy()
                    return
                label.config(text=text)
                changed = True
        except queue_mod.Empty:
            pass
        if changed:
            resize_to_fit()
        root.after(100, poll)

    resize_to_fit()
    root.after(100, poll)
    root.mainloop()


class MetricsOverlay:
    def __init__(self):
        ctx = multiprocessing.get_context("spawn")
        self._queue = ctx.Queue()
        self._proc = ctx.Process(target=_overlay_process_main, args=(self._queue,), daemon=True)
        self._proc.start()

    def update(self, text: str) -> None:
        self._queue.put(text)

    def close(self) -> None:
        self._queue.put(None)
        self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            self._proc.terminate()
