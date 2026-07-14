"""OS-level always-on-top overlay window showing live run metrics,
separate from the PyBullet simulation window entirely. Anchoring debug
text inside the 3D world to a fixed screen position (via inverting the
chase-cam's view matrix each tick) worked geometrically but looked bad in
practice, so the metrics panel lives in its own borderless Tk window
instead -- simpler and immune to camera movement/FOV assumptions.

Runs Tkinter's mainloop in a genuinely separate OS process (a background
*thread* was tried first and crashed -- Tcl/Tk's interpreter state is not
safe to touch from a thread other than the one that created it; "run Tk on
its own thread" is a well-known Tkinter footgun, not something specific to
this codebase). Uses multiprocessing's "spawn" start method rather than
the Linux default "fork", since by the time this is created PyBullet
already has a live GL context and internal threads open (main.py creates
the overlay right after p.connect(p.GUI, ...)) -- forking a process with
open native threads/GL contexts is unsafe; spawn starts a clean
interpreter instead.
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
        # Auto-size to content instead of a fixed box -- a fixed size
        # clipped the last line once the panel grew (e.g. adding the
        # compute-energy line). Re-anchored to the bottom-right corner
        # every time, since growing/shrinking the window moves its
        # top-left but the corner is what should stay fixed.
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
