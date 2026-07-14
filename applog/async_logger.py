"""Async JSON logging setup shared by main.py's run loop.

Attaches a queue-backed handler to the root logger so file (and optional
console) writes happen on a background thread instead of blocking the sim
loop. The JSON line shape matches what analysis/log_transformer.py expects
to reconstruct experiment_summary.csv offline: each line carries "ts",
"name" (the logger name -- "main", "teleop", "nav_algorithm", ...), "msg"
(the raw record.msg, which is sometimes a dict, e.g. main.py's terminator
record or teleop.py's {"event": "STOP"}), and whichever of "type",
"payload", "strategy" were attached via logging's extra= kwarg.
"""
from __future__ import annotations

import json
import logging
import logging.handlers
import os
import queue
from dataclasses import dataclass
from typing import Optional


@dataclass
class AsyncLoggerCfg:
    logfile: str
    max_bytes: int = 0
    queue_maxsize: int = 8000
    drop_on_full: bool = True
    console: bool = False
    level: int = logging.INFO
    json_format: bool = True


_EXTRA_FIELDS = ("type", "payload", "strategy")


class _JsonFormatter(logging.Formatter):
    # Deliberately bypasses logging.Formatter's default getMessage()/%
    # formatting, which stringifies record.msg -- several callers log a
    # dict directly (main.py's terminator, teleop.py's STOP record) and
    # log_transformer.py needs that structure intact after json.loads.
    def format(self, record: logging.LogRecord) -> str:
        entry = {"ts": record.created, "name": record.name, "msg": record.msg}
        for field in _EXTRA_FIELDS:
            if hasattr(record, field):
                entry[field] = getattr(record, field)
        return json.dumps(entry, default=str)


class _DropOnFullQueueHandler(logging.handlers.QueueHandler):
    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            pass


class AsyncLoggerHandle:
    def __init__(self, listener: logging.handlers.QueueListener,
                 queue_handler: logging.handlers.QueueHandler) -> None:
        self._listener = listener
        self._queue_handler = queue_handler

    def stop(self) -> None:
        self._listener.stop()
        logging.getLogger().removeHandler(self._queue_handler)


def setup_async_logger(cfg: AsyncLoggerCfg) -> AsyncLoggerHandle:
    log_dir = os.path.dirname(cfg.logfile)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)

    if cfg.max_bytes and cfg.max_bytes > 0:
        file_handler: logging.Handler = logging.handlers.RotatingFileHandler(
            cfg.logfile, maxBytes=cfg.max_bytes, backupCount=5)
    else:
        file_handler = logging.FileHandler(cfg.logfile)

    formatter: logging.Formatter = (
        _JsonFormatter() if cfg.json_format
        else logging.Formatter("%(asctime)s %(name)s %(message)s"))
    file_handler.setFormatter(formatter)
    handlers = [file_handler]

    if cfg.console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        handlers.append(console_handler)

    q: "queue.Queue[logging.LogRecord]" = queue.Queue(maxsize=cfg.queue_maxsize)
    handler_cls = _DropOnFullQueueHandler if cfg.drop_on_full else logging.handlers.QueueHandler
    queue_handler = handler_cls(q)

    root = logging.getLogger()
    root.setLevel(cfg.level)
    root.addHandler(queue_handler)

    listener = logging.handlers.QueueListener(q, *handlers, respect_handler_level=True)
    listener.start()

    return AsyncLoggerHandle(listener, queue_handler)
