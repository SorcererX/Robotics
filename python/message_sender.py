#!/usr/bin/env python3
"""Lightweight publisher for protobuf messages over ZeroMQ (sepia.yaml config)."""

import time
from typing import Any, Dict

from config_loader import load_configuration

try:
    import zmq
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency pyzmq. Install it with `pip install pyzmq` or your distro's package manager."
    ) from exc

_CONFIG_CACHE: Dict[str, Any] | None = None
_SENDER_SOCKET = None


def get_config() -> Dict[str, Any]:
    """Load configuration once and cache it for repeated use."""
    global _CONFIG_CACHE
    if _CONFIG_CACHE is None:
        _CONFIG_CACHE = load_configuration()
    return _CONFIG_CACHE


def initialize_sender( warmup_seconds: float = 0.1 ) -> Dict[str, Any]:
    """Initialize configuration and a shared PUB socket. Call once before sending."""
    global _SENDER_SOCKET
    cfg = get_config()
    if _SENDER_SOCKET is None:
        context = zmq.Context.instance()
        socket = context.socket( zmq.PUB )
        socket.connect( cfg["subscriber_url"] )
        _SENDER_SOCKET = socket
        if warmup_seconds > 0:
            time.sleep( warmup_seconds )
    return cfg


def close_sender_socket():
    """Explicitly close the shared sender socket."""
    global _SENDER_SOCKET
    if _SENDER_SOCKET is not None:
        _SENDER_SOCKET.close()
        _SENDER_SOCKET = None


def send_protobuf_message( msg ) -> Dict[str, Any]:
    if _SENDER_SOCKET is None or _CONFIG_CACHE is None:
        return False
    try:
        socket = _SENDER_SOCKET
        payload = msg.SerializeToString()
        topic = msg.DESCRIPTOR.full_name
        socket.send_string( topic, zmq.SNDMORE )
        socket.send( payload )
        return True
    except Exception:
        return False
