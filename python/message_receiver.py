#!/usr/bin/env python3
"""Lightweight subscriber for protobuf messages over ZeroMQ (sepia.yaml config)."""

from typing import Any, Dict, List, Type

from config_loader import load_configuration

try:
    import zmq
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency pyzmq. Install it with `pip install pyzmq` or your distro's package manager."
    ) from exc

_CONFIG_CACHE: Dict[str, Any] | None = None
_RECEIVER_SOCKET = None
_SUBSCRIPTION_MAP: Dict[str, object] | None = None


class MessageHandler:
    """Dispatch to handlers keyed by message type and register subscriptions."""

    handlers: Dict[ Type, Any ]

    def __init__( self ):
        self.handlers = {}

    def handle( self, msg ):
        handler = self.handlers.get( type( msg ), self.on_unknown )
        return handler( msg )

    def on_unknown( self, msg ):
        print( f"Unhandled {msg.DESCRIPTOR.full_name}: {msg}" )

    def register_subscriptions( self ):
        """Register subscriptions for each message type this handler supports."""
        for msg_cls in self.handlers:
            add_subscription( msg_cls )


def get_config() -> Dict[str, Any]:
    """Load configuration once and cache it for repeated use."""
    global _CONFIG_CACHE
    if _CONFIG_CACHE is None:
        _CONFIG_CACHE = load_configuration()
    return _CONFIG_CACHE


def add_subscription( message_cls: Type ) -> str:
    """Register a message class to subscribe to; applies to existing socket if present."""
    global _SUBSCRIPTION_MAP, _RECEIVER_SOCKET
    if _SUBSCRIPTION_MAP is None:
        _SUBSCRIPTION_MAP = {}
    topic = message_cls.DESCRIPTOR.full_name
    _SUBSCRIPTION_MAP[ topic ] = message_cls
    if _RECEIVER_SOCKET is not None:
        _RECEIVER_SOCKET.setsockopt_string( zmq.SUBSCRIBE, topic )
    return topic


def initialize_receiver( timeout_seconds: float = 5.0 ) -> Dict[str, Any]:
    """Initialize configuration and a shared SUB socket using registered subscriptions."""
    global _RECEIVER_SOCKET, _SUBSCRIPTION_MAP
    cfg = get_config()

    if not _SUBSCRIPTION_MAP:
        raise RuntimeError( "No subscriptions registered. Call add_subscription(...) first." )

    if _RECEIVER_SOCKET is None:
        context = zmq.Context.instance()
        socket = context.socket( zmq.SUB )
        for topic in _SUBSCRIPTION_MAP:
            socket.setsockopt_string( zmq.SUBSCRIBE, topic )
        socket.connect( cfg["publisher_url"] )
        socket.setsockopt( zmq.RCVTIMEO, int( timeout_seconds * 1000 ) if timeout_seconds > 0 else -1 )
        _RECEIVER_SOCKET = socket
    return cfg


def close_receiver_socket():
    """Explicitly close the shared receiver socket."""
    global _RECEIVER_SOCKET, _SUBSCRIPTION_MAP
    if _RECEIVER_SOCKET is not None:
        _RECEIVER_SOCKET.close()
        _RECEIVER_SOCKET = None
    _SUBSCRIPTION_MAP = None


def receive_messages( max_messages: int = 1, timeout_seconds: float = 5.0 ) -> List[ Any ]:
    if _RECEIVER_SOCKET is None or _SUBSCRIPTION_MAP is None or _CONFIG_CACHE is None:
        raise RuntimeError( "Receiver not initialized. Call initialize_receiver() before receiving." )

    socket = _RECEIVER_SOCKET
    socket.setsockopt( zmq.RCVTIMEO, int( timeout_seconds * 1000 ) if timeout_seconds > 0 else -1 )

    remaining = max_messages if max_messages != 0 else -1  # -1 for infinite
    received: List[ Any ] = []

    while remaining != 0:
        try:
            topic = socket.recv_string()
            payload = socket.recv()
        except zmq.Again:
            break

        base_topic = topic.split( ",", 1 )[ 0 ]
        message_cls = _SUBSCRIPTION_MAP.get( base_topic )
        if message_cls is None:
            continue

        message = message_cls()
        message.ParseFromString( payload )

        received.append( message )

        if remaining > 0:
            remaining -= 1

    return received
