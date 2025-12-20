#!/usr/bin/env python3
"""
Generic helper that subscribes to ZeroMQ topics defined in sepia.yaml (or the
SEPIA_CONFIG override) and decodes protobuf payloads. No SepiaComm dependency.
Usage example:
  python python/receive_message.py --module roboclaw_msgs_pb2 --message Move --count 5
"""

import argparse
import importlib
import sys
from typing import Dict, Iterable, List

from google.protobuf.json_format import MessageToDict

from config_loader import load_configuration

try:
    import zmq
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency pyzmq. Install it with `pip install pyzmq` or your distro's package manager."
    ) from exc


def import_proto_module( module_name: str ):
    try:
        return importlib.import_module( module_name )
    except ModuleNotFoundError as exc:
        raise ValueError( f"Unable to import module '{module_name}'. Ensure it is on PYTHONPATH." ) from exc


def resolve_messages( module_name: str, message_names: Iterable[ str ] ) -> Dict[ str, object ]:
    module = import_proto_module( module_name )
    mapping: Dict[ str, object ] = {}
    for name in message_names:
        try:
            message_cls = getattr( module, name )
        except AttributeError as exc:
            raise ValueError( f"Message '{name}' not found in module '{module_name}'." ) from exc
        mapping[ message_cls.DESCRIPTOR.full_name ] = message_cls
    return mapping


def receive_messages( subscription_map: Dict[ str, object ], max_messages: int, timeout_seconds: float ) -> List[ Dict[ str, object ] ]:
    cfg = load_configuration()
    endpoint = cfg["publisher_url"]

    context = zmq.Context.instance()
    socket = context.socket( zmq.SUB )

    for topic in subscription_map:
        socket.setsockopt_string( zmq.SUBSCRIBE, topic )

    socket.connect( endpoint )

    received: List[ Dict[ str, object ] ] = []
    remaining = max_messages

    if remaining == 0:
        remaining = -1  # run forever

    try:
        socket.setsockopt( zmq.RCVTIMEO, int( timeout_seconds * 1000 ) if timeout_seconds > 0 else -1 )

        while remaining != 0:
            try:
                topic = socket.recv_string()
                payload = socket.recv()
            except zmq.Again:
                break

            base_topic = topic.split( ",", 1 )[ 0 ]
            message_cls = subscription_map.get( base_topic )

            if message_cls is None:
                continue

            message = message_cls()
            message.ParseFromString( payload )

            summary = MessageToDict(
                message,
                preserving_proto_field_name=True,
                including_default_value_fields=True,
            )

            received.append(
                {
                    "topic": topic,
                    "message": message,
                    "summary": summary,
                    "endpoint": endpoint,
                }
            )

            if remaining > 0:
                remaining -= 1
    finally:
        socket.close()

    return received


def main() -> int:
    parser = argparse.ArgumentParser( description="Receive protobuf messages over raw ZeroMQ (sepia.yaml config)." )
    parser.add_argument( "--module", default="roboclaw_msgs_pb2", help="Python module containing generated protobufs." )
    parser.add_argument(
        "--message",
        action="append",
        default=[ "Move" ],
        metavar="NAME",
        help="Message class to subscribe to (repeatable). Defaults to Move.",
    )
    parser.add_argument( "--count", type=int, default=1, help="Number of messages to receive (0 for infinite)." )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Per-poll timeout in seconds (ignored when --count=0).",
    )
    args = parser.parse_args()

    try:
        subscription_map = resolve_messages( args.module, args.message )
    except ValueError as exc:
        raise SystemExit( str( exc ) ) from exc

    results = receive_messages( subscription_map, args.count, args.timeout )

    if not results:
        print( "No messages received." )
        return 1

    for item in results:
        print( f"Received {item['topic']} from {item['endpoint']}: {item['summary']}" )

    return 0


if __name__ == "__main__":
    sys.exit( main() )
