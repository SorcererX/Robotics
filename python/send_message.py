#!/usr/bin/env python3
"""
Generic helper that publishes protobuf messages over ZeroMQ endpoints defined
in sepia.yaml (or the SEPIA_CONFIG override). This removes the dependency on
the C++ SepiaComm library.
Usage example:
  python python/send_message.py --module roboclaw_msgs_pb2 --message Move --fields left_motor=500,right_motor=-500
"""

import argparse
import ast
import importlib
import sys
import time
from typing import Any, Dict, Iterable, List

from google.protobuf.json_format import MessageToDict

from config_loader import load_configuration

try:
    import zmq
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency pyzmq. Install it with `pip install pyzmq` or your distro's package manager."
    ) from exc


def parse_field_arg( field: str ):
    if "=" not in field:
        raise ValueError( f"Field '{field}' is missing '=' (expected name=value)." )
    name, raw_value = field.split( "=", 1 )
    try:
        value = ast.literal_eval( raw_value )
    except Exception:
        value = raw_value
    return name, value


def parse_fields_arg_list( raw: str ) -> List[ str ]:
    if raw is None:
        return []
    text = raw.strip()
    if not text:
        return []
    if text.startswith( "[" ):
        try:
            values = ast.literal_eval( text )
            if not isinstance( values, list ):
                raise ValueError
            return [ str( entry ) for entry in values ]
        except Exception as exc:
            raise ValueError( f"Unable to parse --fields argument: {raw}" ) from exc
    return [ part.strip() for part in text.split( "," ) if part.strip() ]


def build_message( module_name: str, message_name: str ):
    try:
        module = importlib.import_module( module_name )
    except ModuleNotFoundError as exc:
        raise ValueError(
            f"Unable to import module '{module_name}'. Ensure it is on PYTHONPATH and protobufs are generated."
        ) from exc

    try:
        message_cls = getattr( module, message_name )
    except AttributeError as exc:
        raise ValueError( f"Message '{message_name}' not found in module '{module_name}'." ) from exc

    return message_cls()


def apply_fields( msg, field_pairs: Iterable[str] ):
    for field_def in field_pairs:
        name, value = parse_field_arg( field_def )
        if name not in msg.DESCRIPTOR.fields_by_name:
            raise ValueError( f"Message '{msg.DESCRIPTOR.full_name}' has no field '{name}'." )
        setattr( msg, name, value )


def send_protobuf_message( msg, warmup_seconds: float = 0.1 ) -> Dict[str, Any]:
    cfg = load_configuration()
    context = zmq.Context.instance()
    socket = context.socket( zmq.PUB )
    socket.connect( cfg["subscriber_url"] )
    if warmup_seconds > 0:
        time.sleep( warmup_seconds )
    payload = msg.SerializeToString()
    topic = msg.DESCRIPTOR.full_name
    try:
        socket.send_string( topic, zmq.SNDMORE )
        socket.send( payload )
        summary = MessageToDict(
            msg,
            preserving_proto_field_name=True,
            including_default_value_fields=True,
        ) or {}
        return {"topic": topic, "summary": summary, "endpoint": cfg["subscriber_url"]}
    finally:
        socket.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Send protobuf commands over raw ZeroMQ (sepia.yaml config).")
    parser.add_argument("--module", default="roboclaw_msgs_pb2", help="Python module containing generated protobufs.")
    parser.add_argument("--message", default="Move", help="Message class inside the provided module.")
    parser.add_argument(
        "--fields",
        action="append",
        default=[],
        metavar="LIST",
        help="Comma-separated list or Python list of NAME=VALUE pairs. Repeat to send multiple messages.",
    )
    args = parser.parse_args()
    field_groups = args.fields or [ "" ]

    for field_group in field_groups:
        try:
            msg = build_message( args.module, args.message )
            apply_fields( msg, parse_fields_arg_list( field_group ) )
            result = send_protobuf_message( msg )
        except ValueError as exc:
            raise SystemExit( str( exc ) ) from exc

        print(f"Connecting sender to {result['endpoint']}")
        print(f"Sent {result['topic']}: {result['summary']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
