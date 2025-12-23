#!/usr/bin/env python3
"""Programmatic helper that publishes protobuf messages over ZeroMQ endpoints defined
in sepia.yaml (or the SEPIA_CONFIG override)."""

import roboclaw_msgs_pb2 as roboclaw_msgs

from message_sender import send_protobuf_message


def send_move( left_motor: int, right_motor: int ) -> bool:
    """Construct and send a Move command."""
    msg = roboclaw_msgs.Move()
    msg.left_motor = left_motor
    msg.right_motor = right_motor
    return send_protobuf_message( msg )


if __name__ == "__main__":
    raise SystemExit( "send_message.py is intended for programmatic use. Call send_move(...) from your code." )
