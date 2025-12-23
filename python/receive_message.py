#!/usr/bin/env python3
"""Programmatic helper that subscribes to ZeroMQ topics defined in sepia.yaml (or the
SEPIA_CONFIG override) and decodes protobuf payloads."""

import roboclaw_msgs_pb2 as roboclaw_msgs

from message_receiver import (
    MessageHandler,
    close_receiver_socket,
    initialize_receiver,
    receive_messages,
)


class MovementControlHandler( MessageHandler ):
    def __init__( self ):
        self.handlers = {
            roboclaw_msgs.Move: self.on_move,
            roboclaw_msgs.MovePosition: self.on_move_position,
            roboclaw_msgs.RequestStatus: self.on_request_status,
            roboclaw_msgs.MoveStatus: self.on_move_status,
        }
        self.register_subscriptions()

    def on_move( self, msg: roboclaw_msgs.Move ):
        print( f"Move left={msg.left_motor} right={msg.right_motor}" )

    def on_move_position( self, msg: roboclaw_msgs.MovePosition ):
        print(
            "MovePosition "
            f"left(accel={msg.left_acceleration},speed={msg.left_speed},decel={msg.left_deceleration},dist={msg.left_distance}) "
            f"right(accel={msg.right_acceleration},speed={msg.right_speed},decel={msg.right_deceleration},dist={msg.right_distance}) "
            f"clear_buffer={msg.clear_buffer}"
        )

    def on_request_status( self, msg: roboclaw_msgs.RequestStatus ):
        print( f"RequestStatus time={msg.time}" )

    def on_move_status( self, msg: roboclaw_msgs.MoveStatus ):
        print(
            "MoveStatus "
            f"left_encoder={msg.left_encoder} left_speed={msg.left_speed} "
            f"right_encoder={msg.right_encoder} right_speed={msg.right_speed}"
        )


def receive_roboclaw_messages( max_messages: int = 1, timeout_seconds: float = 5.0 ):
    """Receive messages for previously registered roboclaw subscriptions."""
    initialize_receiver( timeout_seconds=timeout_seconds )
    return receive_messages( max_messages=max_messages, timeout_seconds=timeout_seconds )


def register_movementcontrol_messages( handler=None ):
    """Register MovementControl-related messages based on handler mapping."""
    return handler or MovementControlHandler()


def receive_movementcontrol_messages( max_messages: int = 1, timeout_seconds: float = 5.0 ):
    """Receive messages relevant to MovementControl (Move, MovePosition, RequestStatus, MoveStatus)."""
    register_movementcontrol_messages()
    return receive_roboclaw_messages( max_messages=max_messages, timeout_seconds=timeout_seconds )


def receive_movementcontrol_until_keypress( timeout_seconds: float = 1.0 ):
    """Continuously receive MovementControl messages until interrupted (Ctrl+C)."""
    handler = register_movementcontrol_messages()
    initialize_receiver( timeout_seconds=timeout_seconds )
    print( "Listening for MovementControl messages. Press Ctrl+C to stop." )
    try:
        while True:
            for msg in receive_messages( max_messages=1, timeout_seconds=timeout_seconds ):
                handler.handle( msg )
    except KeyboardInterrupt:
        print( "Stopping listener." )
    finally:
        close_receiver_socket()


if __name__ == "__main__":
    receive_movementcontrol_until_keypress()
