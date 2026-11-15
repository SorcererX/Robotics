#!/usr/bin/env python3
"""Programmatic helper that publishes protobuf messages over ZeroMQ endpoints defined
in sepia.yaml (or the SEPIA_CONFIG override)."""

import imu_msgs_pb2 as imu_msgs
import path_msgs_pb2 as path_msgs
import roboclaw_msgs_pb2 as roboclaw_msgs

from message_sender import send_protobuf_message


def send_move( left_motor: int, right_motor: int ) -> bool:
    """Construct and send a Move command."""
    msg = roboclaw_msgs.Move()
    msg.left_motor = left_motor
    msg.right_motor = right_motor
    return send_protobuf_message( msg )


def send_move_speed_cm( left_cm_s: float, right_cm_s: float, acceleration_cm_s2: float ) -> bool:
    """Run both motors at a speed in cm/s, negative is backward."""
    msg = roboclaw_msgs.MoveSpeedCm(
        left_speed_cm_s=left_cm_s, right_speed_cm_s=right_cm_s, acceleration_cm_s2=acceleration_cm_s2 )
    return send_protobuf_message( msg )


def send_move_position_cm( left_cm: float, right_cm: float, speed_cm_s: float, acceleration_cm_s2: float,
                           deceleration_cm_s2: float | None = None, clear_buffer: bool = True ) -> bool:
    """Move both motors to an absolute position in cm, measured from the last encoder reset."""
    deceleration_cm_s2 = acceleration_cm_s2 if deceleration_cm_s2 is None else deceleration_cm_s2
    msg = roboclaw_msgs.MovePositionCm(
        left_speed_cm_s=speed_cm_s, left_acceleration_cm_s2=acceleration_cm_s2, left_deceleration_cm_s2=deceleration_cm_s2,
        left_position_cm=left_cm,
        right_speed_cm_s=speed_cm_s, right_acceleration_cm_s2=acceleration_cm_s2, right_deceleration_cm_s2=deceleration_cm_s2,
        right_position_cm=right_cm,
        clear_buffer=clear_buffer )
    return send_protobuf_message( msg )


def send_move_distance_cm( left_cm: float, right_cm: float, speed_cm_s: float, acceleration_cm_s2: float,
                           deceleration_cm_s2: float | None = None, clear_buffer: bool = True ) -> bool:
    """Move both motors a distance in cm, negative is backward. With clear_buffer=False the move is queued
    and starts from the target of the move before it."""
    deceleration_cm_s2 = acceleration_cm_s2 if deceleration_cm_s2 is None else deceleration_cm_s2
    msg = roboclaw_msgs.MoveDistanceCm(
        left_speed_cm_s=speed_cm_s, left_acceleration_cm_s2=acceleration_cm_s2, left_deceleration_cm_s2=deceleration_cm_s2,
        left_distance_cm=left_cm,
        right_speed_cm_s=speed_cm_s, right_acceleration_cm_s2=acceleration_cm_s2, right_deceleration_cm_s2=deceleration_cm_s2,
        right_distance_cm=right_cm,
        clear_buffer=clear_buffer )
    return send_protobuf_message( msg )


def send_set_encoders_cm( left_cm: float, right_cm: float ) -> bool:
    """Set both encoders to a position in cm."""
    return send_protobuf_message( roboclaw_msgs.SetEncodersCm( left_position_cm=left_cm, right_position_cm=right_cm ) )


def line( distance_cm: float, speed_cm_s: float = 0 ) -> path_msgs.PathSegment:
    """A straight segment, negative is backward. A speed above 0 replaces the speed of the path."""
    return path_msgs.PathSegment( line=path_msgs.Line( distance_cm=distance_cm ), speed_cm_s=speed_cm_s )


def turn( angle_deg: float, speed_cm_s: float = 0 ) -> path_msgs.PathSegment:
    """A turn on the spot, counter-clockwise (left) is positive."""
    return path_msgs.PathSegment( turn=path_msgs.Turn( angle_deg=angle_deg ), speed_cm_s=speed_cm_s )


def arc( radius_cm: float, angle_deg: float, backward: bool = False, speed_cm_s: float = 0 ) -> path_msgs.PathSegment:
    """A circle segment that changes the heading by angle_deg, the circle is on the left for a positive angle."""
    return path_msgs.PathSegment( arc=path_msgs.Arc( radius_cm=radius_cm, angle_deg=angle_deg, backward=backward ),
                                  speed_cm_s=speed_cm_s )


def send_follow_path( segments: list[ path_msgs.PathSegment ], speed_cm_s: float, acceleration_cm_s2: float,
                      path_id: int = 0, append: bool = False ) -> bool:
    """Drive the segments made with line(), turn() and arc(). PathFollower reports progress with PathProgress."""
    msg = path_msgs.FollowPath( id=path_id, segments=segments, speed_cm_s=speed_cm_s,
                                acceleration_cm_s2=acceleration_cm_s2, append=append )
    return send_protobuf_message( msg )


def send_stop_path( deceleration_cm_s2: float = 0 ) -> bool:
    """Stop the running path, with its own acceleration when deceleration_cm_s2 is 0."""
    return send_protobuf_message( path_msgs.StopPath( deceleration_cm_s2=deceleration_cm_s2 ) )


def send_reset_heading( heading_deg: float = 0 ) -> bool:
    """Set the heading ImuReader reports."""
    return send_protobuf_message( imu_msgs.ResetHeading( heading_deg=heading_deg ) )


if __name__ == "__main__":
    raise SystemExit( "send_message.py is intended for programmatic use. Call send_move(...) or the *_cm functions from your code." )
