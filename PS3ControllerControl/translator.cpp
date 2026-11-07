#include <algorithm>
#include <cstdlib>
#include <iostream>
#include "translator.h"
#include "robotics_msgs.pb.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/scopedmessagesender.h>
#include <sepia/comm2/observer.h>
#include <sepia/comm2/observerbase.h>
#include <sepia/comm2/receiver.h>
#include "roboclaw_msgs.pb.h"

Translator::Translator()
{
}

Translator::~Translator()
{
}

void Translator::own_thread()
{
    sepia::comm2::ScopedMessageSender sender;
    sepia::comm2::Receiver handler( this );
    while( !m_terminate && handler.exec() )
    {
    }
}

namespace
{
    // Sticks rarely rest at exactly 0, and the speed PID follows even a few counts per second.
    constexpr int32_t deadzone = 3277;

    // Stick up is negative, so it is inverted to make up forward.
    double stickToSpeed( int32_t a_stick, double a_maxSpeed )
    {
        if( std::abs( a_stick ) < deadzone )
        {
            return 0;
        }
        const double magnitude = std::min( std::abs( a_stick ), 32767 ) - deadzone;
        const double speed = magnitude * a_maxSpeed / ( 32767 - deadzone );
        return a_stick < 0 ? speed : -speed;
    }
}

void Translator::receive( const robotics_msgs::PS3Axis& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    roboclaw_msgs::MoveSpeedCm msg;

    msg.set_left_speed_cm_s( stickToSpeed( a_msg.left_axis().vertical(), m_maxSpeed ) );
    msg.set_right_speed_cm_s( stickToSpeed( a_msg.right_axis().vertical(), m_maxSpeed ) );
    // Reaches full speed in half a second.
    msg.set_acceleration_cm_s2( m_maxSpeed * 2 );

    sepia::comm2::Dispatcher< roboclaw_msgs::MoveSpeedCm >::send( &msg );
}

void Translator::receive( const robotics_msgs::PS3Button& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    if( a_msg.circle_pressed() )
    {
        m_maxSpeed = 5;
    }
    else if( a_msg.cross_pressed() )
    {
        m_maxSpeed = 10;
    }
    else if( a_msg.square_pressed() )
    {
        m_maxSpeed = 25;
    }
    else if( a_msg.triangle_pressed() )
    {
        m_maxSpeed = 50;
    }
    else if( a_msg.r1_pressed() )
    {
        roboclaw_msgs::RequestStatus msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestStatus >::send( &msg );
    }
}