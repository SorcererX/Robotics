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

void Translator::receive( const robotics_msgs::PS3Axis& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    roboclaw_msgs::Move msg;

    msg.set_left_motor( ( a_msg.left_axis().vertical() * m_scalingFactor ) / 32767 );
    msg.set_right_motor( ( a_msg.right_axis().vertical() * m_scalingFactor ) / 32767 );

    sepia::comm2::Dispatcher< roboclaw_msgs::Move >::send( &msg );
}

void Translator::receive( const robotics_msgs::PS3Button& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    if( a_msg.cross_pressed() )
    {
        m_scalingFactor = -31;
    }
    else if( a_msg.triangle_pressed() )
    {
        m_scalingFactor = -127;
    }
    else if( a_msg.square_pressed() )
    {
        m_scalingFactor = -63;
    }
    else if( a_msg.circle_pressed() )
    {
        m_scalingFactor = -15;
    }
    else if( a_msg.r1_pressed() )
    {
        roboclaw_msgs::RequestStatus msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestStatus >::send( &msg );
    }
}