#include <iostream>
#include "translator.h"
#include "robotics_msgs.pb.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/messagesender.h>
#include <sepia/comm2/observer.h>
#include <sepia/comm2/observerbase.h>
#include "roboclaw_msgs.pb.h"

Translator::Translator()
{
}

Translator::~Translator()
{
}

void Translator::own_thread()
{
    sepia::comm2::MessageSender::init();
    sepia::comm2::ObserverBase::initReceiver();
    sepia::comm2::Observer< robotics_msgs::PS3Controller >::initReceiver();
    while( !m_terminate && sepia::comm2::ObserverBase::threadReceiver() )
    {
    }
    sepia::comm2::ObserverBase::destroyReceiver();
    sepia::comm2::MessageSender::destroy();
}

void Translator::receive( const robotics_msgs::PS3Controller& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    roboclaw_msgs::Move msg;

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

    msg.set_left_motor( ( a_msg.left_axis().vertical() * m_scalingFactor ) / 32767 );
    msg.set_right_motor( ( a_msg.right_axis().vertical() * m_scalingFactor ) / 32767 );

    sepia::comm2::Dispatcher< roboclaw_msgs::Move >::send( &msg );
}