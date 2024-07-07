#include "roboclawinterface.h"
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/receiver.h>
#include <sepia/comm2/scopedmessagesender.h>

RoboClawInterface::RoboClawInterface()
    : m_motor( "/dev/ttyACM0", 5, 0x80 )
{
    m_motor.begin( B115200 );
}

RoboClawInterface::~RoboClawInterface()
{
    m_motor.end();
}

void RoboClawInterface::own_thread()
{
    sepia::comm2::ScopedMessageSender sender;
    sepia::comm2::Receiver handler( this );
    while( !m_terminate && handler.exec() )
    {
    }
}

void RoboClawInterface::receive( const roboclaw_msgs::Move& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::flush;

    bool left_ok = false;
    bool right_ok = false;

    if( a_msg.left_motor() >= 0 )
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( a_msg.left_motor(), 127 );
        left_ok = m_motor.ForwardM1( speed );
    }
    else
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( -a_msg.left_motor(), 127 );
        left_ok = m_motor.BackwardM1( speed );
    }

    if( a_msg.right_motor() >= 0 )
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( a_msg.right_motor(), 127 );
        right_ok = m_motor.ForwardM2( speed );
    }
    else
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( -a_msg.right_motor(), 127 );
        right_ok = m_motor.BackwardM2( speed );
    }

    std::cout << " LEFT: " << left_ok << " RIGHT: " << right_ok << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::MovePosition& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    //m_motor.SpeedAccelDeccelPositionM1M2
}

void RoboClawInterface::receive( const roboclaw_msgs::RequestStatus& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::endl;
    roboclaw_msgs::MoveStatus msg;
    uint8_t status;
    bool valid;

    msg.set_left_encoder( m_motor.ReadEncM1( &status, &valid ) );
    msg.set_right_encoder( m_motor.ReadEncM2( &status, &valid ) );
    msg.set_left_speed( m_motor.ReadSpeedM1( &status, &valid ) );
    msg.set_right_speed( m_motor.ReadSpeedM2( &status, &valid ) );
    std::cout << msg.ShortDebugString() << std::endl;
    //sepia::comm2::Dispatcher< roboclaw_msgs::MoveStatus >::send( &msg );
}