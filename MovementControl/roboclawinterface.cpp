#include "roboclawinterface.h"
#include "roboclaw_msgs.pb.h"

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
    sepia::comm2::ObserverBase::initReceiver();
    sepia::comm2::Observer< roboclaw_msgs::Move >::initReceiver();
    while( !m_terminate && sepia::comm2::ObserverBase::threadReceiver() )
    {
    }
    sepia::comm2::ObserverBase::destroyReceiver();
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
