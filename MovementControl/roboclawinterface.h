#include <roboclaw.h>
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/observer.h>
#include <sepia/util/threadobject.h>

class RoboClawInterface : public sepia::util::ThreadObject,
                          public sepia::comm2::Observer< roboclaw_msgs::Move >,
                          public sepia::comm2::Observer< roboclaw_msgs::MovePosition >,
                          public sepia::comm2::Observer< roboclaw_msgs::RequestStatus >
{
public:
    RoboClawInterface();
    ~RoboClawInterface();

protected:
    void receive( const roboclaw_msgs::Move& a_msg ) override;
    void receive( const roboclaw_msgs::MovePosition& a_msg ) override;
    void receive( const roboclaw_msgs::RequestStatus& a_msg ) override;
    void own_thread() override;
    RoboClaw m_motor;
};