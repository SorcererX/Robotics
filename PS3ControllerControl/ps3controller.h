#include "joystick.h"
#include <linux/joystick.h>
#include "robotics_msgs.pb.h"

class PS3Controller : public Joystick
{
public:
    PS3Controller();
    ~PS3Controller();

protected:
    void own_thread() override;
    void process_event( const struct js_event& e ) override;

private:
    robotics_msgs::PS3Controller m_msg;
    robotics_msgs::PressureButton m_l2;
    robotics_msgs::PressureButton m_r2;
    robotics_msgs::Axis m_left;
    robotics_msgs::Axis m_right;
};