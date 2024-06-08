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
    robotics_msgs::PS3Axis m_axisMsg;
    robotics_msgs::PS3Button m_buttonMsg;
    robotics_msgs::Axis m_left;
    robotics_msgs::Axis m_right;
};