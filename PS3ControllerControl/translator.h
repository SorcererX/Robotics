#include <sepia/util/threadobject.h>
#include <sepia/comm2/observer.h>
#include "robotics_msgs.pb.h"

class Translator : public sepia::util::ThreadObject,
                   public sepia::comm2::Observer< robotics_msgs::PS3Axis >,
                   public sepia::comm2::Observer< robotics_msgs::PS3Button >
{
public:
    Translator();
    ~Translator();
    void receive( const robotics_msgs::PS3Axis& a_msg ) override;
    void receive( const robotics_msgs::PS3Button& a_msg ) override;

protected:
    void own_thread() override;
    int32_t m_scalingFactor{ -31 };
};