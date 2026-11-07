#include <roboclaw.h>
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/observer.h>
#include <sepia/util/threadobject.h>

class RoboClawInterface : public sepia::util::ThreadObject,
                          public sepia::comm2::Observer< roboclaw_msgs::Move >,
                          public sepia::comm2::Observer< roboclaw_msgs::MovePosition >,
                          public sepia::comm2::Observer< roboclaw_msgs::MoveSpeed >,
                          public sepia::comm2::Observer< roboclaw_msgs::RequestStatus >,
                          public sepia::comm2::Observer< roboclaw_msgs::MoveDuty >,
                          public sepia::comm2::Observer< roboclaw_msgs::ResetEncoders >,
                          public sepia::comm2::Observer< roboclaw_msgs::SetEncoders >,
                          public sepia::comm2::Observer< roboclaw_msgs::RequestSettings >,
                          public sepia::comm2::Observer< roboclaw_msgs::WriteSettings >,
                          public sepia::comm2::Observer< roboclaw_msgs::SaveSettings >,
                          public sepia::comm2::Observer< roboclaw_msgs::ReloadSettings >,
                          public sepia::comm2::Observer< roboclaw_msgs::RestoreDefaultSettings >
{
public:
    RoboClawInterface();
    ~RoboClawInterface();

protected:
    void receive( const roboclaw_msgs::Move& a_msg ) override;
    void receive( const roboclaw_msgs::MovePosition& a_msg ) override;
    void receive( const roboclaw_msgs::MoveSpeed& a_msg ) override;
    void receive( const roboclaw_msgs::RequestStatus& a_msg ) override;
    void receive( const roboclaw_msgs::MoveDuty& a_msg ) override;
    void receive( const roboclaw_msgs::ResetEncoders& a_msg ) override;
    void receive( const roboclaw_msgs::SetEncoders& a_msg ) override;
    void receive( const roboclaw_msgs::RequestSettings& a_msg ) override;
    void receive( const roboclaw_msgs::WriteSettings& a_msg ) override;
    void receive( const roboclaw_msgs::SaveSettings& a_msg ) override;
    void receive( const roboclaw_msgs::ReloadSettings& a_msg ) override;
    void receive( const roboclaw_msgs::RestoreDefaultSettings& a_msg ) override;
    void own_thread() override;
    void readSettings( roboclaw_msgs::SettingsReport& a_report );
    void applySettings( const roboclaw_msgs::Settings& a_settings, roboclaw_msgs::SettingsReport& a_report );
    void sendReport( const std::string& a_result, const roboclaw_msgs::SettingsReport* a_previous = nullptr );
    RoboClaw m_motor;
};