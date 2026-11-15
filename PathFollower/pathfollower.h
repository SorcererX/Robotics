#include "path_msgs.pb.h"
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/observer.h>
#include <sepia/util/threadobject.h>
#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <vector>

// Turns the segments of FollowPath into absolute position moves for MovementControl, and follows them with
// MoveStatus to send the next segment before the running one ends.
class PathFollower : public sepia::util::ThreadObject,
                     public sepia::comm2::Observer< path_msgs::FollowPath >,
                     public sepia::comm2::Observer< path_msgs::StopPath >,
                     public sepia::comm2::Observer< roboclaw_msgs::MoveStatus >
{
public:
    PathFollower();
    ~PathFollower();

protected:
    void receive( const path_msgs::FollowPath& a_msg ) override;
    void receive( const path_msgs::StopPath& a_msg ) override;
    void receive( const roboclaw_msgs::MoveStatus& a_msg ) override;
    void own_thread() override;

private:
    using Clock = std::chrono::steady_clock;

    struct Sides
    {
        double left;
        double right;
    };
    // One segment as belt positions in cm, measured the same way as MoveStatus.
    struct Step
    {
        uint32_t pathId;
        uint32_t segment;
        uint32_t segmentCount;
        bool lastOfPath;
        Sides start;
        Sides target;
        Sides speed;
        Sides acceleration;
    };

    bool active() const;
    std::optional< std::string > addSteps( const path_msgs::FollowPath& a_msg, Sides a_start );
    void sendSteps();
    void updatePose( const Sides& a_position );
    void checkTimeouts();
    void end( path_msgs::PathProgress::State a_state, const std::string& a_message );
    void stopMotors( double a_deceleration );
    void sendProgress( const Step& a_step, path_msgs::PathProgress::State a_state, const std::string& a_message = {} );
    void sendProgress( const path_msgs::FollowPath& a_path, path_msgs::PathProgress::State a_state, const std::string& a_message );

    // From sepia.yaml.
    double m_trackWidth{ 0 };
    double m_tolerance{ 1 };

    std::deque< Step > m_steps;
    // How many of the steps at the front have been sent to MovementControl.
    size_t m_sent{ 0 };
    // The first sent step replaces whatever MovementControl is doing.
    bool m_clearBuffer{ true };
    // Paths waiting for a new position to start from.
    std::vector< path_msgs::FollowPath > m_pending;
    // For stopping the motors when the paths end early.
    double m_deceleration{ 0 };

    Clock::time_point m_lastRequest;
    Clock::time_point m_lastStatus;
    Clock::time_point m_started;
    // The pose since the path started, in cm and radians, and the belt positions it was last updated with.
    double m_x{ 0 };
    double m_y{ 0 };
    double m_heading{ 0 };
    Sides m_posePosition{};
    // For noticing a robot that has stopped short of the target.
    Sides m_lastPosition{};
    Clock::time_point m_lastMoved;
};
