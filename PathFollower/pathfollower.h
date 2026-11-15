#include "imu_msgs.pb.h"
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
// MoveStatus to send the next segment before the running one ends. With ImuData, the heading after each turn
// and arc is checked, and corrected by turning on the spot, before the next segment starts.
class PathFollower : public sepia::util::ThreadObject,
                     public sepia::comm2::Observer< path_msgs::FollowPath >,
                     public sepia::comm2::Observer< path_msgs::StopPath >,
                     public sepia::comm2::Observer< roboclaw_msgs::MoveStatus >,
                     public sepia::comm2::Observer< imu_msgs::ImuData >,
                     public sepia::comm2::Observer< path_msgs::RequestFollowerSettings >,
                     public sepia::comm2::Observer< path_msgs::WriteFollowerSettings >
{
public:
    PathFollower();
    ~PathFollower();

protected:
    void receive( const path_msgs::FollowPath& a_msg ) override;
    void receive( const path_msgs::StopPath& a_msg ) override;
    void receive( const roboclaw_msgs::MoveStatus& a_msg ) override;
    void receive( const imu_msgs::ImuData& a_msg ) override;
    void receive( const path_msgs::RequestFollowerSettings& a_msg ) override;
    void receive( const path_msgs::WriteFollowerSettings& a_msg ) override;
    void own_thread() override;

private:
    using Clock = std::chrono::steady_clock;

    struct Sides
    {
        double left;
        double right;
    };
    enum class Phase
    {
        Driving,
        // Stopped after a turn or arc, waiting for the IMU to settle before checking the heading.
        Settling
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
        // The heading the path has after this step, in radians from the start of the path, and how much the
        // step turns.
        double heading;
        double turn;
        // Turns and arcs, which the belts slip on.
        bool checkHeading;
        bool isTurn;
        // The target before corrections, the steps after it are planned from here.
        Sides plannedTarget;

        Phase phase{ Phase::Driving };
        Clock::time_point settleStart{};
        // A correction is not done before it has had time to move.
        Clock::time_point notBefore{};
        // The IMU heading when the step or its last correction started, and how much that was meant to turn.
        double phaseStartHeading{ 0 };
        double phaseTurn{ 0 };
        // How much of the turn the belts give, measured on the step, so corrections allow for the slip.
        double turnRatio{ 1 };
        int corrections{ 0 };
    };

    void apply( const path_msgs::FollowerSettings& a_settings );
    path_msgs::FollowerSettings settings() const;
    void sendSettings( const std::string& a_result );
    bool active() const;
    std::optional< std::string > addSteps( const path_msgs::FollowPath& a_msg, Sides a_start, double a_startHeading );
    void sendSteps();
    void sendStep( const Step& a_step );
    bool checksHeading( Clock::time_point a_now ) const;
    double imuHeading() const;
    bool settle( Step& a_step, Clock::time_point a_now );
    void finishStep( Clock::time_point a_now );
    void updatePose( const Sides& a_position );
    void checkTimeouts();
    void end( path_msgs::PathProgress::State a_state, const std::string& a_message );
    void stopMotors( double a_deceleration );
    void sendProgress( const Step& a_step, path_msgs::PathProgress::State a_state, const std::string& a_message = {} );
    void sendProgress( const path_msgs::FollowPath& a_path, path_msgs::PathProgress::State a_state, const std::string& a_message );

    // From sepia.yaml or WriteFollowerSettings.
    double m_trackWidth{ 0 };
    double m_tolerance{ 1 };
    bool m_useImu{ true };
    // In radians.
    double m_headingTolerance{ 0 };
    int m_maxCorrections{ 3 };
    // In degrees per second.
    double m_calmRate{ 1 };
    Clock::duration m_calmTime{};
    Clock::duration m_settleTimeout{};

    // The latest ImuData, the heading in radians.
    double m_imuHeading{ 0 };
    // When the turn rate went below calmRate, unset while it is above.
    std::optional< Clock::time_point > m_imuCalmSince;
    Clock::time_point m_imuTime{};
    // The IMU heading when the path started, unset when there was no IMU data then.
    std::optional< double > m_imuStart;

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
    // The same from the belts alone.
    double m_encoderX{ 0 };
    double m_encoderY{ 0 };
    double m_encoderHeading{ 0 };
    Sides m_posePosition{};
    // For noticing a robot that has stopped short of the target.
    Sides m_lastPosition{};
    Clock::time_point m_lastMoved;
};
