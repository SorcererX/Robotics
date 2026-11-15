#include "pathfollower.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/receiver.h>
#include <sepia/comm2/scopedmessagesender.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <yaml-cpp/yaml.h>

namespace
{
    using namespace std::chrono_literals;

    // How often MoveStatus is requested while a path runs.
    constexpr auto statusInterval = 50ms;
    // A path fails when MovementControl does not answer for this long.
    constexpr auto statusTimeout = 1s;
    // A path fails when neither belt moves for this long before the target is reached.
    constexpr auto stallTimeout = 3s;
    // Belt movement in cm that counts as moving.
    constexpr double stallDistance = 0.1;
    // Belt speed in cm/s that counts as stopped at the end of a path.
    constexpr double stoppedSpeed = 1;
    // Used by StopPath when no path has run yet.
    constexpr double defaultDeceleration = 50;
    // The running step and the next one are in the controller's buffer, so the next segment starts as soon as
    // the running one ends instead of when the status shows it.
    constexpr size_t queuedSteps = 2;
    // ImuData older than this is not used.
    constexpr auto imuTimeout = 500ms;
    // After a turn or arc has stopped, the heading is checked when the turn rate has stayed below calmRate, in
    // degrees per second, for calmTime, or after settleTimeout.
    constexpr double calmRate = 1;
    constexpr auto calmTime = 200ms;
    constexpr auto settleTimeout = 1500ms;
    // A correction cannot count as done before it has started, which is certain after half its expected time, or
    // after this long for a short one since the message takes time to reach the controller.
    constexpr auto correctionStart = 150ms;
    // The range the measured share of a turn is kept within, so one odd measurement does not give a wild correction.
    constexpr double minimumTurnRatio = 0.3;
    constexpr double maximumTurnRatio = 1.5;
    // Turns smaller than this in radians are too small to measure the slip on.
    constexpr double minimumMeasuredTurn = 2 * std::numbers::pi / 180;

    struct Config
    {
        double trackWidth = 0;
        double tolerance = 1;
        bool useImu = true;
        double headingTolerance = 1;
        int maxCorrections = 3;
    };

    // Reads path_follower from the same file SepiaComm uses.
    Config loadConfig()
    {
        const char* configured = std::getenv( "SEPIA_CONFIG" );
        const std::string path = configured != nullptr && *configured != '\0' ? configured : "sepia.yaml";
        Config config;
        std::error_code ec;
        if( std::filesystem::exists( path, ec ) )
        {
            try
            {
                if( const YAML::Node node = YAML::LoadFile( path )[ "path_follower" ] )
                {
                    config.trackWidth = node[ "track_width_cm" ].as< double >( config.trackWidth );
                    config.tolerance = node[ "tolerance_cm" ].as< double >( config.tolerance );
                    config.useImu = node[ "use_imu" ].as< bool >( config.useImu );
                    config.headingTolerance = node[ "heading_tolerance_deg" ].as< double >( config.headingTolerance );
                    config.maxCorrections = node[ "max_heading_corrections" ].as< int >( config.maxCorrections );
                }
            }
            catch( const YAML::Exception& ex )
            {
                std::cerr << "Unable to read path_follower from '" << path << "': " << ex.what() << std::endl;
            }
        }
        if( !( config.tolerance > 0 ) )
        {
            std::cerr << "tolerance_cm must be above 0, using 1." << std::endl;
            config.tolerance = 1;
        }
        if( !( config.headingTolerance > 0 ) )
        {
            std::cerr << "heading_tolerance_deg must be above 0, using 1." << std::endl;
            config.headingTolerance = 1;
        }
        config.maxCorrections = std::max( config.maxCorrections, 0 );
        return config;
    }

    double toRadians( double a_degrees )
    {
        return a_degrees * std::numbers::pi / 180;
    }

    double toDegrees( double a_radians )
    {
        return a_radians * 180 / std::numbers::pi;
    }
}

PathFollower::PathFollower()
{
    const Config config = loadConfig();
    m_trackWidth = config.trackWidth;
    m_tolerance = config.tolerance;
    m_useImu = config.useImu;
    m_headingTolerance = toRadians( config.headingTolerance );
    m_maxCorrections = config.maxCorrections;
    if( m_trackWidth > 0 )
    {
        std::cout << "Track width: " << m_trackWidth << " cm, tolerance: " << m_tolerance << " cm" << std::endl;
    }
    else
    {
        std::cout << "path_follower.track_width_cm is not set in sepia.yaml, only lines can be driven. Tolerance: " << m_tolerance << " cm"
                  << std::endl;
    }
    if( m_useImu )
    {
        std::cout << "Turns and arcs are checked with ImuData, heading tolerance: " << config.headingTolerance << " degrees, at most " << m_maxCorrections
                  << " corrections" << std::endl;
    }
}

PathFollower::~PathFollower()
{
}

void PathFollower::own_thread()
{
    sepia::comm2::ScopedMessageSender sender;
    // Returns regularly without messages, so the status can be requested.
    sepia::comm2::Receiver handler( this, 20 );
    while( !m_terminate && handler.exec() )
    {
        checkTimeouts();
    }
}

bool PathFollower::active() const
{
    return !m_steps.empty() || !m_pending.empty();
}

void PathFollower::receive( const path_msgs::FollowPath& a_msg )
{
    std::cout << "FollowPath " << a_msg.ShortDebugString() << std::endl;
    if( a_msg.append() && active() )
    {
        // A path still waiting for the position starts after the others.
        if( !m_pending.empty() )
        {
            m_pending.push_back( a_msg );
            return;
        }
        if( const auto error = addSteps( a_msg, m_steps.back().plannedTarget, m_steps.back().heading ) )
        {
            sendProgress( a_msg, path_msgs::PathProgress::FAILED, *error );
            return;
        }
        sendSteps();
        return;
    }

    if( active() )
    {
        end( path_msgs::PathProgress::STOPPED, "replaced by path " + std::to_string( a_msg.id() ) );
    }
    // The path starts from the position in the next MoveStatus.
    m_pending.push_back( a_msg );
    m_clearBuffer = true;
    m_started = Clock::now();
    m_lastRequest = {};
    checkTimeouts();
}

void PathFollower::receive( const path_msgs::StopPath& a_msg )
{
    std::cout << "StopPath " << a_msg.ShortDebugString() << std::endl;
    const double deceleration = a_msg.deceleration_cm_s2() > 0 ? a_msg.deceleration_cm_s2() : m_deceleration > 0 ? m_deceleration : defaultDeceleration;
    // Also stops motors that were started by something else.
    stopMotors( deceleration );
    if( active() )
    {
        end( path_msgs::PathProgress::STOPPED, "stopped by StopPath" );
    }
}

void PathFollower::receive( const imu_msgs::ImuData& a_msg )
{
    const auto now = Clock::now();
    m_imuHeading = toRadians( a_msg.heading_deg() );
    if( std::abs( a_msg.yaw_rate_deg_s() ) >= calmRate )
    {
        m_imuCalmSince.reset();
    }
    else if( !m_imuCalmSince )
    {
        m_imuCalmSince = now;
    }
    m_imuTime = now;
}

void PathFollower::receive( const roboclaw_msgs::MoveStatus& a_msg )
{
    const auto now = Clock::now();
    m_lastStatus = now;
    const Sides position{ .left = a_msg.left_position_cm(), .right = a_msg.right_position_cm() };

    if( !m_pending.empty() )
    {
        // Appended paths keep the pose of the path they were appended to.
        if( m_steps.empty() )
        {
            m_x = m_y = m_heading = 0;
            m_posePosition = position;
            const bool imu = now - m_imuTime < imuTimeout;
            m_imuStart = imu ? std::optional( m_imuHeading ) : std::nullopt;
            if( m_useImu && !imu )
            {
                std::cout << "No ImuData, turns and arcs are not checked on this path." << std::endl;
            }
        }
        const auto pending = std::move( m_pending );
        m_pending.clear();
        for( const auto& path : pending )
        {
            const Sides start = m_steps.empty() ? position : m_steps.back().plannedTarget;
            const double heading = m_steps.empty() ? 0 : m_steps.back().heading;
            if( const auto error = addSteps( path, start, heading ) )
            {
                sendProgress( path, path_msgs::PathProgress::FAILED, *error );
            }
        }
        m_lastPosition = position;
        m_lastMoved = now;
        sendSteps();
    }
    if( m_steps.empty() )
    {
        return;
    }
    updatePose( position );

    if( std::abs( position.left - m_lastPosition.left ) > stallDistance || std::abs( position.right - m_lastPosition.right ) > stallDistance )
    {
        m_lastPosition = position;
        m_lastMoved = now;
    }

    const bool stopped = std::abs( a_msg.left_speed_cm_s() ) < stoppedSpeed && std::abs( a_msg.right_speed_cm_s() ) < stoppedSpeed;
    while( !m_steps.empty() )
    {
        Step& step = m_steps.front();
        if( step.phase == Phase::Settling )
        {
            if( !settle( step, now ) )
            {
                break;
            }
            finishStep( now );
            continue;
        }

        // The belt that moves the furthest decides when the step is done, the other may hardly move.
        const bool left = std::abs( step.target.left - step.start.left ) >= std::abs( step.target.right - step.start.right );
        const double target = left ? step.target.left : step.target.right;
        const double direction = target >= ( left ? step.start.left : step.start.right ) ? 1 : -1;
        const double remaining = ( target - ( left ? position.left : position.right ) ) * direction;
        // A step with nothing queued after it waits for the robot to stop, the others hand over to the next step.
        if( remaining > m_tolerance || ( m_sent == 1 && !stopped ) || now < step.notBefore )
        {
            break;
        }
        // The next step is held back after a turn or arc, so the heading can be checked first.
        if( step.checkHeading && m_sent == 1 && checksHeading( now ) )
        {
            step.phase = Phase::Settling;
            step.settleStart = now;
            break;
        }
        finishStep( now );
    }
    if( m_steps.empty() )
    {
        m_clearBuffer = true;
    }
    sendSteps();
}

std::optional< std::string > PathFollower::addSteps( const path_msgs::FollowPath& a_msg, Sides a_start, double a_startHeading )
{
    if( !std::isfinite( a_msg.speed_cm_s() ) || a_msg.speed_cm_s() <= 0 )
    {
        return "speed_cm_s must be above 0";
    }
    if( !std::isfinite( a_msg.acceleration_cm_s2() ) || a_msg.acceleration_cm_s2() <= 0 )
    {
        return "acceleration_cm_s2 must be above 0";
    }

    // All segments are checked before any is added, so a bad path does not run halfway.
    std::vector< Step > steps;
    Sides position = a_start;
    double heading = a_startHeading;
    const auto count = static_cast< uint32_t >( a_msg.segments_size() );
    for( uint32_t i = 0; i < count; i++ )
    {
        const path_msgs::PathSegment& segment = a_msg.segments( static_cast< int >( i ) );
        const std::string name = "segment " + std::to_string( i );
        // The belts move the same distance plus or minus the turn, the left belt is on the inside of a left turn.
        Sides distance{};
        double angle = 0;
        switch( segment.segment_case() )
        {
            case path_msgs::PathSegment::kLine:
                distance = { .left = segment.line().distance_cm(), .right = segment.line().distance_cm() };
                break;
            case path_msgs::PathSegment::kTurn:
            {
                if( !( m_trackWidth > 0 ) )
                {
                    return name + ": turns need path_follower.track_width_cm in sepia.yaml";
                }
                angle = toRadians( segment.turn().angle_deg() );
                const double turn = angle * m_trackWidth / 2;
                distance = { .left = -turn, .right = turn };
                break;
            }
            case path_msgs::PathSegment::kArc:
            {
                if( !( m_trackWidth > 0 ) )
                {
                    return name + ": arcs need path_follower.track_width_cm in sepia.yaml";
                }
                if( segment.arc().radius_cm() < 0 )
                {
                    return name + ": radius_cm must not be negative";
                }
                angle = toRadians( segment.arc().angle_deg() );
                const double along = segment.arc().radius_cm() * std::abs( angle ) * ( segment.arc().backward() ? -1 : 1 );
                const double turn = angle * m_trackWidth / 2;
                distance = { .left = along - turn, .right = along + turn };
                break;
            }
            default:
                return name + " has no line, turn or arc";
        }
        if( !std::isfinite( distance.left ) || !std::isfinite( distance.right ) || !std::isfinite( segment.speed_cm_s() ) )
        {
            return name + " is not a number";
        }

        const double longest = std::max( std::abs( distance.left ), std::abs( distance.right ) );
        if( longest < 0.01 )
        {
            continue;
        }
        heading += angle;
        // The shorter belt gets a slower profile with the same shape, so both finish together.
        const double speed = segment.speed_cm_s() > 0 ? segment.speed_cm_s() : a_msg.speed_cm_s();
        const Sides share{ .left = std::abs( distance.left ) / longest, .right = std::abs( distance.right ) / longest };
        const Sides target{ .left = position.left + distance.left, .right = position.right + distance.right };
        steps.push_back( { .pathId = a_msg.id(),
                           .segment = i,
                           .segmentCount = count,
                           .lastOfPath = false,
                           .start = position,
                           .target = target,
                           .speed = { .left = speed * share.left, .right = speed * share.right },
                           .acceleration = { .left = a_msg.acceleration_cm_s2() * share.left, .right = a_msg.acceleration_cm_s2() * share.right },
                           .heading = heading,
                           .turn = angle,
                           .checkHeading = !segment.has_line(),
                           .isTurn = segment.has_turn(),
                           .plannedTarget = target } );
        position = target;
    }

    if( steps.empty() )
    {
        sendProgress( a_msg, path_msgs::PathProgress::DONE, "nothing to drive" );
        return std::nullopt;
    }
    steps.back().lastOfPath = true;
    m_steps.insert( m_steps.end(), steps.begin(), steps.end() );
    m_deceleration = a_msg.acceleration_cm_s2();
    return std::nullopt;
}

void PathFollower::sendSteps()
{
    while( m_sent < m_steps.size() && m_sent < queuedSteps )
    {
        // Nothing is queued after a turn or arc whose heading will be checked.
        if( m_sent > 0 && m_steps[ m_sent - 1 ].checkHeading && checksHeading( Clock::now() ) )
        {
            break;
        }
        Step& step = m_steps[ m_sent ];
        sendStep( step );
        if( m_sent == 0 )
        {
            step.phaseStartHeading = imuHeading();
            step.phaseTurn = step.turn;
            sendProgress( step, path_msgs::PathProgress::RUNNING );
        }
        m_sent++;
    }
}

void PathFollower::sendStep( const Step& a_step )
{
    roboclaw_msgs::MovePositionCm msg;
    msg.set_left_speed_cm_s( a_step.speed.left );
    msg.set_left_acceleration_cm_s2( a_step.acceleration.left );
    msg.set_left_deceleration_cm_s2( a_step.acceleration.left );
    msg.set_left_position_cm( a_step.target.left );
    msg.set_right_speed_cm_s( a_step.speed.right );
    msg.set_right_acceleration_cm_s2( a_step.acceleration.right );
    msg.set_right_deceleration_cm_s2( a_step.acceleration.right );
    msg.set_right_position_cm( a_step.target.right );
    msg.set_clear_buffer( m_clearBuffer );
    m_clearBuffer = false;
    sepia::comm2::Dispatcher< roboclaw_msgs::MovePositionCm >::send( &msg );
}

bool PathFollower::checksHeading( Clock::time_point a_now ) const
{
    return m_useImu && m_imuStart && m_trackWidth > 0 && a_now - m_imuTime < imuTimeout;
}

double PathFollower::imuHeading() const
{
    return m_imuStart ? m_imuHeading - *m_imuStart : 0;
}

bool PathFollower::settle( Step& a_step, Clock::time_point a_now )
{
    // Waiting for the IMU is not a stall.
    m_lastMoved = a_now;
    // Calm counts from when the belts stopped, the robot can still rock on them after that.
    const bool calm = m_imuCalmSince && a_now - std::max( *m_imuCalmSince, a_step.settleStart ) >= calmTime;
    if( !calm && a_now - a_step.settleStart < settleTimeout )
    {
        return false;
    }

    const double heading = imuHeading();
    const double error = a_step.heading - heading;
    const std::string name = "Path " + std::to_string( a_step.pathId ) + " segment " + std::to_string( a_step.segment );
    // How much of the turn the belts gave, so a correction allows for the same slip.
    const double turned = heading - a_step.phaseStartHeading;
    if( std::abs( a_step.phaseTurn ) >= minimumMeasuredTurn && std::abs( turned ) >= minimumMeasuredTurn )
    {
        a_step.turnRatio = std::clamp( turned / a_step.phaseTurn, minimumTurnRatio, maximumTurnRatio );
        if( a_step.isTurn && a_step.corrections == 0 )
        {
            std::cout << name << ": turn of " << toDegrees( a_step.phaseTurn ) << " degrees turned " << toDegrees( turned ) << ", a track width of "
                      << m_trackWidth * a_step.phaseTurn / turned << " cm would have turned it exactly." << std::endl;
        }
    }

    if( std::abs( error ) <= m_headingTolerance )
    {
        std::cout << name << ": heading off by " << toDegrees( error ) << " degrees after " << a_step.corrections << " corrections." << std::endl;
        return true;
    }
    if( a_step.corrections >= m_maxCorrections )
    {
        std::cout << name << ": heading still off by " << toDegrees( error ) << " degrees after " << a_step.corrections << " corrections, continuing."
                  << std::endl;
        return true;
    }

    // Turns on the spot from the target, so the belt positions stay planned instead of following where they stopped.
    const double turn = error / a_step.turnRatio;
    const double belt = turn * m_trackWidth / 2;
    const double speed = std::max( a_step.speed.left, a_step.speed.right );
    const double acceleration = std::max( a_step.acceleration.left, a_step.acceleration.right );
    a_step.start = a_step.target;
    a_step.target = { .left = a_step.target.left - belt, .right = a_step.target.right + belt };
    a_step.speed = { .left = speed, .right = speed };
    a_step.acceleration = { .left = acceleration, .right = acceleration };
    a_step.phaseStartHeading = heading;
    a_step.phaseTurn = turn;
    a_step.corrections++;
    a_step.phase = Phase::Driving;
    // Accelerating and braking take as long as the whole move when it is too short to reach the speed.
    const double distance = std::abs( belt );
    const double seconds = distance < speed * speed / acceleration ? 2 * std::sqrt( distance / acceleration ) : distance / speed + speed / acceleration;
    const auto half = std::chrono::duration_cast< Clock::duration >( std::chrono::duration< double >( seconds / 2 ) );
    a_step.notBefore = a_now + std::max( half, std::chrono::duration_cast< Clock::duration >( correctionStart ) );
    std::cout << name << ": heading off by " << toDegrees( error ) << " degrees, correcting by " << toDegrees( turn ) << std::endl;
    sendStep( a_step );
    return false;
}

void PathFollower::finishStep( Clock::time_point a_now )
{
    const Step step = m_steps.front();
    if( step.lastOfPath )
    {
        sendProgress( step, path_msgs::PathProgress::DONE );
    }
    m_steps.pop_front();
    m_sent = m_sent > 0 ? m_sent - 1 : 0;
    m_lastMoved = a_now;

    // Corrections moved the belts, so the steps that are not sent yet start from where the corrections ended.
    if( step.corrections > 0 )
    {
        const Sides shift{ .left = step.target.left - step.plannedTarget.left, .right = step.target.right - step.plannedTarget.right };
        for( size_t i = m_sent; i < m_steps.size(); i++ )
        {
            for( Sides* sides : { &m_steps[ i ].start, &m_steps[ i ].target, &m_steps[ i ].plannedTarget } )
            {
                sides->left += shift.left;
                sides->right += shift.right;
            }
        }
    }
    if( !m_steps.empty() && m_sent > 0 )
    {
        m_steps.front().phaseStartHeading = imuHeading();
        m_steps.front().phaseTurn = m_steps.front().turn;
        sendProgress( m_steps.front(), path_msgs::PathProgress::RUNNING );
    }
}

void PathFollower::updatePose( const Sides& a_position )
{
    const double left = a_position.left - m_posePosition.left;
    const double right = a_position.right - m_posePosition.right;
    m_posePosition = a_position;
    // The IMU sees the turns the belts slip on. Without it, or without the track width, the belts give the turn.
    const double along = ( left + right ) / 2;
    const bool imu = m_imuStart && Clock::now() - m_imuTime < imuTimeout;
    const double turn = imu ? imuHeading() - m_heading : m_trackWidth > 0 ? ( right - left ) / m_trackWidth : 0;
    const double heading = m_heading + turn / 2;
    m_x += along * std::cos( heading );
    m_y += along * std::sin( heading );
    m_heading += turn;

    path_msgs::PathPose msg;
    msg.set_id( m_steps.front().pathId );
    msg.set_x_cm( m_x );
    msg.set_y_cm( m_y );
    msg.set_heading_deg( m_heading * 180 / std::numbers::pi );
    sepia::comm2::Dispatcher< path_msgs::PathPose >::send( &msg );
}

void PathFollower::checkTimeouts()
{
    if( !active() )
    {
        return;
    }
    const auto now = Clock::now();
    if( now - m_lastRequest >= statusInterval )
    {
        roboclaw_msgs::RequestStatus msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestStatus >::send( &msg );
        m_lastRequest = now;
    }

    if( now - std::max( m_lastStatus, m_started ) > statusTimeout )
    {
        stopMotors( m_deceleration > 0 ? m_deceleration : defaultDeceleration );
        end( path_msgs::PathProgress::FAILED, "no MoveStatus from MovementControl" );
    }
    else if( !m_steps.empty() && now - m_lastMoved > stallTimeout )
    {
        const Step& step = m_steps.front();
        const std::string message = "not moving, target left " + std::to_string( step.target.left ) + " right " + std::to_string( step.target.right ) +
                                    " position left " + std::to_string( m_lastPosition.left ) + " right " + std::to_string( m_lastPosition.right );
        stopMotors( m_deceleration );
        end( path_msgs::PathProgress::FAILED, message );
    }
}

void PathFollower::end( path_msgs::PathProgress::State a_state, const std::string& a_message )
{
    // Each path is reported once, at the segment it got to.
    std::optional< uint32_t > reported;
    for( const Step& step : m_steps )
    {
        if( step.pathId != reported )
        {
            sendProgress( step, a_state, a_message );
            reported = step.pathId;
        }
    }
    for( const auto& path : m_pending )
    {
        sendProgress( path, a_state, a_message );
    }
    m_steps.clear();
    m_pending.clear();
    m_sent = 0;
    m_clearBuffer = true;
}

void PathFollower::stopMotors( double a_deceleration )
{
    roboclaw_msgs::MoveSpeedCm msg;
    msg.set_acceleration_cm_s2( a_deceleration );
    sepia::comm2::Dispatcher< roboclaw_msgs::MoveSpeedCm >::send( &msg );
}

void PathFollower::sendProgress( const Step& a_step, path_msgs::PathProgress::State a_state, const std::string& a_message )
{
    path_msgs::PathProgress msg;
    msg.set_id( a_step.pathId );
    msg.set_state( a_state );
    msg.set_segment( a_step.segment );
    msg.set_segment_count( a_step.segmentCount );
    msg.set_message( a_message );
    std::cout << "PathProgress " << msg.ShortDebugString() << std::endl;
    sepia::comm2::Dispatcher< path_msgs::PathProgress >::send( &msg );
}

void PathFollower::sendProgress( const path_msgs::FollowPath& a_path, path_msgs::PathProgress::State a_state, const std::string& a_message )
{
    path_msgs::PathProgress msg;
    msg.set_id( a_path.id() );
    msg.set_state( a_state );
    msg.set_segment_count( static_cast< uint32_t >( a_path.segments_size() ) );
    msg.set_message( a_message );
    std::cout << "PathProgress " << msg.ShortDebugString() << std::endl;
    sepia::comm2::Dispatcher< path_msgs::PathProgress >::send( &msg );
}
