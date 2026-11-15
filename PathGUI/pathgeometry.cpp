#include "pathgeometry.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double toRadians = std::numbers::pi / 180;

    // How far the middle of the robot moves along the segment and how much the heading changes.
    struct Motion
    {
        double along;
        double turn;
    };

    Motion motion( const path_msgs::PathSegment& a_segment )
    {
        switch( a_segment.segment_case() )
        {
            case path_msgs::PathSegment::kLine:
                return { .along = a_segment.line().distance_cm(), .turn = 0 };
            case path_msgs::PathSegment::kTurn:
                return { .along = 0, .turn = a_segment.turn().angle_deg() * toRadians };
            case path_msgs::PathSegment::kArc:
            {
                const double turn = a_segment.arc().angle_deg() * toRadians;
                const double along = a_segment.arc().radius_cm() * std::abs( turn );
                return { .along = a_segment.arc().backward() ? -along : along, .turn = turn };
            }
            default:
                return { .along = 0, .turn = 0 };
        }
    }

    // The point relative to the robot, x forward and y to the left.
    std::pair< double, double > toLocal( const Pose& a_start, double a_x, double a_y )
    {
        const double dx = a_x - a_start.x;
        const double dy = a_y - a_start.y;
        return { dx * std::cos( a_start.heading ) + dy * std::sin( a_start.heading ), -dx * std::sin( a_start.heading ) + dy * std::cos( a_start.heading ) };
    }
}

Pose poseAlong( const Pose& a_start, const path_msgs::PathSegment& a_segment, double a_fraction )
{
    const auto [ along, turn ] = motion( a_segment );
    const double distance = along * a_fraction;
    const double angle = turn * a_fraction;
    if( std::abs( turn ) < 1e-9 )
    {
        return { .x = a_start.x + distance * std::cos( a_start.heading ), .y = a_start.y + distance * std::sin( a_start.heading ), .heading = a_start.heading };
    }
    // The signed radius, 0 for a turn on the spot.
    const double radius = along / turn;
    const double heading = a_start.heading + angle;
    return { .x = a_start.x + radius * ( std::sin( heading ) - std::sin( a_start.heading ) ),
             .y = a_start.y - radius * ( std::cos( heading ) - std::cos( a_start.heading ) ),
             .heading = heading };
}

std::vector< Pose > sampleSegment( const Pose& a_start, const path_msgs::PathSegment& a_segment )
{
    // A point every 5 degrees is smooth at any zoom that fits the robot's arcs.
    const int count = a_segment.has_arc() ? std::max( 2, static_cast< int >( std::abs( a_segment.arc().angle_deg() ) / 5 ) + 1 ) : 2;
    std::vector< Pose > poses;
    poses.reserve( count );
    for( int i = 0; i < count; i++ )
    {
        poses.push_back( poseAlong( a_start, a_segment, static_cast< double >( i ) / ( count - 1 ) ) );
    }
    return poses;
}

std::vector< path_msgs::PathSegment > straightTo( const Pose& a_start, double a_x, double a_y )
{
    const auto [ forward, left ] = toLocal( a_start, a_x, a_y );
    const double distance = std::hypot( forward, left );
    std::vector< path_msgs::PathSegment > segments;
    if( distance < 0.1 )
    {
        return segments;
    }
    const double angle = std::atan2( left, forward ) / toRadians;
    if( std::abs( angle ) >= 0.1 )
    {
        segments.emplace_back().mutable_turn()->set_angle_deg( angle );
    }
    segments.emplace_back().mutable_line()->set_distance_cm( distance );
    return segments;
}

std::optional< path_msgs::PathSegment > arcTo( const Pose& a_start, double a_x, double a_y )
{
    const auto [ forward, left ] = toLocal( a_start, a_x, a_y );
    const double distance = std::hypot( forward, left );
    if( distance < 0.1 )
    {
        return std::nullopt;
    }
    path_msgs::PathSegment segment;
    // Less than 0.1 degree off the heading is a line.
    if( std::abs( left ) < distance * 0.001 )
    {
        if( forward < 0 )
        {
            return std::nullopt;
        }
        segment.mutable_line()->set_distance_cm( forward );
        return segment;
    }
    // The circle that touches the heading at the robot and goes through the point. The angle between the
    // heading and the chord is half the angle of the arc.
    auto* arc = segment.mutable_arc();
    arc->set_radius_cm( distance * distance / ( 2 * std::abs( left ) ) );
    arc->set_angle_deg( 2 * std::atan2( left, forward ) / toRadians );
    return segment;
}
