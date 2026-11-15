#ifndef PATH_GEOMETRY_H
#define PATH_GEOMETRY_H
#include "path_msgs.pb.h"
#include <optional>
#include <vector>

// Where the middle of the robot is, in cm with x forward and y to the left of where the path starts. The
// heading is in radians, counter-clockwise from x.
struct Pose
{
    double x;
    double y;
    double heading;
};

// Where the robot is after a_fraction (0 to 1) of the segment.
Pose poseAlong( const Pose& a_start, const path_msgs::PathSegment& a_segment, double a_fraction );

// Points along the segment for drawing, including both ends.
std::vector< Pose > sampleSegment( const Pose& a_start, const path_msgs::PathSegment& a_segment );

// Turns towards the point and drives to it.
std::vector< path_msgs::PathSegment > straightTo( const Pose& a_start, double a_x, double a_y );

// The arc that starts in the current heading and ends at the point, or a line when the point is straight
// ahead. Nothing when the point is where the robot is or straight behind it.
std::optional< path_msgs::PathSegment > arcTo( const Pose& a_start, double a_x, double a_y );

#endif // PATH_GEOMETRY_H
