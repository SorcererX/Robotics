#include <iostream>
#include "pathfollower.h"

int main( int argc, char** argv )
{
    PathFollower follower;

    follower.start();

    while( true )
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for( 1s );
    }
    follower.stop();
    follower.join();
    return 0;
}
