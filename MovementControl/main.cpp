#include <iostream>
#include "roboclawinterface.h"

int main( int argc, char** argv )
{
    RoboClawInterface roboclaw;

    roboclaw.start();

    while( true )
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for( 1s );
    }
    roboclaw.stop();
    roboclaw.join();
    return 0;
}