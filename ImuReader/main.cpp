#include <iostream>
#include "imureader.h"

int main( int argc, char** argv )
{
    ImuReader imu;

    imu.start();

    while( true )
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for( 1s );
    }
    imu.stop();
    imu.join();
    return 0;
}
