#include <iostream>
#include "ps3controller.h"
#include "translator.h"

int main( int argc, char** argv )
{
    Translator translator;
    PS3Controller controller;

    while( !controller.init() )
    {
        std::cout << "Unable to initialize controller. Waiting 1 second before retrying ..." << std::endl;
        using namespace std::chrono_literals;
        std::this_thread::sleep_for( 1s );
    }

    translator.start();
    controller.start();

    while( true )
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for( 1s );
    }
    controller.stop();
    translator.stop();
    controller.join();
    translator.join();
}
