#include "joystick.h"
#include <fcntl.h>
#include <unistd.h>

Joystick::Joystick( const std::string& a_device )
    : m_device( a_device )
{
}

Joystick::~Joystick()
{
    stop();
    join();
}

bool Joystick::init()
{
    m_fd = open( m_device.c_str(), O_RDONLY );

    if( m_fd < 0 )
    {
        m_axises = 0;
        m_buttons = 0;
        return false;
    }

    char number_of_axes;
    char number_of_buttons;

    ioctl( m_fd, JSIOCGAXES, &number_of_axes );
    ioctl( m_fd, JSIOCGBUTTONS, &number_of_buttons );
    m_buttons = number_of_buttons;
    m_axises = number_of_axes;
    return true;
}

void Joystick::own_thread()
{
    struct js_event e;
    while( !m_terminate && read( m_fd, &e, sizeof( e ) ) > 0 )
    {
        process_event( e );
    }
}
