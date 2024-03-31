#include "ps3controller.h"
#include <sepia/comm2/messagesender.h>
#include <sepia/comm2/dispatcher.h>

PS3Controller::PS3Controller()
    : Joystick( "/dev/input/js0" )
{
    m_msg.set_allocated_l2( &m_l2 );
    m_msg.set_allocated_r2( &m_r2 );
    m_msg.set_allocated_left_axis( &m_left );
    m_msg.set_allocated_right_axis( &m_right );
}

PS3Controller::~PS3Controller()
{
    stop();
    join();
}

void PS3Controller::own_thread()
{
    sepia::comm2::MessageSender::init();
    Joystick::own_thread();
    sepia::comm2::MessageSender::destroy();
}

void PS3Controller::process_event( const struct js_event& e )
{
    if( e.type == JS_EVENT_BUTTON && e.number < m_buttons )
    {
        switch( e.number )
        {
            case 0:
                m_msg.set_cross_pressed( e.value );
                break;
            case 1:
                m_msg.set_circle_pressed( e.value );
                break;
            case 2:
                m_msg.set_triangle_pressed( e.value );
                break;
            case 3:
                m_msg.set_square_pressed( e.value );
                break;
            case 4:
                m_msg.set_l1_pressed( e.value );
                break;
            case 5:
                m_msg.set_r1_pressed( e.value );
                break;
            case 6:
                m_l2.set_pressed( e.value );
                break;
            case 7:
                m_r2.set_pressed( e.value );
                break;
            case 8:
                m_msg.set_select_pressed( e.value );
                break;
            case 9:
                m_msg.set_start_pressed( e.value );
                break;
            case 10:
                m_msg.set_ps_pressed( e.value );
                break;
            case 11:
                m_msg.set_l3_pressed( e.value );
                break;
            case 12:
                m_msg.set_r3_pressed( e.value );
                break;
            case 13:
                m_msg.set_up_pressed( e.value );
                break;
            case 14:
                m_msg.set_down_pressed( e.value );
                break;
            case 15:
                m_msg.set_left_pressed( e.value );
                break;
            case 16:
                m_msg.set_right_pressed( e.value );
                break;
            default:
                return;
        }
    }
    else if( e.type == JS_EVENT_AXIS && e.number < m_axises )
    {
        switch( e.number )
        {
            case 0:
                m_left.set_horizontal( e.value );
                break;
            case 1:
                m_left.set_vertical( e.value );
                break;
            case 2:
                m_l2.set_value( e.value );
                break;
            case 3:
                m_right.set_horizontal( e.value );
                break;
            case 4:
                m_right.set_vertical( e.value );
                break;
            case 5:
                m_r2.set_value( e.value );
                break;
            default:
                return;
        }
    }

    m_msg.set_time( e.time );

    sepia::comm2::Dispatcher< robotics_msgs::PS3Controller >::send( &m_msg );
}