#include "roboclawinterface.h"
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/receiver.h>
#include <sepia/comm2/scopedmessagesender.h>
#include <algorithm>

RoboClawInterface::RoboClawInterface()
    : m_motor( "/dev/ttyACM0", 5, 0x80 )
{
    m_motor.begin( B115200 );

    const auto printVelocityPID = []( const char* a_name, const auto& a_pid )
    {
        if( a_pid )
        {
            std::cout << a_name << " velocity PID P: " << a_pid->kp << " I: " << a_pid->ki << " D: " << a_pid->kd << " QPPS: " << a_pid->qpps << std::endl;
        }
        else
        {
            std::cout << a_name << " velocity PID could not be read." << std::endl;
        }
    };
    printVelocityPID( "M1", m_motor.readM1VelocityPID() );
    printVelocityPID( "M2", m_motor.readM2VelocityPID() );
}

RoboClawInterface::~RoboClawInterface()
{
    m_motor.end();
}

void RoboClawInterface::own_thread()
{
    sepia::comm2::ScopedMessageSender sender;
    sepia::comm2::Receiver handler( this );
    while( !m_terminate && handler.exec() )
    {
    }
}

void RoboClawInterface::receive( const roboclaw_msgs::Move& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::flush;

    bool left_ok = false;
    bool right_ok = false;

    if( a_msg.left_motor() >= 0 )
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( a_msg.left_motor(), 127 );
        left_ok = m_motor.forwardM1( speed );
    }
    else
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( -a_msg.left_motor(), 127 );
        left_ok = m_motor.backwardM1( speed );
    }

    if( a_msg.right_motor() >= 0 )
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( a_msg.right_motor(), 127 );
        right_ok = m_motor.forwardM2( speed );
    }
    else
    {
        // 0 is full stop, valid range is 0-127
        uint8_t speed = std::min( -a_msg.right_motor(), 127 );
        right_ok = m_motor.backwardM2( speed );
    }

    std::cout << " LEFT: " << left_ok << " RIGHT: " << right_ok << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::MovePosition& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::flush;
    bool result = m_motor.speedAccelDeccelPositionM1M2(
        a_msg.left_acceleration(),
        a_msg.left_speed(),
        a_msg.left_deceleration(),
        a_msg.left_distance(),
        a_msg.right_acceleration(),
        a_msg.right_speed(),
        a_msg.right_deceleration(),
        a_msg.right_distance(),
        a_msg.clear_buffer() );

    std::cout << " Move Position result: " << result << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::MoveSpeed& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::flush;
    bool result = m_motor.speedAccelM1M2( a_msg.acceleration(), a_msg.left_speed(), a_msg.right_speed() );

    std::cout << " Move Speed result: " << result << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::RequestStatus& a_msg )
{
    const auto encoders = m_motor.readEncoders();
    const auto speeds = m_motor.readSpeeds();

    // A failed read is not sent as 0, the GUI keeps showing the last status instead.
    if( !encoders || !speeds )
    {
        std::cout << "Status could not be read." << std::endl;
        return;
    }

    roboclaw_msgs::MoveStatus msg;
    msg.set_left_encoder( static_cast< int32_t >( encoders->m1 ) );
    msg.set_right_encoder( static_cast< int32_t >( encoders->m2 ) );
    msg.set_left_speed( speeds->m1 );
    msg.set_right_speed( speeds->m2 );

    // The rest is optional, so one failed read does not hide the others.
    if( const auto error = m_motor.readError() )
    {
        msg.set_error( *error );
    }
    if( const auto currents = m_motor.readCurrents() )
    {
        msg.set_left_current( currents->m1 );
        msg.set_right_current( currents->m2 );
    }
    if( const auto voltage = m_motor.readMainBatteryVoltage() )
    {
        msg.set_main_battery_voltage( *voltage );
    }
    if( const auto pwms = m_motor.readPWMs() )
    {
        msg.set_left_pwm( pwms->m1 );
        msg.set_right_pwm( pwms->m2 );
    }
    if( const auto temperature = m_motor.readTemp() )
    {
        msg.set_temperature( *temperature );
    }
    if( const auto errors = m_motor.readSpeedErrors() )
    {
        msg.set_left_speed_error( errors->m1 );
        msg.set_right_speed_error( errors->m2 );
    }
    if( const auto errors = m_motor.readPosErrors() )
    {
        msg.set_left_position_error( errors->m1 );
        msg.set_right_position_error( errors->m2 );
    }
    if( const auto buffers = m_motor.readBuffers() )
    {
        msg.set_left_buffer( buffers->m1 );
        msg.set_right_buffer( buffers->m2 );
    }
    sepia::comm2::Dispatcher< roboclaw_msgs::MoveStatus >::send( &msg );
}

void RoboClawInterface::receive( const roboclaw_msgs::MoveDuty& a_msg )
{
    std::cout << a_msg.ShortDebugString() << std::flush;
    // The duty is sent as two's complement.
    const auto duty = []( int32_t a_duty ) { return static_cast< uint16_t >( std::clamp( a_duty, -32767, 32767 ) ); };
    bool result = m_motor.dutyAccelM1M2( duty( a_msg.left_duty() ), a_msg.left_acceleration(), duty( a_msg.right_duty() ), a_msg.right_acceleration() );

    std::cout << " Move Duty result: " << result << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::ResetEncoders& a_msg )
{
    std::cout << "Reset encoders result: " << m_motor.resetEncoders() << std::endl;
}

void RoboClawInterface::receive( const roboclaw_msgs::SetEncoders& a_msg )
{
    const bool left = m_motor.setEncM1( a_msg.left_encoder() );
    const bool right = m_motor.setEncM2( a_msg.right_encoder() );
    std::cout << a_msg.ShortDebugString() << " LEFT: " << left << " RIGHT: " << right << std::endl;
}

namespace
{
    void check( bool a_ok, const char* a_name, roboclaw_msgs::SettingsReport& a_report )
    {
        if( !a_ok )
        {
            a_report.add_failed( a_name );
        }
    }

    // The gains are stored as fixed point, so what is read back is only this close to what was written.
    bool sameGain( float a_left, float a_right )
    {
        return std::abs( a_left - a_right ) < 0.00002f;
    }
}

void RoboClawInterface::readSettings( roboclaw_msgs::SettingsReport& a_report )
{
    auto& s = *a_report.mutable_settings();
    auto& m1 = *s.mutable_m1();
    auto& m2 = *s.mutable_m2();

    if( const auto version = m_motor.readVersion() )
    {
        a_report.set_version( *version );
    }
    else
        a_report.add_failed( "ReadVersion" );

    if( const auto pid = m_motor.readM1VelocityPID() )
    {
        m1.set_velocity_p( pid->kp );
        m1.set_velocity_i( pid->ki );
        m1.set_velocity_d( pid->kd );
        m1.set_velocity_qpps( pid->qpps );
    }
    else
        a_report.add_failed( "ReadM1VelocityPID" );

    if( const auto pid = m_motor.readM2VelocityPID() )
    {
        m2.set_velocity_p( pid->kp );
        m2.set_velocity_i( pid->ki );
        m2.set_velocity_d( pid->kd );
        m2.set_velocity_qpps( pid->qpps );
    }
    else
        a_report.add_failed( "ReadM2VelocityPID" );

    if( const auto pid = m_motor.readM1PositionPID() )
    {
        m1.set_position_p( pid->kp );
        m1.set_position_i( pid->ki );
        m1.set_position_d( pid->kd );
        m1.set_position_max_i( pid->kiMax );
        m1.set_position_deadzone( pid->deadZone );
        m1.set_position_min( static_cast< int32_t >( pid->min ) );
        m1.set_position_max( static_cast< int32_t >( pid->max ) );
    }
    else
        a_report.add_failed( "ReadM1PositionPID" );

    if( const auto pid = m_motor.readM2PositionPID() )
    {
        m2.set_position_p( pid->kp );
        m2.set_position_i( pid->ki );
        m2.set_position_d( pid->kd );
        m2.set_position_max_i( pid->kiMax );
        m2.set_position_deadzone( pid->deadZone );
        m2.set_position_min( static_cast< int32_t >( pid->min ) );
        m2.set_position_max( static_cast< int32_t >( pid->max ) );
    }
    else
        a_report.add_failed( "ReadM2PositionPID" );

    if( const auto current = m_motor.readM1MaxCurrent() )
    {
        m1.set_max_current( *current );
    }
    else
        a_report.add_failed( "ReadM1MaxCurrent" );

    if( const auto current = m_motor.readM2MaxCurrent() )
    {
        m2.set_max_current( *current );
    }
    else
        a_report.add_failed( "ReadM2MaxCurrent" );

    if( const auto accels = m_motor.getDefaultAccels() )
    {
        m1.set_default_duty_acceleration( accels->m1.accel );
        m1.set_default_duty_deceleration( accels->m1.deccel );
        m2.set_default_duty_acceleration( accels->m2.accel );
        m2.set_default_duty_deceleration( accels->m2.deccel );
    }
    else
        a_report.add_failed( "GetDefaultAccels" );

    if( const auto values = m_motor.getDefaultSpeeds() )
    {
        m1.set_default_speed( values->m1 );
        m2.set_default_speed( values->m2 );
    }
    else
        a_report.add_failed( "GetDefaultSpeeds" );

    if( const auto values = m_motor.getSpeedErrorLimits() )
    {
        m1.set_speed_error_limit( values->m1 );
        m2.set_speed_error_limit( values->m2 );
    }
    else
        a_report.add_failed( "GetSpeedErrorLimits" );

    if( const auto values = m_motor.getPosErrorLimits() )
    {
        m1.set_position_error_limit( values->m1 );
        m2.set_position_error_limit( values->m2 );
    }
    else
        a_report.add_failed( "GetPosErrorLimits" );

    if( const auto values = m_motor.readEncoderModes() )
    {
        m1.set_encoder_mode( values->m1 );
        m2.set_encoder_mode( values->m2 );
    }
    else
        a_report.add_failed( "ReadEncoderModes" );

    if( const auto autos = m_motor.getAutos() )
    {
        m1.set_auto_duty( autos->m1.duty );
        m1.set_auto_timeout( autos->m1.timeout );
        m2.set_auto_duty( autos->m2.duty );
        m2.set_auto_timeout( autos->m2.timeout );
    }
    else
        a_report.add_failed( "GetAutos" );

    if( const auto values = m_motor.getCurrentBlanking() )
    {
        m1.set_current_blanking( values->m1 );
        m2.set_current_blanking( values->m2 );
    }
    else
        a_report.add_failed( "GetCurrentBlanking" );

    if( const auto values = m_motor.getCtrlModes() )
    {
        m1.set_ctrl_mode( values->m1 );
        m2.set_ctrl_mode( values->m2 );
    }
    else
        a_report.add_failed( "GetCtrlModes" );

    if( const auto values = m_motor.getCtrls() )
    {
        m1.set_ctrl_value( values->m1 );
        m2.set_ctrl_value( values->m2 );
    }
    else
        a_report.add_failed( "GetCtrls" );

    if( const auto voltages = m_motor.readMinMaxMainVoltages() )
    {
        s.set_main_voltage_min( voltages->min );
        s.set_main_voltage_max( voltages->max );
    }
    else
        a_report.add_failed( "ReadMinMaxMainVoltages" );

    if( const auto voltages = m_motor.readMinMaxLogicVoltages() )
    {
        s.set_logic_voltage_min( voltages->min );
        s.set_logic_voltage_max( voltages->max );
    }
    else
        a_report.add_failed( "ReadMinMaxLogicVoltages" );

    if( const auto offsets = m_motor.getVoltageOffsets() )
    {
        s.set_main_voltage_offset( offsets->main );
        s.set_logic_voltage_offset( offsets->logic );
    }
    else
        a_report.add_failed( "GetVoltageOffsets" );

    if( const auto pins = m_motor.getPinFunctions() )
    {
        s.set_s3_mode( pins->s3 );
        s.set_s4_mode( pins->s4 );
        s.set_s5_mode( pins->s5 );
    }
    else
        a_report.add_failed( "GetPinFunctions" );

    if( const auto deadband = m_motor.getDeadBand() )
    {
        s.set_deadband_min( deadband->min );
        s.set_deadband_max( deadband->max );
    }
    else
        a_report.add_failed( "GetDeadBand" );

    if( const auto config = m_motor.getConfig() )
    {
        s.set_config( *config );
    }
    else
        a_report.add_failed( "GetConfig" );

    if( const auto mode = m_motor.getPWMMode() )
    {
        s.set_pwm_mode( *mode );
    }
    else
        a_report.add_failed( "GetPWMMode" );

    if( const auto timeout = m_motor.getTimeout() )
    {
        s.set_serial_timeout( *timeout );
    }
    else
        a_report.add_failed( "GetTimeout" );
}

void RoboClawInterface::applySettings( const roboclaw_msgs::Settings& a_settings, roboclaw_msgs::SettingsReport& a_report )
{
    // Only writes what differs from the controller, and nothing that could not be read first.
    roboclaw_msgs::SettingsReport currentReport;
    readSettings( currentReport );
    if( currentReport.failed_size() > 0 )
    {
        for( const auto& failed : currentReport.failed() )
        {
            a_report.add_failed( failed );
        }
        a_report.set_result( "Nothing written, could not read the current settings." );
        return;
    }
    const auto& now = currentReport.settings();
    const auto& next = a_settings;
    int written = 0;

    auto velocity = [ & ]( const roboclaw_msgs::MotorSettings& a_now, const roboclaw_msgs::MotorSettings& a_next, auto a_set, const char* a_name )
    {
        if( !sameGain( a_now.velocity_p(), a_next.velocity_p() ) || !sameGain( a_now.velocity_i(), a_next.velocity_i() ) ||
            !sameGain( a_now.velocity_d(), a_next.velocity_d() ) || a_now.velocity_qpps() != a_next.velocity_qpps() )
        {
            check( ( m_motor.*a_set )( a_next.velocity_p(), a_next.velocity_i(), a_next.velocity_d(), a_next.velocity_qpps() ), a_name, a_report );
            written++;
        }
    };
    velocity( now.m1(), next.m1(), &RoboClaw::setM1VelocityPID, "SetM1VelocityPID" );
    velocity( now.m2(), next.m2(), &RoboClaw::setM2VelocityPID, "SetM2VelocityPID" );

    auto position = [ & ]( const roboclaw_msgs::MotorSettings& a_now, const roboclaw_msgs::MotorSettings& a_next, auto a_set, const char* a_name )
    {
        if( !sameGain( a_now.position_p(), a_next.position_p() ) || !sameGain( a_now.position_i(), a_next.position_i() ) ||
            !sameGain( a_now.position_d(), a_next.position_d() ) || a_now.position_max_i() != a_next.position_max_i() ||
            a_now.position_deadzone() != a_next.position_deadzone() || a_now.position_min() != a_next.position_min() ||
            a_now.position_max() != a_next.position_max() )
        {
            check( ( m_motor.*a_set )( a_next.position_p(), a_next.position_i(), a_next.position_d(), a_next.position_max_i(), a_next.position_deadzone(),
                                       static_cast< uint32_t >( a_next.position_min() ), static_cast< uint32_t >( a_next.position_max() ) ),
                   a_name, a_report );
            written++;
        }
    };
    position( now.m1(), next.m1(), &RoboClaw::setM1PositionPID, "SetM1PositionPID" );
    position( now.m2(), next.m2(), &RoboClaw::setM2PositionPID, "SetM2PositionPID" );

    auto apply = [ & ]( bool a_changed, auto a_write, const char* a_name )
    {
        if( a_changed )
        {
            check( a_write(), a_name, a_report );
            written++;
        }
    };

    apply( now.m1().max_current() != next.m1().max_current(), [ & ]() { return m_motor.setM1MaxCurrent( next.m1().max_current() ); }, "SetM1MaxCurrent" );
    apply( now.m2().max_current() != next.m2().max_current(), [ & ]() { return m_motor.setM2MaxCurrent( next.m2().max_current() ); }, "SetM2MaxCurrent" );
    apply( now.m1().default_duty_acceleration() != next.m1().default_duty_acceleration() || now.m1().default_duty_deceleration() != next.m1().default_duty_deceleration(),
           [ & ]() { return m_motor.setM1DefaultAccel( next.m1().default_duty_acceleration(), next.m1().default_duty_deceleration() ); }, "SetM1DefaultAccel" );
    apply( now.m2().default_duty_acceleration() != next.m2().default_duty_acceleration() || now.m2().default_duty_deceleration() != next.m2().default_duty_deceleration(),
           [ & ]() { return m_motor.setM2DefaultAccel( next.m2().default_duty_acceleration(), next.m2().default_duty_deceleration() ); }, "SetM2DefaultAccel" );
    apply( now.m1().default_speed() != next.m1().default_speed(), [ & ]() { return m_motor.setM1DefaultSpeed( next.m1().default_speed() ); }, "SetM1DefaultSpeed" );
    apply( now.m2().default_speed() != next.m2().default_speed(), [ & ]() { return m_motor.setM2DefaultSpeed( next.m2().default_speed() ); }, "SetM2DefaultSpeed" );
    apply( now.m1().speed_error_limit() != next.m1().speed_error_limit() || now.m2().speed_error_limit() != next.m2().speed_error_limit(),
           [ & ]() { return m_motor.setSpeedErrorLimits( next.m1().speed_error_limit(), next.m2().speed_error_limit() ); }, "SetSpeedErrorLimits" );
    apply( now.m1().position_error_limit() != next.m1().position_error_limit() || now.m2().position_error_limit() != next.m2().position_error_limit(),
           [ & ]() { return m_motor.setPosErrorLimits( next.m1().position_error_limit(), next.m2().position_error_limit() ); }, "SetPosErrorLimits" );
    apply( now.m1().encoder_mode() != next.m1().encoder_mode(), [ & ]() { return m_motor.setM1EncoderMode( next.m1().encoder_mode() ); }, "SetM1EncoderMode" );
    apply( now.m2().encoder_mode() != next.m2().encoder_mode(), [ & ]() { return m_motor.setM2EncoderMode( next.m2().encoder_mode() ); }, "SetM2EncoderMode" );
    apply( now.m1().auto_duty() != next.m1().auto_duty() || now.m1().auto_timeout() != next.m1().auto_timeout(),
           [ & ]() { return m_motor.setAuto1( next.m1().auto_duty(), next.m1().auto_timeout() ); }, "SetAuto1" );
    apply( now.m2().auto_duty() != next.m2().auto_duty() || now.m2().auto_timeout() != next.m2().auto_timeout(),
           [ & ]() { return m_motor.setAuto2( next.m2().auto_duty(), next.m2().auto_timeout() ); }, "SetAuto2" );
    apply( now.m1().current_blanking() != next.m1().current_blanking() || now.m2().current_blanking() != next.m2().current_blanking(),
           [ & ]() { return m_motor.setCurrentBlanking( next.m1().current_blanking(), next.m2().current_blanking() ); }, "SetCurrentBlanking" );
    apply( now.m1().ctrl_mode() != next.m1().ctrl_mode() || now.m2().ctrl_mode() != next.m2().ctrl_mode(),
           [ & ]() { return m_motor.setCtrlModes( next.m1().ctrl_mode(), next.m2().ctrl_mode() ); }, "SetCtrlModes" );
    apply( now.m1().ctrl_value() != next.m1().ctrl_value(), [ & ]() { return m_motor.setCtrl1( next.m1().ctrl_value() ); }, "SetCtrl1" );
    apply( now.m2().ctrl_value() != next.m2().ctrl_value(), [ & ]() { return m_motor.setCtrl2( next.m2().ctrl_value() ); }, "SetCtrl2" );
    apply( now.main_voltage_min() != next.main_voltage_min() || now.main_voltage_max() != next.main_voltage_max(),
           [ & ]() { return m_motor.setMainVoltages( next.main_voltage_min(), next.main_voltage_max() ); }, "SetMainVoltages" );
    apply( now.logic_voltage_min() != next.logic_voltage_min() || now.logic_voltage_max() != next.logic_voltage_max(),
           [ & ]() { return m_motor.setLogicVoltages( next.logic_voltage_min(), next.logic_voltage_max() ); }, "SetLogicVoltages" );
    apply( now.main_voltage_offset() != next.main_voltage_offset() || now.logic_voltage_offset() != next.logic_voltage_offset(),
           [ & ]() { return m_motor.setVoltageOffsets( next.main_voltage_offset(), next.logic_voltage_offset() ); }, "SetVoltageOffsets" );
    apply( now.s3_mode() != next.s3_mode() || now.s4_mode() != next.s4_mode() || now.s5_mode() != next.s5_mode(),
           [ & ]() { return m_motor.setPinFunctions( next.s3_mode(), next.s4_mode(), next.s5_mode() ); }, "SetPinFunctions" );
    apply( now.deadband_min() != next.deadband_min() || now.deadband_max() != next.deadband_max(),
           [ & ]() { return m_motor.setDeadBand( next.deadband_min(), next.deadband_max() ); }, "SetDeadBand" );
    apply( now.pwm_mode() != next.pwm_mode(), [ & ]() { return m_motor.setPWMMode( next.pwm_mode() ); }, "SetPWMMode" );
    apply( now.serial_timeout() != next.serial_timeout(), [ & ]() { return m_motor.setTimeout( next.serial_timeout() ); }, "SetTimeout" );
    // Last, this one can change how the controller is reached.
    apply( now.config() != next.config(), [ & ]() { return m_motor.setConfig( next.config() ); }, "SetConfig" );

    a_report.set_result( "Wrote " + std::to_string( written ) + " changed group(s) to RAM." );
}

void RoboClawInterface::sendReport( const std::string& a_result, const roboclaw_msgs::SettingsReport* a_previous )
{
    roboclaw_msgs::SettingsReport report;
    readSettings( report );
    report.set_result( a_result );
    if( a_previous != nullptr )
    {
        for( const auto& failed : a_previous->failed() )
        {
            report.add_failed( failed );
        }
    }
    std::cout << "Settings: " << a_result << " failed: " << report.failed_size() << std::endl;
    sepia::comm2::Dispatcher< roboclaw_msgs::SettingsReport >::send( &report );
}

void RoboClawInterface::receive( const roboclaw_msgs::RequestSettings& a_msg )
{
    sendReport( "Read from controller." );
}

void RoboClawInterface::receive( const roboclaw_msgs::WriteSettings& a_msg )
{
    roboclaw_msgs::SettingsReport result;
    applySettings( a_msg.settings(), result );
    sendReport( result.result(), &result );
}

void RoboClawInterface::receive( const roboclaw_msgs::SaveSettings& a_msg )
{
    sendReport( m_motor.writeNVM() ? "Saved to flash." : "Save to flash FAILED." );
}

void RoboClawInterface::receive( const roboclaw_msgs::ReloadSettings& a_msg )
{
    sendReport( m_motor.readNVM() ? "Reloaded from flash." : "Reload from flash FAILED." );
}

void RoboClawInterface::receive( const roboclaw_msgs::RestoreDefaultSettings& a_msg )
{
    sendReport( m_motor.restoreDefaults() ? "Restored factory defaults." : "Restore defaults FAILED." );
}
