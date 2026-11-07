#include "movementwidget.h"
#include "roboclaw_msgs.pb.h"
#include "ui_movementwidget.h"

#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <cmath>
#include <utility>
#include <sepia/comm2/dispatcher.h>

MovementWidget::MovementWidget( QWidget* a_parent, Commands a_commands )
    : QWidget( a_parent )
    , m_ui( new Ui::MovementWidget )
    , m_statusRequestTimer( new QTimer( this ) )
{
    m_ui->setupUi( this );

    // The other page is taken out, so it does not make the widget as large as both.
    const bool cm = a_commands == Commands::Cm;
    QWidget* unused = cm ? m_ui->pageRaw : m_ui->pageCm;
    m_ui->stackCommands->removeWidget( unused );
    unused->hide();
    // The cm encoders are on the cm page, the raw ones below Status.
    m_ui->gbEncoders->setVisible( !cm );


    addConnect( m_ui->iLeftSpeed, std::bind_front( &roboclaw_msgs::MovePosition::set_left_speed, &m_msg ) );
    addConnect( m_ui->iLeftAcceleration, std::bind_front( &roboclaw_msgs::MovePosition::set_left_acceleration, &m_msg ) );
    addConnect( m_ui->iLeftDeceleration, std::bind_front( &roboclaw_msgs::MovePosition::set_left_deceleration, &m_msg ) );
    addConnect( m_ui->iLeftDistance, std::bind_front( &roboclaw_msgs::MovePosition::set_left_distance, &m_msg ) );
    addConnect( m_ui->iRightSpeed, std::bind_front( &roboclaw_msgs::MovePosition::set_right_speed, &m_msg ) );
    addConnect( m_ui->iRightAcceleration, std::bind_front( &roboclaw_msgs::MovePosition::set_right_acceleration, &m_msg ) );
    addConnect( m_ui->iRightDeceleration, std::bind_front( &roboclaw_msgs::MovePosition::set_right_deceleration, &m_msg ) );
    addConnect( m_ui->iRightDistance, std::bind_front( &roboclaw_msgs::MovePosition::set_right_distance, &m_msg ) );

    connect( m_ui->cbClear, &QCheckBox::clicked, this, [ this ]()
    { m_msg.set_clear_buffer( m_ui->cbClear->isChecked() ); } );

    connect( m_ui->btnStop, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::Move msg;
        msg.set_left_motor( 0 );
        msg.set_right_motor( 0 );
        sepia::comm2::Dispatcher< roboclaw_msgs::Move >::send( &msg ); } );

    connect( m_ui->btnResetEncoders, &QPushButton::clicked, this, []()
    {
        roboclaw_msgs::ResetEncoders msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::ResetEncoders >::send( &msg );
    } );

    connect( m_ui->btnSetEncoders, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::SetEncoders msg;
        msg.set_left_encoder( m_ui->iLeftEncoder->value() );
        msg.set_right_encoder( m_ui->iRightEncoder->value() );
        sepia::comm2::Dispatcher< roboclaw_msgs::SetEncoders >::send( &msg );
    } );

    // Each duty can be entered in % or raw, the controller uses 32767 for 100 %.
    linkPercentToRaw( m_ui->iLeftDuty, m_ui->iLeftDutyRaw );
    linkPercentToRaw( m_ui->iRightDuty, m_ui->iRightDutyRaw );
    linkPercentToRaw( m_ui->iDutyAcceleration, m_ui->iDutyAccelerationRaw );

    connect( m_ui->btnMoveDuty, &QPushButton::clicked, this, [ this ]()
    {
        // The raw values are sent, so they are exact even when the % is rounded.
        const auto acceleration = static_cast< uint32_t >( m_ui->iDutyAccelerationRaw->value() );
        roboclaw_msgs::MoveDuty msg;
        msg.set_left_duty( m_ui->iLeftDutyRaw->value() );
        msg.set_right_duty( m_ui->iRightDutyRaw->value() );
        msg.set_left_acceleration( acceleration );
        msg.set_right_acceleration( acceleration );
        std::cout << "Sending: " << msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MoveDuty >::send( &msg );
    } );

    connect( m_ui->btnStart, &QPushButton::clicked, this, [ this ]()
    {
        std::cout << "Sending: " << m_msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MovePosition >::send( &m_msg );
    } );

    connect( m_ui->btnMove, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::MoveSpeed msg;
        msg.set_left_speed( m_ui->iLeftMoveSpeed->value() );
        msg.set_right_speed( m_ui->iRightMoveSpeed->value() );
        msg.set_acceleration( m_ui->iMoveAcceleration->value() );
        std::cout << "Sending: " << msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MoveSpeed >::send( &msg );
    } );

    // The cm commands are converted to counts by MovementControl.
    connect( m_ui->btnMoveSpeedCm, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::MoveSpeedCm msg;
        msg.set_left_speed_cm_s( m_ui->cLeftMoveSpeed->value() );
        msg.set_right_speed_cm_s( m_ui->cRightMoveSpeed->value() );
        msg.set_acceleration_cm_s2( m_ui->cMoveAcceleration->value() );
        std::cout << "Sending: " << msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MoveSpeedCm >::send( &msg );
    } );

    connect( m_ui->btnMovePositionCm, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::MovePositionCm msg;
        msg.set_left_speed_cm_s( m_ui->cLeftSpeed->value() );
        msg.set_left_acceleration_cm_s2( m_ui->cLeftAcceleration->value() );
        msg.set_left_deceleration_cm_s2( m_ui->cLeftDeceleration->value() );
        msg.set_left_position_cm( m_ui->cLeftPosition->value() );
        msg.set_right_speed_cm_s( m_ui->cRightSpeed->value() );
        msg.set_right_acceleration_cm_s2( m_ui->cRightAcceleration->value() );
        msg.set_right_deceleration_cm_s2( m_ui->cRightDeceleration->value() );
        msg.set_right_position_cm( m_ui->cRightPosition->value() );
        msg.set_clear_buffer( m_ui->cbClearCm->isChecked() );
        std::cout << "Sending: " << msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MovePositionCm >::send( &msg );
    } );

    connect( m_ui->btnMoveDistanceCm, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::MoveDistanceCm msg;
        msg.set_left_speed_cm_s( m_ui->cLeftSpeed->value() );
        msg.set_left_acceleration_cm_s2( m_ui->cLeftAcceleration->value() );
        msg.set_left_deceleration_cm_s2( m_ui->cLeftDeceleration->value() );
        msg.set_left_distance_cm( m_ui->cLeftDistance->value() );
        msg.set_right_speed_cm_s( m_ui->cRightSpeed->value() );
        msg.set_right_acceleration_cm_s2( m_ui->cRightAcceleration->value() );
        msg.set_right_deceleration_cm_s2( m_ui->cRightDeceleration->value() );
        msg.set_right_distance_cm( m_ui->cRightDistance->value() );
        msg.set_clear_buffer( m_ui->cbClearCm->isChecked() );
        std::cout << "Sending: " << msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MoveDistanceCm >::send( &msg );
    } );

    connect( m_ui->btnSetEncodersCm, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::SetEncodersCm msg;
        msg.set_left_position_cm( m_ui->cLeftEncoder->value() );
        msg.set_right_position_cm( m_ui->cRightEncoder->value() );
        sepia::comm2::Dispatcher< roboclaw_msgs::SetEncodersCm >::send( &msg );
    } );

    connect( m_statusRequestTimer, &QTimer::timeout, this, [ this ]()
    {
        roboclaw_msgs::RequestStatus msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestStatus >::send( &msg );
    } );
    // Started when the widget is shown, so only the visible tab asks for status.
    m_statusRequestTimer->setInterval( 100 );
}

void MovementWidget::showEvent( QShowEvent* a_event )
{
    QWidget::showEvent( a_event );
    m_statusRequestTimer->start();
}

void MovementWidget::hideEvent( QHideEvent* a_event )
{
    QWidget::hideEvent( a_event );
    m_statusRequestTimer->stop();
}

void MovementWidget::receive( const roboclaw_msgs::MoveStatus& a_message )
{
    // Fields that could not be read are shown as "-".
    const auto show = []( QLineEdit* a_field, bool a_has, const QString& a_text ) { a_field->setText( a_has ? a_text : "-" ); };
    const auto scaled = []( double a_value, double a_scale, int a_decimals ) { return QString::number( a_value / a_scale, 'f', a_decimals ); };
    // The cm values come converted from MovementControl. One that does not convert sends no counts per cm.
    const bool hasCm = a_message.left_counts_per_cm() > 0 && a_message.right_counts_per_cm() > 0;
    const auto counts = [ & ]( QLineEdit* a_counts, QLineEdit* a_cm, bool a_has, int32_t a_value, double a_cmValue, const char* a_unit )
    {
        show( a_counts, a_has, QString::number( a_value ) );
        show( a_cm, a_has && hasCm, QString::number( a_cmValue, 'f', 2 ) + " " + a_unit );
    };

    counts( m_ui->sLeftSpeed, m_ui->sLeftSpeedCm, true, a_message.left_speed(), a_message.left_speed_cm_s(), "cm/s" );
    counts( m_ui->sRightSpeed, m_ui->sRightSpeedCm, true, a_message.right_speed(), a_message.right_speed_cm_s(), "cm/s" );
    counts( m_ui->sLeftEncoder, m_ui->sLeftEncoderCm, true, a_message.left_encoder(), a_message.left_position_cm(), "cm" );
    counts( m_ui->sRightEncoder, m_ui->sRightEncoderCm, true, a_message.right_encoder(), a_message.right_position_cm(), "cm" );
    counts( m_ui->sLeftSpeedError, m_ui->sLeftSpeedErrorCm, a_message.has_left_speed_error(), a_message.left_speed_error(), a_message.left_speed_error_cm_s(), "cm/s" );
    counts( m_ui->sRightSpeedError, m_ui->sRightSpeedErrorCm, a_message.has_right_speed_error(), a_message.right_speed_error(), a_message.right_speed_error_cm_s(), "cm/s" );
    counts( m_ui->sLeftPositionError, m_ui->sLeftPositionErrorCm, a_message.has_left_position_error(), a_message.left_position_error(), a_message.left_position_error_cm(), "cm" );
    counts( m_ui->sRightPositionError, m_ui->sRightPositionErrorCm, a_message.has_right_position_error(), a_message.right_position_error(), a_message.right_position_error_cm(), "cm" );
    const auto buffer = []( uint32_t a_value )
    {
        if( a_value == 128 )
        {
            return QString( "Idle" );
        }
        return a_value == 0 ? QString( "Running" ) : QString( "%1 waiting" ).arg( a_value );
    };

    show( m_ui->sLeftCurrent, a_message.has_left_current(), scaled( a_message.left_current(), 100.0, 2 ) );
    show( m_ui->sRightCurrent, a_message.has_right_current(), scaled( a_message.right_current(), 100.0, 2 ) );
    show( m_ui->sLeftPwm, a_message.has_left_pwm(), scaled( a_message.left_pwm(), 327.67, 1 ) );
    show( m_ui->sRightPwm, a_message.has_right_pwm(), scaled( a_message.right_pwm(), 327.67, 1 ) );
    show( m_ui->sLeftBuffer, a_message.has_left_buffer(), buffer( a_message.left_buffer() ) );
    show( m_ui->sRightBuffer, a_message.has_right_buffer(), buffer( a_message.right_buffer() ) );
    show( m_ui->sBattery, a_message.has_main_battery_voltage(), scaled( a_message.main_battery_voltage(), 10.0, 1 ) );
    show( m_ui->sTemperature, a_message.has_temperature(), scaled( a_message.temperature(), 10.0, 1 ) );
    m_ui->sErrors->setText( a_message.has_error() ? errorText( a_message.error() ) : "-" );
}

void MovementWidget::linkPercentToRaw( QDoubleSpinBox* a_percent, QSpinBox* a_raw )
{
    constexpr double rawPerPercent = 327.67;
    connect( a_percent,
             static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ),
             this,
             [ a_raw ]( double a_value )
    {
        const QSignalBlocker blocker( a_raw );
        a_raw->setValue( static_cast< int >( std::lround( a_value * rawPerPercent ) ) );
    } );
    connect( a_raw,
             static_cast< void ( QSpinBox::* )( int ) >( &QSpinBox::valueChanged ),
             this,
             [ a_percent ]( int a_value )
    {
        const QSignalBlocker blocker( a_percent );
        a_percent->setValue( a_value / rawPerPercent );
    } );
    // Start from the % given in the .ui file.
    const QSignalBlocker blocker( a_raw );
    a_raw->setValue( static_cast< int >( std::lround( a_percent->value() * rawPerPercent ) ) );
}

QString MovementWidget::errorText( uint32_t a_error )
{
    // Bits from the RoboClaw user manual, command 90.
    static constexpr std::pair< uint32_t, const char* > names[] = {
        { 0x00000001, "E-Stop" },
        { 0x00000002, "Temperature error" },
        { 0x00000004, "Temperature 2 error" },
        { 0x00000008, "Main voltage high error" },
        { 0x00000010, "Logic voltage high error" },
        { 0x00000020, "Logic voltage low error" },
        { 0x00000040, "M1 driver fault" },
        { 0x00000080, "M2 driver fault" },
        { 0x00000100, "M1 speed error" },
        { 0x00000200, "M2 speed error" },
        { 0x00000400, "M1 position error" },
        { 0x00000800, "M2 position error" },
        { 0x00001000, "M1 current error" },
        { 0x00002000, "M2 current error" },
        { 0x00010000, "M1 over current warning" },
        { 0x00020000, "M2 over current warning" },
        { 0x00040000, "Main voltage high warning" },
        { 0x00080000, "Main voltage low warning" },
        { 0x00100000, "Temperature warning" },
        { 0x00200000, "Temperature 2 warning" },
        { 0x00400000, "S4 signal triggered" },
        { 0x00800000, "S5 signal triggered" },
        { 0x01000000, "Speed error limit warning" },
        { 0x02000000, "Position error limit warning" },
    };

    QStringList active;
    uint32_t known = 0;
    for( const auto& [ bit, name ] : names )
    {
        known |= bit;
        if( ( a_error & bit ) != 0 )
        {
            active << name;
        }
    }
    if( ( a_error & ~known ) != 0 )
    {
        active << QString( "Unknown 0x%1" ).arg( a_error & ~known, 8, 16, QChar( '0' ) );
    }
    return active.isEmpty() ? QString( "None" ) : active.join( ", " );
}

void MovementWidget::addConnect( QDoubleSpinBox* a_object, const std::function< void( int32_t ) >& a_lambda )
{
    connect( a_object,
             static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ),
             this,
             [ this, a_lambda ]( double a_value )
    {
        a_lambda( a_value );
    } );
}

MovementWidget::~MovementWidget()
{
    delete m_ui;
    m_ui = nullptr;
}
