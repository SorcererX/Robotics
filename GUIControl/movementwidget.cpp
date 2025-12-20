#include "movementwidget.h"
#include "roboclaw_msgs.pb.h"
#include "ui_movementwidget.h"

#include <QSpinBox>
#include <QTimer>
#include <sepia/comm2/dispatcher.h>

MovementWidget::MovementWidget( QWidget* a_parent )
    : QWidget( a_parent )
    , m_ui( new Ui::MovementWidget )
    , m_statusRequestTimer( new QTimer( this ) )
{
    m_ui->setupUi( this );

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

    connect( m_ui->btnStart, &QPushButton::clicked, this, [ this ]()
    {
        std::cout << "Sending: " << m_msg.ShortDebugString() << std::endl;
        sepia::comm2::Dispatcher< roboclaw_msgs::MovePosition >::send( &m_msg );
    } );

    connect( m_statusRequestTimer, &QTimer::timeout, this, [ this ]()
    {
        roboclaw_msgs::RequestStatus msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestStatus >::send( &msg );
    } );
    m_statusRequestTimer->setInterval( 100 );
    m_statusRequestTimer->start();
}

void MovementWidget::receive( const roboclaw_msgs::MoveStatus& a_message )
{
    m_ui->sLeftEncoder->setText( QString::number( a_message.left_encoder() ) );
    m_ui->sLeftSpeed->setText( QString::number( a_message.left_speed() ) );
    m_ui->sRightEncoder->setText( QString::number( a_message.right_encoder() ) );
    m_ui->sRightSpeed->setText( QString::number( a_message.right_speed() ) );
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
