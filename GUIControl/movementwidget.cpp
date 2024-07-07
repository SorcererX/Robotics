#include "movementwidget.h"
#include "ui_movementwidget.h"

#include <QSpinBox>
#include <sepia/comm2/dispatcher.h>

MovementWidget::MovementWidget( QWidget* a_parent )
    : QWidget( a_parent )
    , m_ui( new Ui::MovementWidget )
{
    m_ui->setupUi( this );

    connect( m_ui->iLeftSpeed,
             static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ),
             this,
             [ this ]( double a_value )
    {
        m_msg.set_left_motor( a_value );
    } );

    connect( m_ui->iRightSpeed,
             static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ),
             this,
             [ this ]( double a_value )
    {
        m_msg.set_right_motor( a_value );
    } );

    connect( m_ui->btnStart, &QPushButton::clicked, this, [ this ]()
    {
        sepia::comm2::Dispatcher< roboclaw_msgs::Move >::send( &m_msg );
    } );

    connect( m_ui->btnStop, &QPushButton::clicked, this, [ this ]()
    {
        roboclaw_msgs::Move msg;
        msg.set_left_motor( 0 );
        msg.set_right_motor( 0 );
        sepia::comm2::Dispatcher< roboclaw_msgs::Move >::send( &msg );
    } );
}

MovementWidget::~MovementWidget()
{
    delete m_ui;
    m_ui = nullptr;
}
