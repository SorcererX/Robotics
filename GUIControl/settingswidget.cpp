#include "settingswidget.h"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>
#include <limits>
#include <sepia/comm2/dispatcher.h>

namespace
{
    using google::protobuf::FieldDescriptor;

    // Fields that are narrower on the controller than in the message.
    double maximumOf( const std::string& a_name )
    {
        static const std::map< std::string, double > maximums = {
            { "default_speed", 65535 }, { "auto_duty", 65535 }, { "current_blanking", 65535 }, { "ctrl_value", 65535 },
            { "main_voltage_min", 65535 }, { "main_voltage_max", 65535 }, { "logic_voltage_min", 65535 }, { "logic_voltage_max", 65535 },
            { "config", 65535 }, { "encoder_mode", 255 }, { "ctrl_mode", 255 }, { "s3_mode", 255 }, { "s4_mode", 255 }, { "s5_mode", 255 },
            { "deadband_min", 255 }, { "deadband_max", 255 }, { "pwm_mode", 255 }, { "serial_timeout", 255 },
            { "main_voltage_offset", 127 }, { "logic_voltage_offset", 127 } };
        auto it = maximums.find( a_name );
        return it == maximums.end() ? 0 : it->second;
    }

    QString labelOf( const std::string& a_name )
    {
        static const QStringList acronyms = { "p", "i", "d", "qpps", "pwm", "s3", "s4", "s5" };
        static const std::map< std::string, QString > units = {
            { "max_current", "10 mA" }, { "current_blanking", "/65536" }, { "serial_timeout", "0.1 s" },
            { "main_voltage_min", "0.1 V" }, { "main_voltage_max", "0.1 V" }, { "logic_voltage_min", "0.1 V" }, { "logic_voltage_max", "0.1 V" },
            { "main_voltage_offset", "0.1 V" }, { "logic_voltage_offset", "0.1 V" } };

        QStringList words = QString::fromStdString( a_name ).split( '_' );
        for( auto& word : words )
        {
            if( acronyms.contains( word ) )
            {
                word = word.toUpper();
            }
        }
        QString label = words.join( ' ' );
        label[ 0 ] = label[ 0 ].toUpper();
        if( auto it = units.find( a_name ); it != units.end() )
        {
            label += " (" + it->second + ")";
        }
        return label;
    }

    constexpr uint32_t CONFIG_LINK_MASK = 0x07E3; // Mode, baud rate and packet address, see command 98.
}

SettingsWidget::SettingsWidget( QWidget* a_parent )
    : QWidget( a_parent )
    , m_status( new QLabel( "Waiting for MovementControl...", this ) )
{
    auto* motorBox = new QGroupBox( "Motor" );
    auto* motorGrid = new QGridLayout( motorBox );
    for( const auto& [ column, text ] : { std::pair( 1, "M1" ), std::pair( 2, "M2" ) } )
    {
        auto* header = new QLabel( text );
        header->setAlignment( Qt::AlignCenter );
        motorGrid->addWidget( header, 0, column );
    }
    addRows( motorGrid, roboclaw_msgs::MotorSettings::descriptor(), 2 );

    auto* controllerBox = new QGroupBox( "Controller" );
    auto* controllerGrid = new QGridLayout( controllerBox );
    addRows( controllerGrid, roboclaw_msgs::Settings::descriptor(), 1 );

    auto* content = new QWidget;
    auto* columns = new QHBoxLayout( content );
    columns->addWidget( motorBox, 0, Qt::AlignTop );
    columns->addWidget( controllerBox, 0, Qt::AlignTop );
    columns->addStretch();

    auto* scroll = new QScrollArea;
    scroll->setWidget( content );
    scroll->setWidgetResizable( true );
    // Only scroll vertically, the window is made wide enough for both columns.
    scroll->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    scroll->setMinimumWidth( content->sizeHint().width() + scroll->verticalScrollBar()->sizeHint().width() + 2 * scroll->frameWidth() );

    auto* btnRead = new QPushButton( "Read" );
    auto* btnWrite = new QPushButton( "Write changes to RAM" );
    auto* btnSave = new QPushButton( "Save to flash" );
    auto* btnReload = new QPushButton( "Reload from flash" );
    auto* btnDefaults = new QPushButton( "Factory defaults" );

    connect( btnRead, &QPushButton::clicked, this, []()
    {
        roboclaw_msgs::RequestSettings msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestSettings >::send( &msg );
    } );

    connect( btnWrite, &QPushButton::clicked, this, [ this ]()
    {
        if( !m_received )
        {
            m_status->setText( "Read the settings first." );
            return;
        }
        roboclaw_msgs::WriteSettings msg;
        auto& settings = *msg.mutable_settings();
        for( const auto& [ field, box ] : m_general )
        {
            collect( settings, field, box );
        }
        for( const auto& [ field, box ] : m_m1 )
        {
            collect( *settings.mutable_m1(), field, box );
        }
        for( const auto& [ field, box ] : m_m2 )
        {
            collect( *settings.mutable_m2(), field, box );
        }
        if( confirmConfigChange( settings.config() ) )
        {
            sepia::comm2::Dispatcher< roboclaw_msgs::WriteSettings >::send( &msg );
        }
    } );

    connect( btnSave, &QPushButton::clicked, this, [ this ]()
    {
        if( confirmed( "Store the settings currently in the controller's RAM to flash?\nThey will be used from the next power on." ) )
        {
            roboclaw_msgs::SaveSettings msg;
            sepia::comm2::Dispatcher< roboclaw_msgs::SaveSettings >::send( &msg );
        }
    } );

    connect( btnReload, &QPushButton::clicked, this, [ this ]()
    {
        if( confirmed( "Discard changes in RAM and reload the stored settings?" ) )
        {
            roboclaw_msgs::ReloadSettings msg;
            sepia::comm2::Dispatcher< roboclaw_msgs::ReloadSettings >::send( &msg );
        }
    } );

    connect( btnDefaults, &QPushButton::clicked, this, [ this ]()
    {
        if( confirmed( "Erase all settings, including tuned PID values, and restore the factory defaults?\nThis can not be undone." ) )
        {
            roboclaw_msgs::RestoreDefaultSettings msg;
            sepia::comm2::Dispatcher< roboclaw_msgs::RestoreDefaultSettings >::send( &msg );
        }
    } );

    // RAM on the first row, flash on the second, with the destructive one apart from the rest.
    btnWrite->setToolTip( "Writes the highlighted fields. They are lost at power off unless saved to flash." );
    auto* buttons = new QGridLayout;
    buttons->addWidget( btnRead, 0, 0 );
    buttons->addWidget( btnWrite, 0, 1 );
    buttons->addWidget( btnSave, 1, 0 );
    buttons->addWidget( btnReload, 1, 1 );
    buttons->setColumnStretch( 2, 1 );
    buttons->addWidget( btnDefaults, 1, 3 );

    m_status->setWordWrap( true );

    auto* layout = new QVBoxLayout( this );
    layout->addWidget( scroll );
    layout->addLayout( buttons );
    layout->addWidget( m_status );

    // The subscription takes a moment to be established.
    auto* timer = new QTimer( this );
    connect( timer, &QTimer::timeout, this, [ this, timer ]()
    {
        if( m_received )
        {
            timer->stop();
            return;
        }
        roboclaw_msgs::RequestSettings msg;
        sepia::comm2::Dispatcher< roboclaw_msgs::RequestSettings >::send( &msg );
    } );
    timer->start( 1000 );
}

void SettingsWidget::addRows( QGridLayout* a_layout, const google::protobuf::Descriptor* a_type, int a_columns )
{
    for( int i = 0; i < a_type->field_count(); i++ )
    {
        Field field = a_type->field( i );
        if( field->type() == FieldDescriptor::TYPE_MESSAGE )
        {
            continue;
        }
        int row = a_layout->rowCount();
        a_layout->addWidget( new QLabel( labelOf( field->name() ) ), row, 0 );
        if( a_columns == 2 )
        {
            m_m1[ field ] = createSpinBox( field );
            m_m2[ field ] = createSpinBox( field );
            a_layout->addWidget( m_m1[ field ], row, 1 );
            a_layout->addWidget( m_m2[ field ], row, 2 );
        }
        else
        {
            m_general[ field ] = createSpinBox( field );
            a_layout->addWidget( m_general[ field ], row, 1 );
        }
    }
}

QDoubleSpinBox* SettingsWidget::createSpinBox( Field a_field )
{
    auto* box = new QDoubleSpinBox;
    double maximum = maximumOf( a_field->name() );
    switch( a_field->cpp_type() )
    {
    case FieldDescriptor::CPPTYPE_FLOAT:
        box->setDecimals( 6 );
        box->setRange( 0, 4194304 ); // Largest position gain, the fixed point value is 32 bits with 1024 per unit.
        break;
    case FieldDescriptor::CPPTYPE_INT32:
        box->setDecimals( 0 );
        box->setRange( maximum > 0 ? -maximum - 1 : std::numeric_limits< int32_t >::min(), maximum > 0 ? maximum : std::numeric_limits< int32_t >::max() );
        break;
    default:
        box->setDecimals( 0 );
        box->setRange( 0, maximum > 0 ? maximum : std::numeric_limits< uint32_t >::max() );
        break;
    }
    box->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
    box->setEnabled( false );
    box->setToolTip( QString::fromStdString( a_field->full_name() ) );
    connect( box, static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ), this, [ this ]( double )
    { updateChanged(); } );
    return box;
}

void SettingsWidget::show( const google::protobuf::Message& a_message, Field a_field, QDoubleSpinBox* a_box )
{
    const auto* reflection = a_message.GetReflection();
    switch( a_field->cpp_type() )
    {
    case FieldDescriptor::CPPTYPE_FLOAT:
        a_box->setValue( reflection->GetFloat( a_message, a_field ) );
        break;
    case FieldDescriptor::CPPTYPE_INT32:
        a_box->setValue( reflection->GetInt32( a_message, a_field ) );
        break;
    default:
        a_box->setValue( reflection->GetUInt32( a_message, a_field ) );
        break;
    }
    a_box->setEnabled( true );
}

void SettingsWidget::collect( google::protobuf::Message& a_message, Field a_field, const QDoubleSpinBox* a_box ) const
{
    const auto* reflection = a_message.GetReflection();
    switch( a_field->cpp_type() )
    {
    case FieldDescriptor::CPPTYPE_FLOAT:
        reflection->SetFloat( &a_message, a_field, a_box->value() );
        break;
    case FieldDescriptor::CPPTYPE_INT32:
        reflection->SetInt32( &a_message, a_field, a_box->value() );
        break;
    default:
        reflection->SetUInt32( &a_message, a_field, a_box->value() );
        break;
    }
}

void SettingsWidget::updateChanged()
{
    if( !m_received )
    {
        return;
    }
    auto mark = []( const google::protobuf::Message& a_read, const std::map< Field, QDoubleSpinBox* >& a_boxes )
    {
        for( const auto& [ field, box ] : a_boxes )
        {
            const auto* reflection = a_read.GetReflection();
            // Each branch is converted to double, a plain ?: would turn negative int32 values into large unsigned ones.
            double read = field->cpp_type() == FieldDescriptor::CPPTYPE_FLOAT ? static_cast< double >( reflection->GetFloat( a_read, field ) )
                        : field->cpp_type() == FieldDescriptor::CPPTYPE_INT32 ? static_cast< double >( reflection->GetInt32( a_read, field ) )
                                                                              : static_cast< double >( reflection->GetUInt32( a_read, field ) );
            bool changed = std::abs( box->value() - read ) > ( field->cpp_type() == FieldDescriptor::CPPTYPE_FLOAT ? 0.00002 : 0.5 );
            box->setStyleSheet( changed ? "background-color: #ffd27f; color: black;" : "" );
        }
    };
    mark( m_settings, m_general );
    mark( m_settings.m1(), m_m1 );
    mark( m_settings.m2(), m_m2 );
}

bool SettingsWidget::confirmed( const QString& a_question )
{
    return QMessageBox::question( this, "RoboClaw settings", a_question ) == QMessageBox::Yes;
}

bool SettingsWidget::confirmConfigChange( uint32_t a_config )
{
    if( ( ( a_config ^ m_settings.config() ) & CONFIG_LINK_MASK ) == 0 )
    {
        return true;
    }
    return confirmed( "This config changes the control mode, baud rate or packet address.\n"
                      "MovementControl expects packet serial at address 0x80 and may lose contact with the controller.\nContinue?" );
}

void SettingsWidget::receive( const roboclaw_msgs::SettingsReport& a_message )
{
    m_settings = a_message.settings();
    m_received = true;
    for( const auto& [ field, box ] : m_general )
    {
        show( m_settings, field, box );
    }
    for( const auto& [ field, box ] : m_m1 )
    {
        show( m_settings.m1(), field, box );
    }
    for( const auto& [ field, box ] : m_m2 )
    {
        show( m_settings.m2(), field, box );
    }
    updateChanged();

    QString text = QString::fromStdString( a_message.version() ).trimmed() + "\n" + QString::fromStdString( a_message.result() );
    if( a_message.failed_size() > 0 )
    {
        QStringList failed;
        for( const auto& name : a_message.failed() )
        {
            failed << QString::fromStdString( name );
        }
        text += "\nFAILED: " + failed.join( ", " );
    }
    m_status->setText( text );
}
