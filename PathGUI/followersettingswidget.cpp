#include "followersettingswidget.h"
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <sepia/comm2/dispatcher.h>

namespace
{
    // The same marking as the settings in GUIControl.
    const char* changedStyle = "background-color: #ffd27f; color: black;";
}

FollowerSettingsWidget::FollowerSettingsWidget( QWidget* a_parent )
    : QWidget( a_parent )
    , m_useImu( new QCheckBox( "Check turns and arcs with the IMU" ) )
    , m_apply( new QPushButton( "Apply" ) )
    , m_status( new QLabel )
    , m_answerTimer( new QTimer( this ) )
{
    auto* layout = new QVBoxLayout( this );
    auto* note = new QLabel( "Changes last until PathFollower restarts. Copy as YAML gives the lines for sepia.yaml to keep them." );
    note->setWordWrap( true );
    layout->addWidget( note );

    auto* form = new QFormLayout;
    layout->addLayout( form );
    setLayout( layout );

    using Settings = path_msgs::FollowerSettings;
    addField( form, "Track width", "Middle to middle of the belts. Turns and arcs need it, 0 allows only lines.", "track_width_cm", 0, 200, 1, " cm",
              []( const Settings& s ) { return s.track_width_cm(); }, []( Settings& s, double v ) { s.set_track_width_cm( v ); } );
    addField( form, "Position tolerance", "How close the belts must get to a segment's end before the next one counts as started.", "tolerance_cm", 0.1, 20, 1,
              " cm", []( const Settings& s ) { return s.tolerance_cm(); }, []( Settings& s, double v ) { s.set_tolerance_cm( v ); } );
    form->addRow( m_useImu );
    connect( m_useImu, &QCheckBox::toggled, this, &FollowerSettingsWidget::markChanged );
    addField( form, "Heading tolerance", "A heading this close to the planned one after a turn or arc is not corrected.", "heading_tolerance_deg", 0.1, 45, 1, " °",
              []( const Settings& s ) { return s.heading_tolerance_deg(); }, []( Settings& s, double v ) { s.set_heading_tolerance_deg( v ); } );
    addField( form, "Max corrections", "How many times a heading is corrected before the path continues anyway.", "max_heading_corrections", 0, 10, 0, "",
              []( const Settings& s ) { return s.max_heading_corrections(); },
              []( Settings& s, double v ) { s.set_max_heading_corrections( static_cast< uint32_t >( v ) ); } );
    addField( form, "Calm rate", "After a turn or arc, the robot counts as calm while it turns slower than this.", "calm_rate_deg_s", 0.1, 30, 1, " °/s",
              []( const Settings& s ) { return s.calm_rate_deg_s(); }, []( Settings& s, double v ) { s.set_calm_rate_deg_s( v ); } );
    addField( form, "Calm time", "How long the robot must be calm before the heading is checked.", "calm_time_s", 0, 5, 2, " s",
              []( const Settings& s ) { return s.calm_time_s(); }, []( Settings& s, double v ) { s.set_calm_time_s( v ); } );
    addField( form, "Settle timeout", "The heading is checked after this long even if the robot never gets calm.", "settle_timeout_s", 0, 10, 2, " s",
              []( const Settings& s ) { return s.settle_timeout_s(); }, []( Settings& s, double v ) { s.set_settle_timeout_s( v ); } );
    auto* buttons = new QHBoxLayout;
    auto* read = new QPushButton( "Read" );
    auto* copy = new QPushButton( "Copy as YAML" );
    buttons->addWidget( read );
    buttons->addWidget( m_apply );
    buttons->addWidget( copy );
    layout->addLayout( buttons );
    m_status->setWordWrap( true );
    layout->addWidget( m_status );
    layout->addStretch( 1 );

    m_apply->setEnabled( false );
    connect( read, &QPushButton::clicked, this, &FollowerSettingsWidget::request );
    connect( m_apply, &QPushButton::clicked, this, &FollowerSettingsWidget::apply );
    connect( copy, &QPushButton::clicked, this, &FollowerSettingsWidget::copyYaml );

    m_answerTimer->setSingleShot( true );
    m_answerTimer->setInterval( 1500 );
    connect( m_answerTimer, &QTimer::timeout, this, [ this ]()
    {
        if( !m_answered )
        {
            m_status->setText( "No answer from PathFollower. Is it running?" );
        }
    } );
}

void FollowerSettingsWidget::addField( QFormLayout* a_form, const QString& a_label, const QString& a_tooltip, const char* a_key, double a_min, double a_max, int a_decimals,
                                       const QString& a_suffix, std::function< double( const path_msgs::FollowerSettings& ) > a_get,
                                       std::function< void( path_msgs::FollowerSettings&, double ) > a_set )
{
    auto* box = new QDoubleSpinBox;
    box->setRange( a_min, a_max );
    box->setDecimals( a_decimals );
    box->setSuffix( a_suffix );
    box->setToolTip( a_tooltip );
    auto* label = new QLabel( a_label );
    label->setToolTip( a_tooltip );
    a_form->addRow( label, box );
    connect( box, qOverload< double >( &QDoubleSpinBox::valueChanged ), this, &FollowerSettingsWidget::markChanged );
    m_fields.push_back( { .box = box, .key = a_key, .get = std::move( a_get ), .set = std::move( a_set ) } );
}

void FollowerSettingsWidget::showEvent( QShowEvent* a_event )
{
    QWidget::showEvent( a_event );
    // Read the first time the tab is shown, Read gets them again.
    if( !m_reported )
    {
        request();
    }
}

path_msgs::FollowerSettings FollowerSettingsWidget::entered() const
{
    path_msgs::FollowerSettings settings;
    for( const Field& field : m_fields )
    {
        field.set( settings, field.box->value() );
    }
    settings.set_use_imu( m_useImu->isChecked() );
    return settings;
}

void FollowerSettingsWidget::markChanged()
{
    if( !m_reported )
    {
        return;
    }
    for( const Field& field : m_fields )
    {
        // Compared as shown, so a value that only differs past the shown decimals is not marked.
        const bool changed = QString::number( field.box->value(), 'f', field.box->decimals() ) !=
                             QString::number( field.get( *m_reported ), 'f', field.box->decimals() );
        field.box->setStyleSheet( changed ? changedStyle : "" );
    }
    m_useImu->setStyleSheet( m_useImu->isChecked() != m_reported->use_imu() ? changedStyle : "" );
}

void FollowerSettingsWidget::request()
{
    path_msgs::RequestFollowerSettings msg;
    sepia::comm2::Dispatcher< path_msgs::RequestFollowerSettings >::send( &msg );
    m_answered = false;
    m_answerTimer->start();
    m_status->setText( "Reading from PathFollower." );
}

void FollowerSettingsWidget::apply()
{
    path_msgs::WriteFollowerSettings msg;
    *msg.mutable_settings() = entered();
    sepia::comm2::Dispatcher< path_msgs::WriteFollowerSettings >::send( &msg );
    m_answered = false;
    m_answerTimer->start();
    m_status->setText( "Sent to PathFollower." );
}

void FollowerSettingsWidget::copyYaml()
{
    QString yaml = "path_follower:\n";
    for( const Field& field : m_fields )
    {
        // A point as the decimal separator, whatever the locale, since YAML needs it.
        yaml += QString( "  %1: %2\n" ).arg( field.key, QString::number( field.box->value(), 'f', field.box->decimals() ) );
    }
    yaml += QString( "  use_imu: %1\n" ).arg( m_useImu->isChecked() ? "true" : "false" );
    QApplication::clipboard()->setText( yaml );
    m_status->setText( "The settings in the fields are copied." );
}

void FollowerSettingsWidget::receive( const path_msgs::FollowerSettingsReport& a_msg )
{
    m_answered = true;
    m_reported = a_msg.settings();
    for( const Field& field : m_fields )
    {
        const QSignalBlocker blocker( field.box );
        field.box->setValue( field.get( a_msg.settings() ) );
    }
    {
        const QSignalBlocker blocker( m_useImu );
        m_useImu->setChecked( a_msg.settings().use_imu() );
    }
    markChanged();
    m_apply->setEnabled( true );
    m_status->setText( QString::fromStdString( a_msg.result() ) );
}
