#include "pathwindow.h"
#include "followersettingswidget.h"
#include "pathcanvas.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeySequence>
#include <QLabel>
#include <QLocale>
#include <QPushButton>
#include <QRadioButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <numbers>
#include <sepia/comm2/dispatcher.h>
#include <set>

namespace
{
    enum Column
    {
        Type,
        Length,
        Angle,
        Speed,
        Columns
    };

    QDoubleSpinBox* spinBox( double a_min, double a_max, double a_value, const QString& a_suffix )
    {
        auto* box = new QDoubleSpinBox;
        box->setRange( a_min, a_max );
        box->setDecimals( 1 );
        box->setValue( a_value );
        box->setSuffix( a_suffix );
        return box;
    }

    QString number( double a_value )
    {
        return QLocale().toString( a_value, 'f', 1 );
    }

    // Accepts both the local decimal separator and a point.
    std::optional< double > parse( const QString& a_text )
    {
        bool ok = false;
        double value = QLocale().toDouble( a_text, &ok );
        if( !ok )
        {
            value = a_text.toDouble( &ok );
        }
        return ok ? std::optional( value ) : std::nullopt;
    }
}

PathWindow::PathWindow( QWidget* a_parent )
    : QWidget( a_parent )
    , m_canvas( new PathCanvas( this ) )
    , m_followerSettings( new FollowerSettingsWidget )
    , m_straight( new QRadioButton( "Straight (turn, then line)" ) )
    , m_table( new QTableWidget( 0, Columns ) )
    , m_speed( spinBox( 0.1, 200, 10, " cm/s" ) )
    , m_acceleration( spinBox( 0.1, 2000, 20, " cm/s²" ) )
    , m_status( new QLabel( "Click on the grid to draw a path from the triangle." ) )
    , m_answerTimer( new QTimer( this ) )
    , m_pathId( static_cast< uint32_t >( QDateTime::currentSecsSinceEpoch() ) )
{
    auto* panel = new QVBoxLayout;

    auto* draw = new QGroupBox( "Draw" );
    auto* drawLayout = new QVBoxLayout( draw );
    auto* arc = new QRadioButton( "Arc from the heading" );
    m_straight->setChecked( true );
    auto* snap = new QComboBox;
    for( const double cm : { 0.0, 1.0, 5.0, 10.0 } )
    {
        snap->addItem( cm > 0 ? QString( "Snap to %1 cm" ).arg( cm ) : QString( "No snap" ), cm );
    }
    snap->setCurrentIndex( 2 );
    drawLayout->addWidget( m_straight );
    drawLayout->addWidget( arc );
    drawLayout->addWidget( snap );

    // Exact segments are added in the same group, below the drawing tools.
    auto* exactLayout = new QGridLayout;
    drawLayout->addLayout( exactLayout );
    panel->addWidget( draw );
    auto* line = spinBox( -1000, 1000, 50, " cm" );
    auto* turn = spinBox( -3600, 3600, 90, " °" );
    auto* addLine = new QPushButton( "Add line" );
    auto* addTurn = new QPushButton( "Add turn" );
    exactLayout->addWidget( line, 0, 0 );
    exactLayout->addWidget( addLine, 0, 1 );
    exactLayout->addWidget( turn, 1, 0 );
    exactLayout->addWidget( addTurn, 1, 1 );

    auto* segments = new QGroupBox( "Segments" );
    auto* segmentsLayout = new QVBoxLayout( segments );
    m_table->setHorizontalHeaderLabels( { "", "cm", "°", "cm/s" } );
    m_table->horizontalHeader()->setSectionResizeMode( QHeaderView::Stretch );
    m_table->verticalHeader()->setSectionResizeMode( QHeaderView::ResizeToContents );
    m_table->setSelectionBehavior( QAbstractItemView::SelectRows );
    m_table->setMinimumHeight( 200 );
    m_table->setToolTip( "Double-click a value to change it. An empty speed uses the speed of the path." );
    auto* undo = new QPushButton( "Undo" );
    auto* remove = new QPushButton( "Remove" );
    auto* clear = new QPushButton( "Clear" );
    auto* editButtons = new QHBoxLayout;
    editButtons->addWidget( undo );
    editButtons->addWidget( remove );
    editButtons->addWidget( clear );
    segmentsLayout->addWidget( m_table );
    segmentsLayout->addLayout( editButtons );
    panel->addWidget( segments, 1 );

    auto* drive = new QGroupBox( "Drive" );
    auto* driveLayout = new QGridLayout( drive );
    auto* runButton = new QPushButton( "Run" );
    auto* stop = new QPushButton( "Stop" );
    auto* fit = new QPushButton( "Fit view" );
    auto* clearTrace = new QPushButton( "Clear trace" );
    m_status->setWordWrap( true );
    driveLayout->addWidget( new QLabel( "Speed" ), 0, 0 );
    driveLayout->addWidget( m_speed, 0, 1 );
    driveLayout->addWidget( new QLabel( "Acceleration" ), 1, 0 );
    driveLayout->addWidget( m_acceleration, 1, 1 );
    driveLayout->addWidget( runButton, 2, 0 );
    driveLayout->addWidget( stop, 2, 1 );
    driveLayout->addWidget( fit, 3, 0 );
    driveLayout->addWidget( clearTrace, 3, 1 );
    driveLayout->addWidget( m_status, 4, 0, 1, 2 );
    panel->addWidget( drive );

    // Each way of placing the robot can be hidden, with where it puts the robot: the planned end, and where the
    // encoders alone and the encoders with the IMU heading say the robot is.
    auto* position = new QGroupBox( "Position" );
    auto* positionLayout = new QGridLayout( position );
    const std::pair< PathCanvas::Layer, QString > layers[] = {
        { PathCanvas::Layer::Planned, "Planned" },
        { PathCanvas::Layer::Encoders, "Encoders" },
        { PathCanvas::Layer::Imu, "IMU" },
    };
    for( int column = 1; const char* unit : { "x cm", "y cm", "°" } )
    {
        positionLayout->addWidget( new QLabel( unit ), 0, column++, Qt::AlignRight );
    }
    for( int row = 1; const auto& [ layer, name ] : layers )
    {
        QPixmap swatch( 12, 12 );
        swatch.fill( PathCanvas::color( layer ) );
        auto* shown = new QCheckBox( name );
        shown->setIcon( QIcon( swatch ) );
        shown->setChecked( true );
        connect( shown, &QCheckBox::toggled, m_canvas, [ this, layer ]( bool a_shown ) { m_canvas->setShown( layer, a_shown ); } );
        positionLayout->addWidget( shown, row, 0 );
        for( int column = 0; column < 3; column++ )
        {
            auto* value = new QLabel;
            value->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
            m_poses[ layer ][ static_cast< size_t >( column ) ] = value;
            positionLayout->addWidget( value, row, column + 1 );
        }
        showPose( layer, std::nullopt );
        row++;
    }
    positionLayout->setColumnStretch( 0, 1 );
    panel->addWidget( position );

    // The path and the follower's settings are on separate tabs, so the window does not get taller.
    auto* panelWidget = new QWidget;
    panelWidget->setLayout( panel );
    auto* tabs = new QTabWidget;
    tabs->addTab( panelWidget, "Path" );
    tabs->addTab( m_followerSettings, "Follower" );
    tabs->setFixedWidth( 320 );
    auto* layout = new QHBoxLayout( this );
    layout->addWidget( m_canvas, 1 );
    layout->addWidget( tabs );

    connect( snap, qOverload< int >( &QComboBox::currentIndexChanged ), this, [ this, snap ]() { m_canvas->setSnap( snap->currentData().toDouble() ); } );
    connect( m_canvas, &PathCanvas::pointHovered, this, [ this ]( double a_x, double a_y ) { m_canvas->setPreview( segmentsTo( a_x, a_y ) ); } );
    connect( m_canvas, &PathCanvas::hoverEnded, this, [ this ]() { m_canvas->setPreview( {} ); } );
    connect( m_canvas, &PathCanvas::pointClicked, this, [ this ]( double a_x, double a_y )
    {
        add( segmentsTo( a_x, a_y ) );
        m_canvas->setPreview( {} );
    } );
    connect( addLine, &QPushButton::clicked, this, [ this, line ]()
    {
        path_msgs::PathSegment segment;
        segment.mutable_line()->set_distance_cm( line->value() );
        add( { segment } );
    } );
    connect( addTurn, &QPushButton::clicked, this, [ this, turn ]()
    {
        path_msgs::PathSegment segment;
        segment.mutable_turn()->set_angle_deg( turn->value() );
        add( { segment } );
    } );
    connect( undo, &QPushButton::clicked, this, &PathWindow::undo );
    connect( remove, &QPushButton::clicked, this, &PathWindow::removeSelected );
    connect( clear, &QPushButton::clicked, this, [ this ]()
    {
        m_segments.clear();
        m_added.clear();
        changed();
    } );
    connect( m_table, &QTableWidget::cellChanged, this, &PathWindow::cellEdited );
    connect( new QShortcut( QKeySequence::Undo, this ), &QShortcut::activated, this, &PathWindow::undo );
    connect( new QShortcut( QKeySequence::Delete, m_table ), &QShortcut::activated, this, &PathWindow::removeSelected );

    connect( runButton, &QPushButton::clicked, this, &PathWindow::run );
    connect( stop, &QPushButton::clicked, this, []()
    {
        path_msgs::StopPath msg;
        sepia::comm2::Dispatcher< path_msgs::StopPath >::send( &msg );
    } );
    connect( fit, &QPushButton::clicked, m_canvas, &PathCanvas::fit );
    connect( clearTrace, &QPushButton::clicked, m_canvas, &PathCanvas::clearTrace );

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

Pose PathWindow::endPose() const
{
    Pose pose{ 0, 0, 0 };
    for( const auto& segment : m_segments )
    {
        pose = poseAlong( pose, segment, 1 );
    }
    return pose;
}

std::vector< path_msgs::PathSegment > PathWindow::segmentsTo( double a_x, double a_y ) const
{
    if( m_straight->isChecked() )
    {
        return straightTo( endPose(), a_x, a_y );
    }
    if( const auto segment = arcTo( endPose(), a_x, a_y ) )
    {
        return { *segment };
    }
    return {};
}

void PathWindow::add( const std::vector< path_msgs::PathSegment >& a_segments )
{
    if( a_segments.empty() )
    {
        return;
    }
    m_segments.insert( m_segments.end(), a_segments.begin(), a_segments.end() );
    m_added.push_back( a_segments.size() );
    changed();
}

void PathWindow::undo()
{
    if( m_segments.empty() )
    {
        return;
    }
    // After a removal the groups no longer match, then one segment is undone at a time.
    size_t count = 1;
    if( !m_added.empty() )
    {
        count = std::min( m_added.back(), m_segments.size() );
        m_added.pop_back();
    }
    m_segments.resize( m_segments.size() - count );
    changed();
}

void PathWindow::removeSelected()
{
    std::set< int > rows;
    for( const auto& range : m_table->selectedRanges() )
    {
        for( int row = range.topRow(); row <= range.bottomRow(); row++ )
        {
            rows.insert( row );
        }
    }
    if( rows.empty() )
    {
        return;
    }
    for( auto row = rows.rbegin(); row != rows.rend(); ++row )
    {
        m_segments.erase( m_segments.begin() + *row );
    }
    m_added.clear();
    changed();
}

void PathWindow::changed()
{
    m_canvas->setSegments( m_segments );
    fillTable();
    showPose( PathCanvas::Layer::Planned, m_segments.empty() ? std::nullopt : std::optional( endPose() ) );
}

void PathWindow::fillTable()
{
    const QSignalBlocker blocker( m_table );
    m_table->setRowCount( static_cast< int >( m_segments.size() ) );
    const auto cell = [ this ]( int a_row, int a_column, const QString& a_text, bool a_editable )
    {
        auto* item = new QTableWidgetItem( a_text );
        if( !a_editable )
        {
            item->setFlags( item->flags() & ~Qt::ItemIsEditable );
        }
        if( m_running == a_row )
        {
            item->setBackground( QColor( 255, 200, 140 ) );
            item->setForeground( Qt::black );
        }
        m_table->setItem( a_row, a_column, item );
    };

    for( int row = 0; row < static_cast< int >( m_segments.size() ); row++ )
    {
        const auto& segment = m_segments[ static_cast< size_t >( row ) ];
        if( segment.has_line() )
        {
            cell( row, Type, "Line", false );
            cell( row, Length, number( segment.line().distance_cm() ), true );
            cell( row, Angle, {}, false );
        }
        else if( segment.has_turn() )
        {
            cell( row, Type, "Turn", false );
            cell( row, Length, {}, false );
            cell( row, Angle, number( segment.turn().angle_deg() ), true );
        }
        else
        {
            // The length of an arc is its radius.
            cell( row, Type, segment.arc().backward() ? "Arc back" : "Arc", false );
            cell( row, Length, number( segment.arc().radius_cm() ), true );
            cell( row, Angle, number( segment.arc().angle_deg() ), true );
        }
        cell( row, Speed, segment.speed_cm_s() > 0 ? number( segment.speed_cm_s() ) : QString(), true );
    }
}

void PathWindow::cellEdited( int a_row, int a_column )
{
    auto& segment = m_segments[ static_cast< size_t >( a_row ) ];
    const QString text = m_table->item( a_row, a_column )->text().trimmed();
    // An empty speed uses the speed of the path.
    const auto value = a_column == Speed && text.isEmpty() ? std::optional( 0.0 ) : parse( text );
    if( value )
    {
        switch( a_column )
        {
            case Length:
                if( segment.has_line() )
                {
                    segment.mutable_line()->set_distance_cm( *value );
                }
                else if( *value >= 0 )
                {
                    segment.mutable_arc()->set_radius_cm( *value );
                }
                break;
            case Angle:
                if( segment.has_turn() )
                {
                    segment.mutable_turn()->set_angle_deg( *value );
                }
                else
                {
                    segment.mutable_arc()->set_angle_deg( *value );
                }
                break;
            case Speed:
                segment.set_speed_cm_s( std::max( *value, 0.0 ) );
                break;
        }
    }
    // Shows the value that is used, also when the text was not a number.
    changed();
}

void PathWindow::showRunning( std::optional< int > a_segment )
{
    m_running = a_segment;
    m_canvas->setRunning( a_segment );
    fillTable();
}

void PathWindow::run()
{
    if( m_segments.empty() )
    {
        m_status->setText( "Draw a path first." );
        return;
    }
    path_msgs::FollowPath msg;
    msg.set_id( ++m_pathId );
    for( const auto& segment : m_segments )
    {
        *msg.add_segments() = segment;
    }
    msg.set_speed_cm_s( m_speed->value() );
    msg.set_acceleration_cm_s2( m_acceleration->value() );
    sepia::comm2::Dispatcher< path_msgs::FollowPath >::send( &msg );

    m_canvas->clearTrace();
    showPose( PathCanvas::Layer::Encoders, std::nullopt );
    showPose( PathCanvas::Layer::Imu, std::nullopt );
    showRunning( std::nullopt );
    m_answered = false;
    m_answerTimer->start();
    m_status->setText( "Sent, waiting for PathFollower." );
}

void PathWindow::receive( const path_msgs::PathProgress& a_msg )
{
    if( a_msg.id() != m_pathId )
    {
        return;
    }
    m_answered = true;
    const QString message = QString::fromStdString( a_msg.message() );
    switch( a_msg.state() )
    {
        case path_msgs::PathProgress::RUNNING:
            showRunning( static_cast< int >( a_msg.segment() ) );
            m_status->setText( QString( "Driving segment %1 of %2." ).arg( a_msg.segment() + 1 ).arg( a_msg.segment_count() ) );
            break;
        case path_msgs::PathProgress::DONE:
            showRunning( std::nullopt );
            m_status->setText( message.isEmpty() ? "Done." : "Done: " + message );
            break;
        case path_msgs::PathProgress::STOPPED:
            showRunning( std::nullopt );
            m_status->setText( "Stopped: " + message );
            break;
        default:
            showRunning( std::nullopt );
            m_status->setText( "Failed: " + message );
            break;
    }
}

void PathWindow::receive( const path_msgs::PathPose& a_msg )
{
    if( a_msg.id() != m_pathId )
    {
        return;
    }
    const Pose encoders{ .x = a_msg.encoder_x_cm(), .y = a_msg.encoder_y_cm(), .heading = a_msg.encoder_heading_deg() * std::numbers::pi / 180 };
    // Without the IMU the other pose is the same as the encoder one, so it is not drawn.
    std::optional< Pose > imu;
    if( a_msg.imu_heading() )
    {
        imu = Pose{ .x = a_msg.x_cm(), .y = a_msg.y_cm(), .heading = a_msg.heading_deg() * std::numbers::pi / 180 };
    }
    m_canvas->addTrace( encoders, imu );
    showPose( PathCanvas::Layer::Encoders, encoders );
    showPose( PathCanvas::Layer::Imu, imu );
}

void PathWindow::showPose( PathCanvas::Layer a_layer, const std::optional< Pose >& a_pose )
{
    auto& labels = m_poses[ a_layer ];
    if( !a_pose )
    {
        for( QLabel* label : labels )
        {
            label->setText( "–" );
        }
        return;
    }
    labels[ 0 ]->setText( number( a_pose->x ) );
    labels[ 1 ]->setText( number( a_pose->y ) );
    labels[ 2 ]->setText( number( a_pose->heading * 180 / std::numbers::pi ) );
}
