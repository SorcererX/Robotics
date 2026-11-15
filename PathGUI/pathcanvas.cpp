#include "pathcanvas.h"
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    // A press that moves less than this is a click.
    constexpr double clickDistance = 4;

    const QColor pathColor( 40, 110, 200 );
    const QColor runningColor( 230, 120, 20 );
    const QColor previewColor( 120, 120, 120 );
    const QColor traceColor( 30, 160, 70 );
}

PathCanvas::PathCanvas( QWidget* a_parent )
    : QWidget( a_parent )
{
    setMouseTracking( true );
    setMinimumSize( 400, 400 );
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
}

void PathCanvas::setSegments( const std::vector< path_msgs::PathSegment >& a_segments )
{
    m_segments = a_segments;
    update();
}

void PathCanvas::setPreview( const std::vector< path_msgs::PathSegment >& a_segments )
{
    m_preview = a_segments;
    update();
}

void PathCanvas::setRunning( std::optional< int > a_segment )
{
    m_running = a_segment;
    update();
}

void PathCanvas::clearTrace()
{
    m_trace.clear();
    update();
}

void PathCanvas::addTrace( const Pose& a_pose )
{
    m_trace.push_back( a_pose );
    update();
}

void PathCanvas::setSnap( double a_snap )
{
    m_snap = a_snap;
}

void PathCanvas::fit()
{
    // The start is always in view.
    double minX = 0, maxX = 0, minY = 0, maxY = 0;
    Pose pose{ 0, 0, 0 };
    const auto include = [ & ]( const Pose& a_pose )
    {
        minX = std::min( minX, a_pose.x );
        maxX = std::max( maxX, a_pose.x );
        minY = std::min( minY, a_pose.y );
        maxY = std::max( maxY, a_pose.y );
    };
    for( const auto& segment : m_segments )
    {
        for( const Pose& point : sampleSegment( pose, segment ) )
        {
            include( point );
        }
        pose = poseAlong( pose, segment, 1 );
    }
    for( const Pose& point : m_trace )
    {
        include( point );
    }

    // At least a metre is shown, with a margin around the path.
    const double margin = 40;
    const double height = std::max( maxX - minX, 100.0 ) + 2 * margin;
    const double width = std::max( maxY - minY, 100.0 ) + 2 * margin;
    m_scale = std::min( this->width() / width, this->height() / height );
    // The x axis points up and the y axis to the left.
    const double centreX = ( minX + maxX ) / 2;
    const double centreY = ( minY + maxY ) / 2;
    m_origin = QPointF( this->width() / 2.0 + centreY * m_scale, this->height() / 2.0 + centreX * m_scale );
    m_originSet = true;
    update();
}

QPointF PathCanvas::toScreen( double a_x, double a_y ) const
{
    return { m_origin.x() - a_y * m_scale, m_origin.y() - a_x * m_scale };
}

QPointF PathCanvas::toScreen( const Pose& a_pose ) const
{
    return toScreen( a_pose.x, a_pose.y );
}

std::pair< double, double > PathCanvas::toWorld( const QPointF& a_screen ) const
{
    return { ( m_origin.y() - a_screen.y() ) / m_scale, ( m_origin.x() - a_screen.x() ) / m_scale };
}

std::pair< double, double > PathCanvas::snapped( const QPointF& a_screen ) const
{
    auto [ x, y ] = toWorld( a_screen );
    if( m_snap > 0 )
    {
        x = std::round( x / m_snap ) * m_snap;
        y = std::round( y / m_snap ) * m_snap;
    }
    return { x, y };
}

Pose PathCanvas::endPose() const
{
    Pose pose{ 0, 0, 0 };
    for( const auto& segment : m_segments )
    {
        pose = poseAlong( pose, segment, 1 );
    }
    return pose;
}

void PathCanvas::paintEvent( QPaintEvent* )
{
    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );
    painter.fillRect( rect(), palette().base() );

    drawGrid( painter );

    drawRobot( painter, { 0, 0, 0 }, palette().text().color(), false );
    drawSegments( painter, { 0, 0, 0 }, m_segments, pathColor, Qt::SolidLine, m_running );
    const Pose end = endPose();
    if( !m_segments.empty() )
    {
        drawRobot( painter, end, pathColor, false );
    }
    drawSegments( painter, end, m_preview, previewColor, Qt::DashLine, std::nullopt );

    if( m_trace.size() > 1 )
    {
        QPainterPath trace( toScreen( m_trace.front() ) );
        for( const Pose& pose : m_trace )
        {
            trace.lineTo( toScreen( pose ) );
        }
        painter.setPen( QPen( traceColor, 2 ) );
        painter.setBrush( Qt::NoBrush );
        painter.drawPath( trace );
    }
    if( !m_trace.empty() )
    {
        drawRobot( painter, m_trace.back(), traceColor, true );
    }

    // The point a click adds, and where it is.
    painter.setPen( palette().text().color() );
    QString text = QString( "Grid %1 cm" ).arg( 10 * m_scale >= 8 ? 10 : 100 );
    if( m_hover )
    {
        const QPointF point = toScreen( m_hover->first, m_hover->second );
        painter.drawLine( point + QPointF( -5, 0 ), point + QPointF( 5, 0 ) );
        painter.drawLine( point + QPointF( 0, -5 ), point + QPointF( 0, 5 ) );
        text += QString( "   x %1 cm  y %2 cm" ).arg( QLocale().toString( m_hover->first, 'f', 1 ), QLocale().toString( m_hover->second, 'f', 1 ) );
    }
    painter.drawText( rect().adjusted( 8, 4, -8, -4 ), Qt::AlignLeft | Qt::AlignBottom, text );
}

void PathCanvas::drawGrid( QPainter& a_painter ) const
{
    // 10 cm lines when they are far enough apart, and metre lines darker.
    const double step = 10 * m_scale >= 8 ? 10 : 100;
    const auto [ topX, leftY ] = toWorld( QPointF( 0, 0 ) );
    const auto [ bottomX, rightY ] = toWorld( QPointF( width(), height() ) );
    const QColor minor = palette().mid().color();
    QColor light = minor;
    light.setAlpha( 60 );
    QColor major = minor;
    major.setAlpha( 140 );

    for( double x = std::floor( bottomX / step ) * step; x <= topX; x += step )
    {
        a_painter.setPen( std::fmod( std::abs( x ), 100 ) < 0.5 ? major : light );
        a_painter.drawLine( toScreen( x, leftY ), toScreen( x, rightY ) );
    }
    for( double y = std::floor( rightY / step ) * step; y <= leftY; y += step )
    {
        a_painter.setPen( std::fmod( std::abs( y ), 100 ) < 0.5 ? major : light );
        a_painter.drawLine( toScreen( topX, y ), toScreen( bottomX, y ) );
    }
}

void PathCanvas::drawSegments( QPainter& a_painter, const Pose& a_start, const std::vector< path_msgs::PathSegment >& a_segments, const QColor& a_color,
                               Qt::PenStyle a_style, std::optional< int > a_highlight ) const
{
    Pose pose = a_start;
    for( size_t i = 0; i < a_segments.size(); i++ )
    {
        const auto& segment = a_segments[ i ];
        const bool highlight = a_highlight && *a_highlight == static_cast< int >( i );
        const QColor color = highlight ? runningColor : a_color;
        a_painter.setPen( QPen( color, highlight ? 4 : 2, a_style, Qt::RoundCap ) );
        a_painter.setBrush( Qt::NoBrush );

        if( segment.has_turn() )
        {
            // A turn on the spot is a ring with the new heading.
            const QPointF centre = toScreen( pose );
            a_painter.drawEllipse( centre, 8, 8 );
        }
        else
        {
            const auto points = sampleSegment( pose, segment );
            QPainterPath line( toScreen( points.front() ) );
            for( const Pose& point : points )
            {
                line.lineTo( toScreen( point ) );
            }
            a_painter.drawPath( line );
        }
        pose = poseAlong( pose, segment, 1 );
        a_painter.setPen( Qt::NoPen );
        a_painter.setBrush( color );
        a_painter.drawEllipse( toScreen( pose ), 3, 3 );
    }
}

void PathCanvas::drawRobot( QPainter& a_painter, const Pose& a_pose, const QColor& a_color, bool a_filled ) const
{
    // A triangle pointing in the heading, the same size at any zoom. On the screen x is up and y is left.
    const QPointF centre = toScreen( a_pose );
    const QPointF forward( -std::sin( a_pose.heading ), -std::cos( a_pose.heading ) );
    const QPointF side( -forward.y(), forward.x() );
    const QPointF points[] = { centre + forward * 14, centre - forward * 8 + side * 9, centre - forward * 8 - side * 9 };
    a_painter.setPen( QPen( a_color, 2 ) );
    a_painter.setBrush( a_filled ? QBrush( a_color ) : Qt::NoBrush );
    a_painter.drawPolygon( points, 3 );
}

void PathCanvas::mousePressEvent( QMouseEvent* a_event )
{
    m_pressedAt = a_event->localPos();
    m_pressedOrigin = m_origin;
    m_dragging = false;
}

void PathCanvas::mouseMoveEvent( QMouseEvent* a_event )
{
    if( m_pressedAt )
    {
        const QPointF moved = a_event->localPos() - *m_pressedAt;
        if( m_dragging || std::hypot( moved.x(), moved.y() ) > clickDistance )
        {
            m_dragging = true;
            m_origin = m_pressedOrigin + moved;
            update();
            return;
        }
    }
    const auto [ x, y ] = snapped( a_event->localPos() );
    m_hover = { x, y };
    emit pointHovered( x, y );
    update();
}

void PathCanvas::mouseReleaseEvent( QMouseEvent* a_event )
{
    const bool click = m_pressedAt && !m_dragging && a_event->button() == Qt::LeftButton;
    m_pressedAt.reset();
    m_dragging = false;
    if( click )
    {
        const auto [ x, y ] = snapped( a_event->localPos() );
        emit pointClicked( x, y );
    }
}

void PathCanvas::wheelEvent( QWheelEvent* a_event )
{
    // Zooms around the mouse, so the point under it stays there.
    const double factor = std::pow( 1.0015, a_event->angleDelta().y() );
    const double scale = std::clamp( m_scale * factor, 0.1, 50.0 );
    const QPointF mouse = a_event->position();
    m_origin = mouse + ( m_origin - mouse ) * ( scale / m_scale );
    m_scale = scale;
    update();
}

void PathCanvas::leaveEvent( QEvent* )
{
    m_hover.reset();
    emit hoverEnded();
    update();
}

void PathCanvas::resizeEvent( QResizeEvent* )
{
    // The start begins near the bottom, so there is room to draw forward.
    if( !m_originSet )
    {
        m_origin = QPointF( width() / 2.0, height() * 0.8 );
    }
}
