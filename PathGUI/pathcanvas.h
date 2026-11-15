#ifndef PATH_CANVAS_H
#define PATH_CANVAS_H
#include "pathgeometry.h"
#include <QPointF>
#include <QWidget>
#include <optional>
#include <vector>

class QPainter;

// Draws the path seen from above with the start at the bottom, facing up. A left click adds a point, dragging
// moves the view and the wheel zooms.
class PathCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit PathCanvas( QWidget* a_parent = nullptr );
    void setSegments( const std::vector< path_msgs::PathSegment >& a_segments );
    // Drawn from the end of the path, for showing what a click would add.
    void setPreview( const std::vector< path_msgs::PathSegment >& a_segments );
    void setRunning( std::optional< int > a_segment );
    void clearTrace();
    void addTrace( const Pose& a_pose );
    // Grid in cm that clicked points snap to, 0 for none.
    void setSnap( double a_snap );
    // Zooms and moves the view so the path fits.
    void fit();

signals:
    void pointClicked( double a_x, double a_y );
    void pointHovered( double a_x, double a_y );
    void hoverEnded();

protected:
    void paintEvent( QPaintEvent* a_event ) override;
    void mousePressEvent( QMouseEvent* a_event ) override;
    void mouseMoveEvent( QMouseEvent* a_event ) override;
    void mouseReleaseEvent( QMouseEvent* a_event ) override;
    void wheelEvent( QWheelEvent* a_event ) override;
    void leaveEvent( QEvent* a_event ) override;
    void resizeEvent( QResizeEvent* a_event ) override;

private:
    QPointF toScreen( double a_x, double a_y ) const;
    QPointF toScreen( const Pose& a_pose ) const;
    std::pair< double, double > toWorld( const QPointF& a_screen ) const;
    std::pair< double, double > snapped( const QPointF& a_screen ) const;
    void drawGrid( QPainter& a_painter ) const;
    void drawSegments( QPainter& a_painter, const Pose& a_start, const std::vector< path_msgs::PathSegment >& a_segments, const QColor& a_color,
                       Qt::PenStyle a_style, std::optional< int > a_highlight ) const;
    void drawRobot( QPainter& a_painter, const Pose& a_pose, const QColor& a_color, bool a_filled ) const;
    Pose endPose() const;

    std::vector< path_msgs::PathSegment > m_segments;
    std::vector< path_msgs::PathSegment > m_preview;
    std::optional< int > m_running;
    std::vector< Pose > m_trace;
    double m_snap{ 5 };
    // Pixels per cm, and where the start is on the screen.
    double m_scale{ 3 };
    QPointF m_origin;
    bool m_originSet{ false };
    // Set while a button is down, a press that moves further than a few pixels drags the view instead of clicking.
    std::optional< QPointF > m_pressedAt;
    QPointF m_pressedOrigin;
    bool m_dragging{ false };
    std::optional< std::pair< double, double > > m_hover;
};

#endif // PATH_CANVAS_H
