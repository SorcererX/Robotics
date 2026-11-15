#ifndef PATH_WINDOW_H
#define PATH_WINDOW_H
#include "path_msgs.pb.h"
#include "pathgeometry.h"
#include <QWidget>
#include <sepia/comm2/observer.h>
#include <optional>
#include <vector>

class PathCanvas;
class QDoubleSpinBox;
class QLabel;
class QRadioButton;
class QTableWidget;
class QTimer;

// Draws a path, sends it to PathFollower and shows where the robot went.
class PathWindow : public QWidget,
                   public sepia::comm2::Observer< path_msgs::PathProgress >,
                   public sepia::comm2::Observer< path_msgs::PathPose >
{
    Q_OBJECT
public:
    explicit PathWindow( QWidget* a_parent = nullptr );

protected:
    void receive( const path_msgs::PathProgress& a_msg ) override;
    void receive( const path_msgs::PathPose& a_msg ) override;

private:
    void add( const std::vector< path_msgs::PathSegment >& a_segments );
    void undo();
    void removeSelected();
    void changed();
    void fillTable();
    void cellEdited( int a_row, int a_column );
    void showRunning( std::optional< int > a_segment );
    void run();
    Pose endPose() const;
    std::vector< path_msgs::PathSegment > segmentsTo( double a_x, double a_y ) const;

    PathCanvas* m_canvas;
    QRadioButton* m_straight;
    QTableWidget* m_table;
    QDoubleSpinBox* m_speed;
    QDoubleSpinBox* m_acceleration;
    QLabel* m_status;
    QTimer* m_answerTimer;

    std::vector< path_msgs::PathSegment > m_segments;
    // How many segments each click added, so undo removes a turn and its line together.
    std::vector< size_t > m_added;
    std::optional< int > m_running;
    // The id of the last path sent, progress of other paths is ignored.
    uint32_t m_pathId;
    bool m_answered{ true };
};

#endif // PATH_WINDOW_H
