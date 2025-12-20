#ifndef MOVEMENT_WIDGET_H
#define MOVEMENT_WIDGET_H
#include <QWidget>
#include "roboclaw_msgs.pb.h"
#include <QSpinBox>
#include <sepia/comm2/observer.h>

namespace Ui
{
    class MovementWidget;
}

class MovementWidget : public QWidget,
                       public sepia::comm2::Observer< roboclaw_msgs::MoveStatus >
{
    Q_OBJECT
public:
    MovementWidget( QWidget* a_parent );
    ~MovementWidget();

protected:
    void receive( const roboclaw_msgs::MoveStatus& a_message );
    void addConnect( QDoubleSpinBox* a_object, const std::function< void( int32_t ) >& a_lambda );

private:
    Ui::MovementWidget* m_ui;
    roboclaw_msgs::MovePosition m_msg;
    QTimer* m_statusRequestTimer;
};

#endif // MOVEMENT_WIDGET_H
