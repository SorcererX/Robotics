#ifndef MOVEMENT_WIDGET_H
#define MOVEMENT_WIDGET_H
#include <QWidget>
#include "roboclaw_msgs.pb.h"

namespace Ui
{
    class MovementWidget;
}

class MovementWidget : public QWidget
{
    Q_OBJECT
public:
    MovementWidget( QWidget* a_parent );
    ~MovementWidget();

private:
    Ui::MovementWidget* m_ui;
    roboclaw_msgs::Move m_msg;
};

#endif // MOVEMENT_WIDGET_H