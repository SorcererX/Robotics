#ifndef MOVEMENT_WIDGET_H
#define MOVEMENT_WIDGET_H
#include <QWidget>
#include "roboclaw_msgs.pb.h"
#include <QSpinBox>
#include <functional>
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
    /// Which commands the widget shows, Status and Stop are always shown.
    enum class Commands
    {
        Cm,
        Raw
    };

    MovementWidget( QWidget* a_parent, Commands a_commands );
    ~MovementWidget();

protected:
    void showEvent( QShowEvent* a_event ) override;
    void hideEvent( QHideEvent* a_event ) override;
    void receive( const roboclaw_msgs::MoveStatus& a_message );
    void addConnect( QDoubleSpinBox* a_object, const std::function< void( int32_t ) >& a_lambda );
    void linkPercentToRaw( QDoubleSpinBox* a_percent, QSpinBox* a_raw );
    static QString errorText( uint32_t a_error );

private:
    Ui::MovementWidget* m_ui;
    roboclaw_msgs::MovePosition m_msg;
    QTimer* m_statusRequestTimer;
};

#endif // MOVEMENT_WIDGET_H
