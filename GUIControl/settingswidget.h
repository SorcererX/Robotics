#ifndef SETTINGS_WIDGET_H
#define SETTINGS_WIDGET_H
#include <QWidget>
#include <map>
#include "roboclaw_msgs.pb.h"
#include <sepia/comm2/observer.h>

class QDoubleSpinBox;
class QGridLayout;
class QLabel;

class SettingsWidget : public QWidget,
                       public sepia::comm2::Observer< roboclaw_msgs::SettingsReport >
{
    Q_OBJECT
public:
    SettingsWidget( QWidget* a_parent );

protected:
    void receive( const roboclaw_msgs::SettingsReport& a_message );

private:
    using Field = const google::protobuf::FieldDescriptor*;

    // Adds a row with a spin box for each message, for all fields of the message type.
    void addRows( QGridLayout* a_layout, const google::protobuf::Descriptor* a_type, int a_columns );
    QDoubleSpinBox* createSpinBox( Field a_field );
    void show( const google::protobuf::Message& a_message, Field a_field, QDoubleSpinBox* a_box );
    void collect( google::protobuf::Message& a_message, Field a_field, const QDoubleSpinBox* a_box ) const;
    void updateChanged();
    bool confirmed( const QString& a_question );
    bool confirmConfigChange( uint32_t a_config );

    std::map< Field, QDoubleSpinBox* > m_general;
    std::map< Field, QDoubleSpinBox* > m_m1;
    std::map< Field, QDoubleSpinBox* > m_m2;
    roboclaw_msgs::Settings m_settings;
    bool m_received = false;
    QLabel* m_status;
};

#endif // SETTINGS_WIDGET_H
