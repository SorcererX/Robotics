#ifndef FOLLOWER_SETTINGS_WIDGET_H
#define FOLLOWER_SETTINGS_WIDGET_H
#include "path_msgs.pb.h"
#include <QWidget>
#include <functional>
#include <optional>
#include <sepia/comm2/observer.h>
#include <vector>

class QCheckBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QPushButton;
class QTimer;

// Shows and changes PathFollower's settings. Fields that differ from what PathFollower reported are marked
// until they are applied.
class FollowerSettingsWidget : public QWidget,
                               public sepia::comm2::Observer< path_msgs::FollowerSettingsReport >
{
    Q_OBJECT
public:
    explicit FollowerSettingsWidget( QWidget* a_parent = nullptr );

protected:
    void receive( const path_msgs::FollowerSettingsReport& a_msg ) override;
    void showEvent( QShowEvent* a_event ) override;

private:
    struct Field
    {
        QDoubleSpinBox* box;
        // The key in sepia.yaml.
        const char* key;
        std::function< double( const path_msgs::FollowerSettings& ) > get;
        std::function< void( path_msgs::FollowerSettings&, double ) > set;
    };

    void addField( QFormLayout* a_form, const QString& a_label, const QString& a_tooltip, const char* a_key, double a_min, double a_max, int a_decimals, const QString& a_suffix,
                   std::function< double( const path_msgs::FollowerSettings& ) > a_get,
                   std::function< void( path_msgs::FollowerSettings&, double ) > a_set );
    path_msgs::FollowerSettings entered() const;
    void markChanged();
    void request();
    void apply();
    void copyYaml();

    std::vector< Field > m_fields;
    QCheckBox* m_useImu;
    QPushButton* m_apply;
    QLabel* m_status;
    QTimer* m_answerTimer;
    // What PathFollower last reported, the fields are compared with it.
    std::optional< path_msgs::FollowerSettings > m_reported;
    bool m_answered{ true };
};

#endif // FOLLOWER_SETTINGS_WIDGET_H
