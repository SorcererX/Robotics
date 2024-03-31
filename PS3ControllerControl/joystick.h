#include <linux/joystick.h>
#include <string>
#include <sepia/util/threadobject.h>

class Joystick : public sepia::util::ThreadObject
{
public:
    Joystick( const std::string& a_device = "/dev/input/js0" );
    bool init();
    ~Joystick();

protected:
    void own_thread() override;
    virtual void process_event( const struct js_event& e ) = 0;
    uint8_t m_axises{ 0 };
    uint8_t m_buttons{ 0 };

private:
    std::string m_device;
    int m_fd{ -1 };
};