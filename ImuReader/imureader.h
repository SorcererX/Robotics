#include "imu_msgs.pb.h"
#include <sepia/comm2/observer.h>
#include <sepia/util/threadobject.h>
#include <tss/api/sensor.h>
#include <tss/com/serial.h>
#include <tss/errors.h>
#include <array>
#include <chrono>
#include <string>

// Reads a Yost Labs 3-Space v3 sensor over USB and publishes the heading of the robot as ImuData. The heading
// comes from the gyro alone, a magnetometer is unreliable next to the motors. The magnetometer heading is
// published separately, for comparing.
class ImuReader : public sepia::util::ThreadObject,
                   public sepia::comm2::Observer< imu_msgs::ResetHeading >
{
public:
    ImuReader();
    ~ImuReader();

protected:
    void receive( const imu_msgs::ResetHeading& a_msg ) override;
    void own_thread() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector = std::array< double, 3 >;

    bool connect();
    void disconnect();
    // Called by the C API for each streamed sample.
    static TSS_DataCallbackState onSample( TSS_Sensor* a_sensor );
    void sample( const float* a_gyro, const float* a_accel, const float* a_mag, float a_motionless, uint32_t a_timestamp );
    void updateCompass( const Vector& a_mag );

    // From sepia.yaml.
    std::string m_device;
    double m_streamHz{ 200 };
    double m_publishHz{ 50 };
    double m_headingSign{ 1 };
    double m_compassSign{ 1 };

    SerialComClass m_serial{};
    TSS_Sensor m_sensor{};
    bool m_connected{ false };
    // Set while no sensor is found, so it is only reported once.
    bool m_waiting{ false };
    Clock::time_point m_lastSample;
    Clock::time_point m_lastAttempt;

    // In radians and radians per second.
    double m_heading{ 0 };
    double m_offset{ 0 };
    // Gravity seen by the sensor, filtered, pointing up.
    Vector m_up{};
    bool m_haveSample{ false };
    uint32_t m_lastTimestamp{ 0 };
    // How long the sensor has been still, in seconds.
    double m_stillTime{ 0 };
    // Time until the next ImuData, in seconds.
    double m_untilPublish{ 0 };

    // The compass heading in radians, counting past whole turns, and the last angle it was updated with.
    double m_compass{ 0 };
    double m_compassAngle{ 0 };
    bool m_haveCompass{ false };
    // The sensor axis that is closest to horizontal, the compass measures the field's angle from it.
    int m_compassAxis{ -1 };
};
