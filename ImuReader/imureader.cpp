#include "imureader.h"
#include <sepia/comm2/dispatcher.h>
#include <sepia/comm2/receiver.h>
#include <sepia/comm2/scopedmessagesender.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <yaml-cpp/yaml.h>

namespace
{
    using namespace std::chrono_literals;

    // USB vendor id of Yost Labs.
    constexpr const char* yostVendor = "2476";
    // How long to wait between attempts to find and connect to the sensor.
    constexpr auto retryInterval = 2s;
    // The sensor is reconnected when it sends nothing for this long.
    constexpr auto sampleTimeout = 1s;
    // Seconds for the filter of the direction of gravity, long enough to hide the robot's own acceleration.
    constexpr double upFilter = 1;
    // The sensor counts as still when it says so and turns slower than this, in radians per second.
    constexpr double stillRate = 1 * std::numbers::pi / 180;
    // How long it has to be still, in seconds, before the offset is measured.
    constexpr double stillDelay = 0.5;
    // Seconds for the filter of the gyro offset.
    constexpr double offsetFilter = 2;
    // The sensor's motionless confidence, 0 to 1, that counts as still.
    constexpr float motionlessConfidence = 0.9f;
    // A horizontal field weaker than this in gauss gives no compass heading, the Earth's is 0.1 to 0.4.
    constexpr double minimumField = 0.02;

    constexpr double toDegrees = 180 / std::numbers::pi;

    using Vector = std::array< double, 3 >;

    double dot( const Vector& a_a, const Vector& a_b )
    {
        return a_a[ 0 ] * a_b[ 0 ] + a_a[ 1 ] * a_b[ 1 ] + a_a[ 2 ] * a_b[ 2 ];
    }

    Vector scaled( const Vector& a_vector, double a_factor )
    {
        return { a_vector[ 0 ] * a_factor, a_vector[ 1 ] * a_factor, a_vector[ 2 ] * a_factor };
    }

    Vector cross( const Vector& a_a, const Vector& a_b )
    {
        return { a_a[ 1 ] * a_b[ 2 ] - a_a[ 2 ] * a_b[ 1 ], a_a[ 2 ] * a_b[ 0 ] - a_a[ 0 ] * a_b[ 2 ], a_a[ 0 ] * a_b[ 1 ] - a_a[ 1 ] * a_b[ 0 ] };
    }

    // The part of the vector that is at right angles to a_up.
    Vector horizontal( const Vector& a_vector, const Vector& a_up )
    {
        const double along = dot( a_vector, a_up );
        return { a_vector[ 0 ] - a_up[ 0 ] * along, a_vector[ 1 ] - a_up[ 1 ] * along, a_vector[ 2 ] - a_up[ 2 ] * along };
    }

    struct Config
    {
        std::string device;
        double streamHz = 200;
        double publishHz = 50;
        double headingSign = 1;
        double compassSign = 1;
    };

    // Reads imu_reader from the same file SepiaComm uses.
    Config loadConfig()
    {
        const char* configured = std::getenv( "SEPIA_CONFIG" );
        const std::string path = configured != nullptr && *configured != '\0' ? configured : "sepia.yaml";
        Config config;
        std::error_code ec;
        if( std::filesystem::exists( path, ec ) )
        {
            try
            {
                if( const YAML::Node node = YAML::LoadFile( path )[ "imu_reader" ] )
                {
                    config.device = node[ "device" ].as< std::string >( config.device );
                    config.streamHz = node[ "stream_hz" ].as< double >( config.streamHz );
                    config.publishHz = node[ "publish_hz" ].as< double >( config.publishHz );
                    config.headingSign = node[ "heading_sign" ].as< double >( config.headingSign ) < 0 ? -1 : 1;
                    config.compassSign = node[ "compass_sign" ].as< double >( config.compassSign ) < 0 ? -1 : 1;
                }
            }
            catch( const YAML::Exception& ex )
            {
                std::cerr << "Unable to read imu_reader from '" << path << "': " << ex.what() << std::endl;
            }
        }
        if( !( config.streamHz > 0 ) || !( config.publishHz > 0 ) )
        {
            std::cerr << "stream_hz and publish_hz must be above 0, using 200 and 50." << std::endl;
            config.streamHz = 200;
            config.publishHz = 50;
        }
        return config;
    }

    // The first serial port of a Yost Labs USB device, so nothing is sent to the other serial devices.
    std::string findDevice()
    {
        std::error_code ec;
        for( const auto& entry : std::filesystem::directory_iterator( "/sys/class/tty", ec ) )
        {
            const std::string name = entry.path().filename().string();
            if( !name.starts_with( "ttyACM" ) && !name.starts_with( "ttyUSB" ) )
            {
                continue;
            }
            std::ifstream vendor( entry.path() / "device" / ".." / "idVendor" );
            std::string id;
            if( vendor >> id && id == yostVendor )
            {
                return "/dev/" + name;
            }
        }
        return {};
    }

    // The C API numbers ports instead of naming them: ttyUSB0-127 are 0-127 and ttyACM0-127 are 128-255.
    std::optional< uint8_t > toPort( const std::string& a_device )
    {
        std::error_code ec;
        const std::string name = std::filesystem::canonical( a_device, ec ).filename().string();
        if( ec )
        {
            return std::nullopt;
        }
        for( const auto& [ prefix, first ] : { std::pair{ std::string( "ttyUSB" ), 0 }, std::pair{ std::string( "ttyACM" ), 128 } } )
        {
            if( name.starts_with( prefix ) && name.size() > prefix.size() )
            {
                const int number = std::atoi( name.c_str() + prefix.size() );
                if( number >= 0 && number < 128 )
                {
                    return static_cast< uint8_t >( first + number );
                }
            }
        }
        return std::nullopt;
    }
}

ImuReader::ImuReader()
{
    const Config config = loadConfig();
    m_device = config.device;
    m_streamHz = config.streamHz;
    m_publishHz = config.publishHz;
    m_headingSign = config.headingSign;
    m_compassSign = config.compassSign;
    std::cout << "Device: " << ( m_device.empty() ? "found by USB vendor id" : m_device ) << ", stream: " << m_streamHz << " Hz, publish: " << m_publishHz
              << " Hz, heading sign: " << m_headingSign << ", compass sign: " << m_compassSign << std::endl;
}

ImuReader::~ImuReader()
{
}

void ImuReader::own_thread()
{
    sepia::comm2::ScopedMessageSender sender;
    // Returns regularly without messages, so the sensor can be read.
    sepia::comm2::Receiver handler( this, 5 );
    while( !m_terminate && handler.exec() )
    {
        const auto now = Clock::now();
        if( !m_connected )
        {
            if( now - m_lastAttempt >= retryInterval )
            {
                m_lastAttempt = now;
                connect();
            }
            continue;
        }

        // Limited, so ResetHeading is still handled if the sensor sends faster than it is read.
        for( int i = 0; i < 100 && sensorUpdateStreaming( &m_sensor ); i++ )
        {
        }
        if( Clock::now() - m_lastSample > sampleTimeout )
        {
            std::cout << "No data from the sensor, reconnecting." << std::endl;
            disconnect();
        }
    }
    if( m_connected )
    {
        disconnect();
    }
}

bool ImuReader::connect()
{
    const std::string device = m_device.empty() ? findDevice() : m_device;
    const auto port = device.empty() ? std::nullopt : toPort( device );
    if( !port )
    {
        // Only said once, the search is repeated quietly.
        if( !m_waiting )
        {
            std::cout << ( device.empty() ? std::string( "No Yost Labs sensor found" ) : "'" + device + "' is not a ttyACM or ttyUSB device" )
                      << ", retrying every " << std::chrono::duration_cast< std::chrono::seconds >( retryInterval ).count() << " s." << std::endl;
            m_waiting = true;
        }
        return false;
    }
    m_waiting = false;

    std::cout << "Connecting to " << device << std::endl;
    create_serial_com_class( *port, &m_serial );
    auto* com = reinterpret_cast< TSS_Com_Class* >( &m_serial );
    if( tss_com_open( com ) != TSS_SUCCESS )
    {
        std::cout << "Unable to open " << device << std::endl;
        return false;
    }
    tssCreateSensor( &m_sensor, com );
    if( const int err = tssInitSensor( &m_sensor ); err != TSS_SUCCESS )
    {
        std::cout << "Unable to initialize the sensor: " << err << std::endl;
        tss_com_close( com );
        return false;
    }
    m_sensor.user_data = this;

    // The corrected gyro rate, acceleration and magnetic field, and the motionless confidence, in the order
    // onSample reads them. The timestamp in the header gives the time between samples.
    const std::pair< const char*, int > steps[] = {
        { "stream slots", sensorWriteStreamSlots( &m_sensor, "38,39,40,45" ) },
        { "stream rate", sensorWriteStreamHz( &m_sensor, static_cast< float >( m_streamHz ) ) },
        { "stream delay", sensorWriteStreamDelay( &m_sensor, 0 ) },
        { "stream duration", sensorWriteStreamDuration( &m_sensor, 0 ) },
        { "header timestamp", sensorWriteHeaderTimestampEnabled( &m_sensor, 1 ) },
        { "start streaming", sensorStreamingStart( &m_sensor, &ImuReader::onSample ) },
    };
    for( const auto& [ name, err ] : steps )
    {
        if( err != TSS_SUCCESS )
        {
            std::cout << "Unable to set " << name << ": " << err << std::endl;
            sensorCleanup( &m_sensor );
            tss_com_close( com );
            return false;
        }
    }

    std::cout << "Streaming from sensor " << std::hex << m_sensor.serial_number << std::dec << std::endl;
    m_connected = true;
    m_lastSample = Clock::now();
    m_haveSample = false;
    m_heading = 0;
    m_stillTime = 0;
    m_untilPublish = 0;
    m_haveCompass = false;
    m_compassAxis = -1;
    return true;
}

void ImuReader::disconnect()
{
    sensorStreamingStop( &m_sensor );
    sensorCleanup( &m_sensor );
    tss_com_close( reinterpret_cast< TSS_Com_Class* >( &m_serial ) );
    m_connected = false;
}

void ImuReader::receive( const imu_msgs::ResetHeading& a_msg )
{
    std::cout << "ResetHeading " << a_msg.ShortDebugString() << std::endl;
    m_heading = a_msg.heading_deg() / toDegrees;
    // The compass starts again from the same heading, so the two can be compared from here.
    m_compass = m_heading;
}

TSS_DataCallbackState ImuReader::onSample( TSS_Sensor* a_sensor )
{
    float gyro[ 3 ];
    float accel[ 3 ];
    float mag[ 3 ];
    float motionless = 0;
    if( sensorProcessDataStreamingCallbackOutput( a_sensor, gyro, accel, mag, &motionless ) < 0 )
    {
        return TSS_DataCallbackStateError;
    }
    static_cast< ImuReader* >( a_sensor->user_data )->sample( gyro, accel, mag, motionless, sensorGetLastHeader( a_sensor ).timestamp );
    return TSS_DataCallbackStateProcessed;
}

void ImuReader::updateCompass( const Vector& a_mag )
{
    // The compass measures the field's angle from a sensor axis, so it works with the sensor mounted in any
    // orientation. The axis closest to horizontal is chosen once, a vertical one has no direction to measure.
    if( m_compassAxis < 0 )
    {
        m_compassAxis = 0;
        for( int i = 1; i < 3; i++ )
        {
            if( std::abs( m_up[ i ] ) < std::abs( m_up[ m_compassAxis ] ) )
            {
                m_compassAxis = i;
            }
        }
    }
    Vector axis{};
    axis[ m_compassAxis ] = 1;
    const Vector reference = horizontal( axis, m_up );
    const Vector field = horizontal( a_mag, m_up );
    if( std::sqrt( dot( field, field ) ) < minimumField || std::sqrt( dot( reference, reference ) ) < 0.1 )
    {
        return;
    }

    // The angle from the axis to the field, counter-clockwise around up. The field turns the other way when the
    // robot turns, so the heading is its negative.
    const double angle = -m_compassSign * std::atan2( dot( cross( reference, field ), m_up ), dot( reference, field ) );
    if( !m_haveCompass )
    {
        m_compass = m_heading;
        m_haveCompass = true;
    }
    else
    {
        // The shortest way from the last angle, so the heading keeps counting past whole turns.
        m_compass += std::remainder( angle - m_compassAngle, 2 * std::numbers::pi );
    }
    m_compassAngle = angle;
}

void ImuReader::sample( const float* a_gyro, const float* a_accel, const float* a_mag, float a_motionless, uint32_t a_timestamp )
{
    m_lastSample = Clock::now();
    // The gyro is in radians per second, the accelerometer in g and the magnetometer in gauss.
    const Vector gyro{ a_gyro[ 0 ], a_gyro[ 1 ], a_gyro[ 2 ] };
    const Vector accel{ a_accel[ 0 ], a_accel[ 1 ], a_accel[ 2 ] };
    const Vector mag{ a_mag[ 0 ], a_mag[ 1 ], a_mag[ 2 ] };

    // The sensor's clock wraps, which the unsigned difference handles. A long gap is not integrated over.
    double dt = 0;
    if( m_haveSample )
    {
        dt = static_cast< uint32_t >( a_timestamp - m_lastTimestamp ) * 1e-6;
        if( dt > 0.5 )
        {
            dt = 0;
        }
    }
    m_lastTimestamp = a_timestamp;

    // The heading is the rotation around gravity, so the sensor can be mounted in any orientation.
    const double length = std::sqrt( dot( accel, accel ) );
    if( length > 0.1 )
    {
        const Vector up = scaled( accel, 1 / length );
        if( !m_haveSample )
        {
            m_up = up;
        }
        else
        {
            const double share = dt / ( upFilter + dt );
            Vector filtered{ m_up[ 0 ] + ( up[ 0 ] - m_up[ 0 ] ) * share, m_up[ 1 ] + ( up[ 1 ] - m_up[ 1 ] ) * share, m_up[ 2 ] + ( up[ 2 ] - m_up[ 2 ] ) * share };
            m_up = scaled( filtered, 1 / std::sqrt( dot( filtered, filtered ) ) );
        }
    }
    m_haveSample = true;

    // While still the heading is kept and the offset follows the gyro, so it does not drift while waiting.
    const double rate = m_headingSign * dot( gyro, m_up );
    const bool calm = a_motionless >= motionlessConfidence && std::abs( rate - m_offset ) < stillRate;
    m_stillTime = calm ? m_stillTime + dt : 0;
    const bool still = m_stillTime >= stillDelay;
    if( still )
    {
        m_offset += ( rate - m_offset ) * dt / ( offsetFilter + dt );
    }
    else
    {
        m_heading += ( rate - m_offset ) * dt;
    }
    updateCompass( mag );

    m_untilPublish -= dt;
    if( m_untilPublish > 0 )
    {
        return;
    }
    m_untilPublish = std::max( m_untilPublish + 1 / m_publishHz, 0.0 );

    imu_msgs::ImuData msg;
    msg.set_heading_deg( m_heading * toDegrees );
    msg.set_yaw_rate_deg_s( still ? 0 : ( rate - m_offset ) * toDegrees );
    msg.set_gyro_x_deg_s( gyro[ 0 ] * toDegrees );
    msg.set_gyro_y_deg_s( gyro[ 1 ] * toDegrees );
    msg.set_gyro_z_deg_s( gyro[ 2 ] * toDegrees );
    msg.set_accel_x_g( accel[ 0 ] );
    msg.set_accel_y_g( accel[ 1 ] );
    msg.set_accel_z_g( accel[ 2 ] );
    msg.set_still( still );
    msg.set_yaw_offset_deg_s( m_offset * toDegrees );
    msg.set_timestamp_us( a_timestamp );
    const double strength = std::sqrt( dot( mag, mag ) );
    msg.set_compass_heading_deg( m_compass * toDegrees );
    msg.set_mag_x_gauss( mag[ 0 ] );
    msg.set_mag_y_gauss( mag[ 1 ] );
    msg.set_mag_z_gauss( mag[ 2 ] );
    msg.set_mag_strength_gauss( strength );
    msg.set_mag_dip_deg( strength > 0 ? std::asin( std::clamp( -dot( mag, m_up ) / strength, -1.0, 1.0 ) ) * toDegrees : 0 );
    sepia::comm2::Dispatcher< imu_msgs::ImuData >::send( &msg );
}
