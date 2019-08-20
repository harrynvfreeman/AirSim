// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ArduRoverCarController_hpp
#define msr_airlib_ArduRoverCarController_hpp

#include "vehicles/car/api/CarApiBase.hpp"
#include "sensors/SensorCollection.hpp"

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/lidar/LidarBase.hpp"

#include "UdpSocket.hpp"

#include <sstream>
#include <iostream>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace msr { namespace airlib {

class ArduRoverApi : public CarApiBase {

public:
    ArduRoverApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory, 
                 const Kinematics::State& state, const Environment& environment, const msr::airlib::GeoPoint& home_geopoint)
    : CarApiBase(vehicle_setting, sensor_factory, state, environment),
      state_(state), home_geopoint_(home_geopoint)
    {
        connection_info_ = static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting)->connection_info;
        sensors_ = &getSensors();

        connect();
    }

    ~ArduRoverApi()
    {
        closeConnections();
    }

protected:
    virtual void resetImplementation() override
    {
        CarApiBase::resetImplementation();
    }

public:
    // Update sensor data & send to Ardupilot
    virtual void update() override
    {
        CarApiBase::update();

        sendSensors();
        recvRoverControl();
    }

    virtual const SensorCollection& getSensors() const override
    {
        return CarApiBase::getSensors();
    }

    // TODO:VehicleApiBase implementation
    virtual bool isApiControlEnabled() const override
    {
        // Return true so that CarPawnSim gets control message from external firmware and not keyboard
        return true;
    }

    virtual void enableApiControl(bool) override
    {
        Utils::log("enableApiControl() - Not Implemented", Utils::kLogLevelInfo);
    }

    virtual bool armDisarm(bool) override
    {
        Utils::log("armDisarm() - Not Implemented", Utils::kLogLevelInfo);
        return false;
    }

    virtual GeoPoint getHomeGeoPoint() const override
    {
        return home_geopoint_;
    }

    virtual void getStatusMessages(std::vector<std::string>&) override
    {
    	// Commented out so that terminal is not spammed with messages
        // Utils::log("getStatusMessages - Not Implemented", Utils::kLogLevelInfo);
    }

public:
    virtual void setCarControls(const CarControls& controls) override
    {
        lastRCData_ = controls;
    }

    virtual void updateCarState(const CarState& car_state) override
    {
        last_car_state_ = car_state;
    }

    virtual const CarState& getCarState() const override
    {
        return last_car_state_;
    }

    virtual const CarControls& getCarControls() const override
    {
        return lastCarControls_;
    }

protected:
    void closeConnections()
    {
        if (udpSocket_ != nullptr)
            udpSocket_->close();
    }

    void connect()
    {
        port = static_cast<uint16_t>(connection_info_.udp_port);
        ip = connection_info_.udp_address;

        closeConnections();

        if (ip == "") {
            throw std::invalid_argument("UdpIp setting is invalid.");
        }

        if (port == 0) {
            throw std::invalid_argument("UdpPort setting has an invalid value.");
        }

        Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);
        Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);

        udpSocket_ = std::make_shared<mavlinkcom::UdpSocket>();
        udpSocket_->bind(connection_info_.local_host_ip, connection_info_.control_port);
    }

    const GpsBase* getGps() const
    {
        return static_cast<const GpsBase*>(sensors_->getByType(SensorBase::SensorType::Gps));
    }
    const ImuBase* getImu() const
    {
        return static_cast<const ImuBase*>(sensors_->getByType(SensorBase::SensorType::Imu));
    }
    const MagnetometerBase* getMagnetometer() const
    {
        return static_cast<const MagnetometerBase*>(sensors_->getByType(SensorBase::SensorType::Magnetometer));
    }
    const BarometerBase* getBarometer() const
    {
        return static_cast<const BarometerBase*>(sensors_->getByType(SensorBase::SensorType::Barometer));
    }
    const LidarBase* getLidar() const
    {
        return static_cast<const LidarBase*>(sensors_->getByType(SensorBase::SensorType::Lidar));
    }

private:
    void recvRoverControl()
    {
        // Receive motor data
        RoverControlMessage pkt;
        int recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        while (recv_ret != sizeof(pkt)) {
            if (recv_ret <= 0) {
                Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
            } else {
                Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(pkt)), Utils::kLogLevelInfo);
            }
            
            recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        }

        lastCarControls_.throttle = pkt.throttle;
        lastCarControls_.steering = pkt.steering;

        if (pkt.throttle > 0) {
            lastCarControls_.is_manual_gear = false;
            lastCarControls_.manual_gear = 0;
        } else {
            lastCarControls_.is_manual_gear = true;
            lastCarControls_.manual_gear = -1;
        }
    }

    void sendSensors()
    {
        if (sensors_ == nullptr)
            return;

        const auto& gps_output = getGps()->getOutput();
        const auto& imu_output = getImu()->getOutput();
        // const auto& baro_output = getBarometer()->getOutput();
        // const auto& mag_output = getMagnetometer()->getOutput();

        std::ostringstream oss;

        const auto lidar = getLidar();
        // TODO: Add bool value in settings to check whether to send lidar data or not
        // Since it's possible that we don't want to send the lidar data to Ardupilot but still have the lidar (maybe as a ROS topic)
        if (lidar != nullptr) {
            const auto& lidar_output = lidar->getOutput();
            oss << ","
                   "\"lidar\": {"
                   "\"point_cloud\": [";

            std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(oss, ","));
            oss << "]}";
        }

        float yaw;
        float pitch;
        float roll;
        VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

        char buf[65000];

        // TODO: Split the following sensor packet formation into different parts for individual sensors

        // UDP packets have a maximum size limit of 65kB
        int ret = snprintf(buf, sizeof(buf),
                           "{"
                           "\"timestamp\": %" PRIu64 ","
                           "\"imu\": {" 
                           "\"angular_velocity\": [%.12f, %.12f, %.12f],"
                           "\"linear_acceleration\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"gps\": {"
                           "\"lat\": %.7f,"
                           "\"lon\": %.7f,"
                           "\"alt\": %.3f"
                           "},"
                           "\"velocity\": {"
                           "\"world_linear_velocity\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"pose\": {"
                           "\"roll\": %.12f,"
                           "\"pitch\": %.12f,"
                           "\"yaw\": %.12f"
                           "}"
                           "%s"
                           "}\n",
                           static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E3),
                           imu_output.angular_velocity[0],
                           imu_output.angular_velocity[1],
                           imu_output.angular_velocity[2],
                           imu_output.linear_acceleration[0],
                           imu_output.linear_acceleration[1],
                           imu_output.linear_acceleration[2],
                           gps_output.gnss.geo_point.latitude,
                           gps_output.gnss.geo_point.longitude,
                           gps_output.gnss.geo_point.altitude,
                           gps_output.gnss.velocity[0],
                           gps_output.gnss.velocity[1],
                           gps_output.gnss.velocity[2],
                           roll, pitch, yaw,
                           oss.str().c_str());

        if (ret == -1) {
            Utils::log("Error while allocating memory for sensor message", Utils::kLogLevelInfo);
            return;
        }
        else if (static_cast<uint>(ret) >= sizeof(buf)) {
            Utils::log(Utils::stringf("Sensor message truncated, lost %d bytes", ret - sizeof(buf)), Utils::kLogLevelInfo);
        }

        // Send data
        if (udpSocket_ != nullptr) {
            udpSocket_->sendto(buf, strlen(buf), ip, port);
        }
    }

private:
    struct RoverControlMessage {
        float throttle;
        float steering;
    };

    AirSimSettings::MavLinkConnectionInfo connection_info_;

    std::shared_ptr<mavlinkcom::UdpSocket> udpSocket_;

    uint16_t port;
    std::string ip;

    const SensorCollection* sensors_;

    CarControls lastCarControls_;
    CarControls lastRCData_;
    const Kinematics::State& state_;
    GeoPoint home_geopoint_;
    CarState last_car_state_;
};

}} // namespace

#endif
