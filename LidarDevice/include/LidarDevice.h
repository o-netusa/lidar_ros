/**************************************************************************
 * @file:  LidarDevice.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include "Device.h"
#include "Types.h"

namespace onet { namespace lidar {

/**
 * @brief The class is used to communicate with lidar. You can get point cloud
 * data and parameters of the device, and can also change the motion of the device.
 */
class DLLEXPORT LidarDevice : public Device
{
    class lidar_impl;

public:
    /**
     * @brief The unique constructor.
     * @param ip_addr the ip address of device
     * @param port_num the port number of device
     */
    LidarDevice(const std::string& ip_addr, uint32_t port_num);
    ~LidarDevice() = default;

    /**
     * @brief Get the port number of the device.
     * @return the port number
     */
    uint32_t GetPortNum() const;

    /**
     * @brief Get the ip address of the device.
     * @return the ip address
     */
    const std::string& GetIPAddress() const;

    /**
     * @brief Get the parameters of the device.
     *   If the Init function has not been called, the return value is incorrect.
     * @return the device parameter
     */
    const LidarParameter& GetLidarParameter() const;

    /**
     * @brief Initialize the device according to the configuration files.
     * @return true on success, false otherwise
     */
    bool Init() override;

    /**
     * @brief Start receiving the point cloud data.
     * @param callback how to deal with the point cloud data
     * @param option optional parameters to raw data
     * @return true on success, false otherwise
     */
    bool Start(std::shared_ptr<DeviceCallback> callback = nullptr,
               const WriteRawDataOption& option = {}) override;

    /**
     * @brief Stop the deviece and the point cloud data will no longer be received.
     * @return true on success, false otherwise
     */
    bool Stop() override;

    /**
     * @brief Get if the device has been started
     * @return true indicates the device is started, otherwise false
     */
    bool IsStarted() const override;

    /**
     * @brief Save lidar parameters
     * @return true on success, false otherwise
     */
    bool Save();

    /**
     * @brief Stop all jobs and make device uninitialized
     */
    void Reset();

    /**
     * @brief Set the view parameter of the device.
     *   An exception about device or network might be thrown.
     * @param param the struct of ViewParameter
     */
    void SetViewSpeed(const ViewParameter& param);

    /**
     * @brief Set the laser parameter of the device.
     *   An exception about device or network might be thrown.
     * @param param the struct of LaserParameter
     */
    void SetLaser(const LaserParameter& param);

    /**
     * @brief Set the scan parameter of the device.
     *   An exception about device or network might be thrown.
     * @param mode the enumeration of ScanMode
     */
    void SetScanMode(ScanMode mode);

    /**
     * @brief Set the format of raw data which will be parsed into point cloud data soon.
     *   An exception about device or network might be thrown.
     * @param type the enumeration of RawDataType
     */
    void SetRawDataType(RawDataType type);

    /**
     * @brief Set the echo parameter of the device.
     *   An exception about device or network might be thrown.
     * @param echo_number the number should be in range 1~4 inclusive
     */
    void SetEchoNumber(int32_t echo_number);

private:
    std::shared_ptr<lidar_impl> m_impl;
};

}}  // namespace onet::lidar
