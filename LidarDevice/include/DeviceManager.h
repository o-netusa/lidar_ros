/**************************************************************************
 * @file:  DeviceManager.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "Device.h"

namespace onet { namespace lidar {

class Device;

/**
 * @brief The DeviceManager class
 *   This singleton is used to maintain the devices, including query, creation and deletion
 */
class DLLEXPORT DeviceManager
{
public:
    DISALLOW_COPY_AND_ASSIGN(DeviceManager);
    ~DeviceManager() = default;

    /**
     * @brief Get the unique instance
     */
    static DeviceManager& GetInstance();

    /**
     * @brief Get the number of devices managed by this class
     */
    uint32_t GetDeviceCount() const;

    /**
     * @brief Get the device with the specified id
     * @return nullptr if id is inexistent
     */
    Device* GetDevice(const uuids::uuid& dev_id);

    /**
     * @brief Create a lidar device
     * @return an existing one if ip and port are in used, a new one otherwise
     */
    uuids::uuid CreateDevice(const std::string& ip_addr, uint32_t port_num);

    /**
     * @brief Create a playback device
     */
    uuids::uuid CreateDevice(const std::vector<std::string>& file_list);

    /**
     * @brief Release the resource of the device with the specified id
     */
    bool RemoveDevice(const uuids::uuid& dev_id);

private:
    DeviceManager();

private:
    std::vector<std::shared_ptr<Device>> m_devices;
};

}}  // namespace onet::lidar
