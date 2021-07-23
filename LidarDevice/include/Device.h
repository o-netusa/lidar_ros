/**************************************************************************
 * @file:  Device.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <common/Global.h>
#include <common/Semaphore.h>
#include <uuid.h>

#include <atomic>
#include <sigslot/signal.hpp>

#include "DeviceParams.h"

namespace onet { namespace lidar {

static constexpr char DEFAULT_DATA_FOLDER[]{"raw_data"};

/**
 * @brief The WriteRawDataOption struct
 *   Optional parameters to Start
 * @note Raw data means the raw data of LiDAR. It can be converted into the stardand point cloud
 * data.
 */
struct WriteRawDataOption
{
    /**
     * @brief The FolderRule enum
     *   DEFAULT : Generating the folder by date in format 'yyyymmdd'
     *   SPECIFIED : Generating the folder defined by user. Note the path should be legal
     *   EVERY_START: Generating the folder by time in format 'yyyymmddhhmmss' each time the Start
     * is called
     */
    enum FolderRule
    {
        DEFAULT = 0,
        SPECIFIED = 1,
        EVERY_START = 2
    };
    WriteRawDataOption(bool savable = false, FolderRule rule = DEFAULT,
                       const std::string &path = "")
        : savable(savable), rule(rule), path(path)
    {}

    bool savable;      ///< is raw data savable
    FolderRule rule;   ///< the rule of folder generation
    std::string path;  ///< the folder defined by user
};

/**
 * @brief The DeviceCallback class
 */
class DLLEXPORT DeviceCallback
{
public:
    /// It is used to specify how to handle the point cloud data per frame
    virtual void HandlePointCloud(uint32_t frame_id, std::shared_ptr<PointCloud> cloud,
                                  [[maybe_unused]] const std::string &file_name = {}) = 0;

    /// It is called when the playback is finished
    virtual void PlaybackDone(){};

    /// Disconnect all slots
    void DisconnectAll()
    {
        play_done_signal.disconnect_all();
        per_frame_signal.disconnect_all();
    }

    sigslot::signal<> play_done_signal;
    sigslot::signal<uint32_t, std::shared_ptr<PointCloud>, const std::string &> per_frame_signal;
};

/**
 * Base class for LiDAR device and playback device
 */
class DLLEXPORT Device
{
public:
    Device() = default;
    ~Device() = default;

    /**
     * @brief Get the uuid of device
     */
    virtual const uuids::uuid &GetDeviceId() const { return m_device_id; }

    /**
     * @brief Check if the device is started
     */
    virtual bool IsStarted() const { return m_started; }

    /**
     * @brief Initialize the device
     * @return true on success, false otherwise
     */
    virtual bool Init() = 0;

    /**
     * @brief Start the device
     * @param callback how to handle the point cloud data
     * @param option how to deal with the raw LiDAR data
     */
    virtual bool Start(std::shared_ptr<DeviceCallback> callback = nullptr,
                       [[maybe_unused]] const WriteRawDataOption &option = {}) = 0;

    /**
     * @brief Stop the device
     */
    virtual bool Stop() = 0;

protected:
    uuids::uuid m_device_id{uuids::uuid_random_generator{}()};
    std::atomic_bool m_started{false};
    cppbase::Semaphore m_start_sem{0};
};

}}  // namespace onet::lidar
