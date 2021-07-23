/**************************************************************************
 * @file:  PlaybackDevice.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <mutex>
#include <string>
#include <thread>

#include "Device.h"
#include "DeviceParams.h"

namespace onet { namespace lidar {

/**
 * @brief The PlaybackDevice class
 *   It is used to read point cloud data from files, including .dp, .bin, .pcd,
 * .xyz, .xyzn, .xyzrgb, .ply, .pts
 */
class DLLEXPORT PlaybackDevice : public Device
{
public:
    /**
     * @brief The constructor
     * @param file_list the point cloud files
     */
    PlaybackDevice(const std::vector<std::string>& file_list);
    ~PlaybackDevice();

    /**
     * @brief Get the files of point cloud
     */
    const std::vector<std::string>& GetFileList() const;

    /**
     * @brief Initialize the device
     */
    bool Init() override;

    /**
     * @brief Start reading files one by one
     * @param callback how to handle the point cloud data
     * @param option it is not used here
     * @return true on success, false otherwise
     */
    bool Start(std::shared_ptr<DeviceCallback> callback = nullptr,
               [[maybe_unused]] const WriteRawDataOption& option = {}) override;

    /**
     * @brief Stop reading
     */
    bool Stop() override;

    /**
     * @brief Control the reading process
     * @param is_paused  pause if the parameter is true, continue otherwise
     */
    void Pause(bool is_paused);

    /**
     * @brief Set parameter which is used to parse the raw data
     */
    void SetParameter(std::shared_ptr<DlphDeviceParameter> param);

    /**
     * @brief Get current parameter
     */
    const DlphDeviceParameter* GetParameter() const;

private:
    void PlaybackFunc(std::shared_ptr<DeviceCallback> callback);

private:
    std::vector<std::string> m_file_list;
    std::thread m_playback_thread;
    cppbase::Semaphore m_pause_sem{0};
    std::shared_ptr<DlphDeviceParameter> m_dev_param;
    bool m_paused{false};
    std::mutex m_pause_mutex;
};

}}  // namespace onet::lidar
