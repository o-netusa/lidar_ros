/**************************************************************************
 * @file:  DeviceParams.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <Eigen/Core>
#include <vector>

#include "Types.h"

namespace onet { namespace lidar {

typedef std::vector<Eigen::Vector4f> PointCloud;

/**
 * @brief Dolphin Point Cloud file header struct
 */
struct DlphFileHeader
{
    // LidarParameter.xml
    uint32_t file_type{0};
    uint32_t frame_number{0};
    uint32_t echo_number{0};
    uint32_t lazer_type{0};
    uint32_t factor_number{0};
    uint32_t pulse_width{0};
    uint32_t step1{0};
    uint32_t step2{0};
    uint32_t step3{0};
    uint32_t step4{0};
    float angle1{0};
    float angle2{0};
    float angle3{0};
    float angle4{0};
    float angle5{0};
};

/**
 * @brief Dolphin Point Cloud packet struct
 */
struct DlphPacketHeader
{
    uint32_t sec;
    uint32_t msec;
    uint32_t capture_len;
    uint32_t len;
};

/**
 * @brief The raw data base struct
 */
struct RawDataBase
{
    uint32_t packet_header{0};
    uint16_t packet_info[6] = {0};
    int16_t packet_info_ext[6] = {0};
    uint32_t reserved{0};
};

/**
 * @brief The raw data with FPGA type struct
 */
struct DlphFPGAData : RawDataBase
{
    uint16_t angle_data[715] = {0};
    uint16_t packet_end[3] = {0};
};

/**
 * @brief The raw data with DSP type struct
 */
struct DlphDSPData : RawDataBase
{
    struct DSPPoint
    {
        uint16_t point_x{0};
        int16_t point_y{0};
        int16_t point_z{0};
        uint16_t intensity{0};
    };

    DSPPoint point_data[125];
    uint16_t packet_end[3] = {0};
};

// LidarCheckParameter.xml
struct DlphCheckParameter
{
    // KParameter
    float k{0};
    // CheckParameter
    float zero_range_left_min{0};
    float zero_range_left_max{0};
    float zero_range_right_min{0};
    float zero_range_right_max{0};
    float alpha_right{0};
    float theta_right{0};
    float roll_right{0};
    float pitch_right{0};
    float yaw_right{0};
    float alpha_left{0};
    float theta_left{0};
    float roll_left{0};
    float pitch_left{0};
    float yaw_left{0};
    float rotator_roll{0};
    float rotator_pitch{0};
    float rotator_yaw{0};
    float rotator_mirror_alpha[5] = {0};
    float rotator_mirror_theta[5] = {0};
    float galvano_mirror_right_theta{0};
    float galvano_mirror_left_theta{0};
    // RadianTransformParameter
    float a_left{0};
    float a_right{0};
    float b_left{0};
    float b_right{0};
    // CheckOffsetParameter
    float azimuth{0};
    float right_elevation{0};
    float left_elevation{0};
    // CheckAlgorithmParameter
    float coeff_left{0};
    float coeff2_left{0};
    float distance_left{0};
    float coeff_right{0};
    float coeff2_right{0};
    float distance_right{0};
};

struct ZeroRangeParameter
{
    float min{0};
    float max{0};
};

/**
 * @brief The DlphParameterTable struct
 *   It is used to record some pre-calculated data to make raw-data-parsing faster
 */
struct DlphParameterTable
{
    Eigen::Vector3f RrNr[20001];
    Eigen::Vector3f Nb[4096];
};

/**
 * @brief The DlphLimitParameter struct
 *   It is used to record the range of some parameters individually
 */
struct DlphLimitParameter
{
    // LimitParameter.xml
    ZeroRangeParameter echo_range;
    ZeroRangeParameter factor_range;
    ZeroRangeParameter time_range;
    int32_t max_step{0};
    int32_t max_speed{0};
    int32_t single_step_angle{0};
    int32_t distance{0};
    int32_t permission{0};
};

/**
 * @brief The DlphDeviceParameter struct
 */
struct DlphDeviceParameter
{
    uint32_t echo_number{0};
    LidarParameter lidar_param;
    DlphCheckParameter check_param;
    DlphLimitParameter limit_param;
    DlphParameterTable param_table_left;
    DlphParameterTable param_table_right;
    ZeroRangeParameter zero_range_left;
    ZeroRangeParameter zero_range_right;
    ZeroRangeParameter zero_range_view;
};

}}  // namespace onet::lidar
