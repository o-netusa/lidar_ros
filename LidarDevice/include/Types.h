/**************************************************************************
 * @file:  Types.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <inttypes.h>

enum ScanMode
{
    TWO_WAY = 0,
    ONE_WAY = 1
};

enum RawDataType : uint32_t
{
    FPGA = 0,
    DSP = 1
};

/**
 * @brief The ViewParameter struct
 * @note the rule of parameters according to LimitParameter.xml
 *   1.the sum of steps < max_steps
 *   2.perspectives[i] < perspectives[i+1]
 *   3.(perspectives[i+1] - perspectives[i]) / step[i] > single_step
 */
struct ViewParameter
{
    uint32_t frame{0};            ///< the frame frequency
    uint32_t steps[4] = {0};      ///< the interval steps
    float perspectives[5] = {0};  ///< frontier points
};

/**
 * @brief The LaserParameter struct
 */
struct LaserParameter
{
    uint32_t level{0};        ///< the power level in range 0~19 inclusive
    uint32_t factor{0};       ///< the frequency factor in range 1~10 inclusive
    uint32_t pulse_width{0};  ///< the pulse width in range 0~15 inclusize
};

/**
 * @brief The LidarParameter struct
 */
struct LidarParameter
{
    uint32_t echo_num{0};
    RawDataType type{FPGA};
    ViewParameter view;
    LaserParameter laser;
    // reserved
    uint32_t speed{0};
    uint32_t point_size{0};
};
