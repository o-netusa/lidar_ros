/**************************************************************************
 * @file:  Exceptions.h
 * @brief:
 *
 * Copyright (c) 2020-present O-Net Communications (ShenZhen) Limited.
 * All rights reserved.
 *
 *************************************************************************/

#pragma once

#include <exception>
#include <string>

namespace onet { namespace lidar {

#define EXCEPTION_FROM_STD(CUSTOM)                                           \
    class CUSTOM : public std::exception                                     \
    {                                                                        \
    public:                                                                  \
        explicit CUSTOM(const std::string &msg) : m_msg(msg) {}              \
        virtual ~CUSTOM() noexcept = default;                                \
        const char *what() const noexcept override { return m_msg.c_str(); } \
                                                                             \
    private:                                                                 \
        std::string m_msg;                                                   \
    };

#define EXCEPTION_FROM_CUSTOM(DETAIL, CUSTOM)                    \
    class DETAIL : public CUSTOM                                 \
    {                                                            \
    public:                                                      \
        explicit DETAIL(const std::string &msg) : CUSTOM(msg) {} \
        virtual ~DETAIL() noexcept = default;                    \
    };

// exceptions about network
EXCEPTION_FROM_STD(NetworkIssue)
EXCEPTION_FROM_CUSTOM(Timeout, NetworkIssue)
EXCEPTION_FROM_CUSTOM(SendFailure, NetworkIssue)
EXCEPTION_FROM_CUSTOM(RecvFailure, NetworkIssue)

// exceptions about io
EXCEPTION_FROM_STD(IOIssue)
EXCEPTION_FROM_CUSTOM(BadFile, IOIssue)
EXCEPTION_FROM_CUSTOM(NonexistentFile, IOIssue)
EXCEPTION_FROM_CUSTOM(UnsupportedFile, IOIssue)

// exceptions about device
EXCEPTION_FROM_STD(DeviceIssue)
EXCEPTION_FROM_CUSTOM(InvalidParameter, DeviceIssue)
EXCEPTION_FROM_CUSTOM(InvalidOperation, DeviceIssue)

}}  // namespace onet::lidar
