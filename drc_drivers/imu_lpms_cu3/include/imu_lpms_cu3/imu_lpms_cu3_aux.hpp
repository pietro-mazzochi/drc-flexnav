/*
works with premium IMU (experimental) and lpresearch lpms-cu3
VERSION 0.240924
SENAI DTI
*/

#ifndef DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_AUX_HPP_
#define DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_AUX_HPP_

#include <cstdio>
#include <cstdint>

namespace drc_flexnav_sensors
{
    void jsonCoord3dIntStr(char *dest, const char *name, const int32_t v1, const int32_t v2, const int32_t v3);

    void jsonCoord3dFloatStr(char *dest, const char *name, const float v1, const float v2, const float v3);

    void jsonCoord4dIntStr(char *dest, const char *name, const int32_t v1, const int32_t v2, const int32_t v3, const int32_t v4);

    void jsonCoord4dFloatStr(char *dest, const char *name, const float v1, const float v2, const float v3, const float v4);

    void jsonFloatStr(char *dest, const char *name, const float v1);

    void jsonIntStr(char *dest, const char *name, const int32_t v1);
    void jsonHexStr(char *dest, const char *name, const int32_t v1);

    void jsonStr(char *dest, const char *name, const int value);
}

#endif // DRC_FLEXNAV_SENSORS__IMU_LPMS_CU3_SENSOR_HPP_
